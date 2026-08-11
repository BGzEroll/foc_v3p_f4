# I2C 总线驱动设计与中断调用说明

## 1. 结论

当前 `i2c_bus` 是一个面向 FreeRTOS 任务的同步 I2C 寄存器访问接口，底层使用 DMA 搬运数据。

它的关键结论如下：

- `read_bytes()` 和 `write_bytes()` 只能在调度器已经运行的任务上下文调用。
- 不能从普通外设中断、定时器中断、ADC 中断或 FOC 高频中断中直接调用这两个接口。
- 从调用者视角看，这两个接口是阻塞式接口；等待期间任务会进入阻塞态，不会占用 CPU 空转。
- DMA 完成回调和 I2C 错误回调本身运行在中断上下文，并使用 FreeRTOS 的 `FromISR` API 唤醒等待任务。
- I2C1 和 I2C2 拥有相互独立的 Mutex、完成信号量及 DMA 状态，可以并行工作。
- 同一条物理总线上的多个 `i2c_bus` 对象共享底层资源，会被同一把 Mutex 串行化。
- FOC 路径需要 I2C 数据时，推荐由独立 I2C 任务提前采样，FOC 中断只读取最近一次完整快照。

相关实现：

- [`i2c_bus.h`](../user_lib/drivers/bus/i2c_bus.h)
- [`i2c_bus.cpp`](../user_lib/drivers/bus/i2c_bus.cpp)
- [`i2c.c`](../Core/Src/i2c.c)
- [`dma.c`](../Core/Src/dma.c)
- [`FreeRTOSConfig.h`](../Core/Inc/FreeRTOSConfig.h)

## 2. 当前硬件配置

两条总线均配置为 100 kHz、7 位地址模式，RX 和 TX 都使用 DMA Normal 模式。

| 总线编号 | HAL 外设 | 引脚 | RX DMA | TX DMA | DMA/I2C Event/Error IRQ 优先级 |
|---:|---|---|---|---|---:|
| `0` | I2C1 | PB6 SCL、PB7 SDA | DMA1 Stream0 Channel1 | DMA1 Stream6 Channel1 | 5 |
| `1` | I2C2 | PB10 SCL、PB11 SDA | DMA1 Stream2 Channel7 | DMA1 Stream7 Channel7 | 5 |

驱动初始化时会同时检查 `hdmarx` 和 `hdmatx`，因此任意方向的 DMA 未正确链接都会返回 `INIT_FAILED`。

当前 FreeRTOS 的关键配置为：

| 配置 | 当前值 | 对驱动的影响 |
|---|---:|---|
| `configTICK_RATE_HZ` | 1000 | 一个 tick 为 1 ms |
| `configSUPPORT_STATIC_ALLOCATION` | 1 | Mutex 和信号量可以使用静态内存 |
| `configUSE_MUTEXES` | 1 | 支持总线互斥访问 |
| `INCLUDE_xTaskGetSchedulerState` | 1 | 驱动可以检查调度器状态 |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 5 | 数字优先级 5～15 的 ISR 才能调用 FreeRTOS `FromISR` API |

I2C DMA、Event IRQ 和 Error IRQ 当前都设置为数字优先级 5，正好位于允许调用 `xSemaphoreGiveFromISR()` 的最高优先级边界。

## 3. 软件框架

### 3.1 分层关系

```text
设备驱动或业务任务
        │
        │ read_bytes() / write_bytes()
        ▼
公开 i2c_bus 对象
        │
        │ bus_id：0 或 1
        ▼
物理总线 i2c_dev 资源池
        ├── HAL I2C 句柄
        ├── Mutex
        ├── DMA 完成二值信号量
        ├── transfer_active
        └── transfer_result
        │
        │ HAL_I2C_Mem_Read_DMA()
        │ HAL_I2C_Mem_Write_DMA()
        ▼
STM32 HAL + I2C 外设 + DMA
        │
        │ DMA 完成或 I2C Event/Error IRQ
        ▼
HAL 回调 → xSemaphoreGiveFromISR() → 唤醒原任务
```

`i2c_bus` 是公开的轻量入口，只保存 `bus_id`。真正拥有 HAL 句柄和 RTOS 同步资源的是 `i2c_bus.cpp` 内部的 `i2c_dev`。

当前资源池固定映射为：

```cpp
static i2c_dev i2c_devs[] =
{
    i2c_dev(&hi2c1),
    i2c_dev(&hi2c2)
};
```

因此可以创建多个指向同一总线的公开对象：

```cpp
static i2c_bus mpu_bus(0);
static i2c_bus encoder_bus(0);
```

这两个对象最终都会访问 I2C1 的同一个 `i2c_dev`，所以不会绕开总线 Mutex。

### 3.2 初始化

`i2c_bus::init()` 最终会完成以下工作：

1. 检查总线编号是否有效。
2. 检查 HAL I2C 实例以及 RX、TX DMA 句柄是否存在。
3. 使用静态内存创建 Mutex。
4. 使用静态内存创建二值完成信号量。
5. 标记物理总线已经初始化。

初始化过程不从 FreeRTOS heap 分配内存，多次调用同一条总线的 `init()` 会直接返回 `OK`。

推荐在系统初始化阶段、调度器启动前完成设备对象和 I2C 同步资源初始化。实际 I2C 读写必须等调度器运行后再由任务发起。

### 3.3 一次 DMA 读写的完整时序

以 `read_bytes()` 为例：

1. 检查驱动是否已经初始化。
2. 检查 7 位设备地址、缓冲区和数据长度。
3. 检查当前不是 ISR，并确认 FreeRTOS 调度器正在运行。
4. 获取物理总线 Mutex，防止同一条总线同时发起两笔传输。
5. 清除完成信号量中可能残留的旧事件。
6. 设置 `transfer_active` 和初始传输状态。
7. 调用 `HAL_I2C_Mem_Read_DMA()` 启动传输。
8. 当前任务阻塞等待完成信号量，其他任务可以继续运行。
9. DMA 完成后进入 HAL 回调。
10. 回调写入传输结果，通过 `xSemaphoreGiveFromISR()` 唤醒任务。
11. 被唤醒的任务读取结果并释放 Mutex。

写入路径的区别仅在于使用 `HAL_I2C_Mem_Write_DMA()` 和 TX DMA。

### 3.4 “同步 API + DMA”的含义

该驱动不是异步请求队列，公开 API 要等本次传输成功、失败或超时后才返回：

```cpp
i2c_result result = bus.read_bytes(
    0x68,
    0x75,
    &device_id,
    1);

// 执行到这里时，本次 DMA 已经结束或已经进入错误处理。
```

它仍然使用 DMA，但 DMA 期间阻塞的是当前任务，不是整个 CPU。这样的接口具有以下优点：

- 设备驱动的控制流程简单。
- 缓冲区生命周期容易保证。
- 不需要每个设备再维护一套完成状态机。
- 任务阻塞期间不会忙等待。

它也意味着调用任务可能阻塞最多“等待 Mutex 时间 + DMA 传输时间”，不适合硬实时 ISR 路径。

## 4. 公共 API 说明

### 4.1 构造函数

```cpp
explicit i2c_bus(uint8_t bus_id = 0);
```

| `bus_id` | 对应外设 |
|---:|---|
| 0 | I2C1 |
| 1 | I2C2 |

其他编号会在 `init()` 或读写时返回 `INVALID_BUS`，不会静默回退到 I2C1。

### 4.2 初始化

```cpp
i2c_result init();
```

初始化只创建同步资源，不发起总线传输。

### 4.3 寄存器读取

```cpp
i2c_result read_bytes(uint8_t device_address,
    uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS,
    uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
```

### 4.4 寄存器写入

```cpp
i2c_result write_bytes(uint8_t device_address,
    uint8_t register_address,
    const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS,
    uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
```

参数注意事项：

- `device_address` 填写未左移的 7 位地址，例如 MPU 地址 `0x68`。
- 驱动内部会执行 `device_address << 1`，调用方不能传 `0xD0`。
- 当前寄存器地址固定为 8 位，底层使用 `I2C_MEMADD_SIZE_8BIT`。
- 当前接口表达的是“设备寄存器读写”，不支持无寄存器地址的原始 I2C 收发。
- `data` 在函数返回前必须保持有效。由于当前 API 会等待完成，任务栈上的局部数组可以安全使用。
- 默认 Mutex 等待时间为 10 ms，默认传输完成等待时间为 50 ms。

## 5. 返回值

| 返回值 | 含义 | 常见原因 |
|---|---|---|
| `OK` | 传输成功 | DMA 完成回调正常到达 |
| `INVALID_BUS` | 总线编号无效 | `bus_id` 不是 0 或 1 |
| `INVALID_ARGUMENT` | 参数无效 | 地址超过 7 位、空指针或长度为 0 |
| `NOT_INITIALIZED` | 尚未初始化 | 未调用 `init()` 或恢复失败后失效 |
| `INVALID_CONTEXT` | 调用上下文无效 | ISR 调用或调度器尚未运行 |
| `INIT_FAILED` | 同步资源或 DMA 配置无效 | RX/TX DMA 未链接等 |
| `LOCK_TIMEOUT` | 获取总线 Mutex 超时 | 其他任务长时间占用同一总线 |
| `TRANSFER_TIMEOUT` | DMA/总线操作超时 | 从机无响应、总线异常、完成中断未到达 |
| `BUSY` | HAL I2C 当前忙 | HAL 状态未回到 Ready 等 |
| `NACK` | 从机未应答 | 地址错误、器件未上电、寄存器操作不接受 |
| `DMA_ERROR` | DMA 错误 | DMA 启动或运行异常 |
| `BUS_ERROR` | 其他 I2C 总线错误 | Arbitration Lost、Bus Error 等 |
| `RECOVERY_FAILED` | HAL 重新初始化失败 | `HAL_I2C_DeInit/Init` 未成功 |

`RECOVERY_FAILED` 只表示 HAL 外设重新初始化失败。即使 HAL 重新初始化成功，如果外部从机一直拉低 SDA，物理总线也可能仍未真正恢复。需要处理“时钟脉冲释放 SDA”一类场景时，应另行实现 GPIO 级总线恢复。

## 6. 错误和超时恢复

以下情况会触发恢复：

- HAL DMA 传输启动立即失败。
- 等待 DMA 完成信号量超时。

恢复流程为：

1. 在任务临界区内清除 `transfer_active`。
2. 调用 `HAL_I2C_DeInit()`。
3. 调用 `HAL_I2C_Init()`，重新配置 I2C、DMA 链接和 Event/Error IRQ。
4. 清除可能迟到的完成信号量。
5. 最后释放总线 Mutex。

恢复期间 Mutex 始终由当前任务持有，其他设备不会在总线状态尚未稳定时插入新传输。

当前驱动不会自动重试原始读写。上层设备驱动可以根据返回值决定是否重试，但应限制次数并留出间隔，避免器件掉线时持续占用 CPU 和总线。

## 7. 是否支持中断中调用

### 7.1 API 上下文矩阵

| 接口或路径 | 任务上下文 | ISR 上下文 | 说明 |
|---|---:|---:|---|
| `i2c_bus::init()` | 支持，建议仅启动阶段调用 | 不支持 | 创建 FreeRTOS 同步对象 |
| `i2c_bus::read_bytes()` | 支持 | 不支持 | 会获取 Mutex 并等待完成信号量 |
| `i2c_bus::write_bytes()` | 支持 | 不支持 | 会获取 Mutex 并等待完成信号量 |
| `HAL_I2C_MemRxCpltCallback()` | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部完成路径 |
| `HAL_I2C_MemTxCpltCallback()` | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部完成路径 |
| `HAL_I2C_ErrorCallback()` | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部错误路径 |

驱动在传输入口中进行了明确检查：

```cpp
if(__get_IPSR() != 0U ||
    xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
{
    return i2c_result::INVALID_CONTEXT;
}
```

因此从任何 ISR 直接调用 `read_bytes()` 或 `write_bytes()`，都会立即返回 `INVALID_CONTEXT`，不会启动 DMA。

### 7.2 为什么 DMA 接口仍不能从 ISR 发起

“底层使用 DMA”和“API 可以在 ISR 调用”是两件不同的事情。

当前公共接口在启动 DMA 前后包含以下任务专用操作：

- `xSemaphoreTake(mutex, timeout)`：Mutex 没有可供 ISR 使用的获取版本。
- `xSemaphoreTake(completion_semaphore, timeout)`：ISR 不能阻塞等待传输完成。
- `taskENTER_CRITICAL()`：当前取消和恢复流程按任务上下文设计。
- `HAL_I2C_DeInit()` / `HAL_I2C_Init()`：错误恢复不应在实时 ISR 内执行。

此外，STM32F4 HAL 的 `HAL_I2C_Mem_Read_DMA()` 和 `HAL_I2C_Mem_Write_DMA()` 并不是从函数入口开始就完全异步。HAL 在启动数据 DMA 前还会同步处理 BUSY、START、地址和寄存器地址阶段，并等待相应硬件标志。这段时间会直接占用调用上下文。

所以即使删除 `INVALID_CONTEXT` 检查并改用其他锁，直接从 FOC ISR 调用 HAL DMA 启动函数仍可能给控制周期引入不可控抖动。

### 7.3 FOC 路径为什么尤其不适合直接访问 I2C

FOC 的 ADC/PWM 中断通常具有严格、固定的执行预算。I2C 当前只有 100 kHz，单次寄存器访问涉及起始条件、设备地址、寄存器地址、数据和应答位，其时间尺度远大于普通内存访问。

如果在 FOC ISR 中直接操作 I2C，可能产生：

- FOC 周期抖动。
- 延迟更新 PWM 比较值。
- ADC 下一次采样到来前仍未退出中断。
- 更高优先级中断被不必要地延迟。
- HAL 内部等待硬件标志时卡住整个控制路径。
- I2C 异常时触发几十毫秒级超时，这是 FOC ISR 完全不能接受的。

因此当前驱动主动拒绝 ISR 调用是必要的保护，不是 DMA 功能缺失。

### 7.4 当前工程中的 FOC 相关说明

当前 `user_lib` 中还没有实际 FOC 控制实现，也没有能够据此确认的 FOC ADC/PWM 回调，因此本文不能把某一个现有函数直接认定为 FOC 中断入口。

不过当前 `project.ioc` 中的 TIM1 Update、TIM8 Update 以及已启用的 DMA 中断数字优先级均为 5。如果以后 FOC 路径运行在这些优先级未被修改的中断中，它可以调用符合规则的 FreeRTOS `FromISR` 通知接口，但仍然不能调用当前 `i2c_bus::read_bytes()` 或 `write_bytes()`。

如果以后为了降低 FOC 延迟而把其中断调整为数字优先级 0～4，该中断将高于 FreeRTOS 的系统调用边界，不能调用任何 FreeRTOS `FromISR` API。此时应让 I2C 任务固定周期采样，FOC ISR 只读取已经发布的数据快照。

## 8. FOC 使用 I2C 数据的推荐架构

### 8.1 推荐方案：I2C 任务提前采样，FOC 读取快照

```text
I2C 采样任务
    │
    ├── 周期调用 i2c_bus::read_bytes()
    ├── 校验结果
    └── 发布完整传感器快照
                    │
                    ▼
             最新数据双缓冲
                    │
                    ▼
FOC ADC/PWM ISR ── 只读取最近一次完整快照
```

这种方式下：

- I2C 超时只会阻塞 I2C 任务。
- FOC ISR 的执行时间不依赖 I2C 总线状态。
- I2C 采样频率和 FOC 频率可以解耦。
- FOC 可以检查快照时间戳，发现数据过期后进入降级或保护逻辑。

快照建议包含：

```cpp
struct sensor_snapshot
{
    uint32_t timestamp_ms = 0;
    int16_t axis_x = 0;
    int16_t axis_y = 0;
    int16_t axis_z = 0;
    bool valid = false;
};
```

不要仅用一个 `volatile bool ready` 保护多字段结构体。任务更新多个字段时，FOC ISR 可能读到一半旧、一半新的数据。推荐使用双缓冲，在完整写入备用缓冲后一次性切换活动索引。

### 8.2 示例：任务中周期读取

下面示例演示设备任务如何调用当前驱动。寄存器和数据格式仅作结构示例，需要替换为实际器件定义。

```cpp
#include "devices/mpu.h"

#include "drivers/bus/i2c_bus.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

static constexpr uint8_t MPU_I2C_ADDRESS = 0x68;
static constexpr uint8_t MPU_DATA_REGISTER = 0x3B;
static constexpr uint32_t MPU_SAMPLE_PERIOD_MS = 2;
static constexpr uint16_t MPU_TASK_STACK_DEPTH = 256;
static constexpr UBaseType_t MPU_TASK_PRIORITY = tskIDLE_PRIORITY + 3U;

static i2c_bus mpu_bus(0);

/**
 * @brief MPU 周期采样任务
 *
 * @param argument FreeRTOS 任务参数
 */
static void mpu_task_entry(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        uint8_t raw_data[6]{};
        i2c_result result = mpu_bus.read_bytes(
            MPU_I2C_ADDRESS,
            MPU_DATA_REGISTER,
            raw_data,
            sizeof(raw_data));

        if(result == i2c_result::OK)
        {
            // 解析数据，并把完整结果发布到快照缓冲。
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(MPU_SAMPLE_PERIOD_MS));
    }
}

/**
 * @brief 初始化 MPU 总线资源和采样任务
 */
void mpu::init()
{
    i2c_result result = mpu_bus.init();
    if(result != i2c_result::OK)
    {
        Error_Handler();
    }

    BaseType_t task_result = xTaskCreate(mpu_task_entry,
        "mpu",
        MPU_TASK_STACK_DEPTH,
        nullptr,
        MPU_TASK_PRIORITY,
        nullptr);

    if(task_result != pdPASS)
    {
        Error_Handler();
    }
}
```

`mpu::init()` 可以由 `start_init_all()` 调用。此时只创建静态 I2C 同步资源和任务；真正的 `read_bytes()` 会等调度器启动、任务开始运行后才执行。

### 8.3 按需方案：FOC ISR 只通知 I2C 工作任务

如果确实需要由某个控制周期触发一次 I2C 采样，可以让 ISR 只发送任务通知：

```cpp
static TaskHandle_t i2c_sample_task_handle = nullptr;

/**
 * @brief 从允许调用 FreeRTOS API 的 ISR 请求一次 I2C 采样
 */
void request_i2c_sample_from_isr()
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(i2c_sample_task_handle,
        &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 按需执行 I2C 采样的任务入口
 *
 * @param argument FreeRTOS 任务参数
 */
static void i2c_sample_task_entry(void *argument)
{
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 在任务上下文调用 read_bytes()，并发布采样结果。
    }
}
```

这里仍然没有从 ISR 调用 `i2c_bus`，ISR 只负责产生一个低开销事件。

必须同时满足以下条件：

- `i2c_sample_task_handle` 已经有效。
- 发出通知的中断数字优先级为 5～15。
- 中断中只调用 `...FromISR()` 版本的 FreeRTOS API。
- 采样任务拥有自己的固定缓冲区，不能使用 ISR 栈上临时缓冲区。

如果 FOC 中断以后被配置为数字优先级 0～4，就连 `vTaskNotifyGiveFromISR()` 也不能调用。此时更适合让 I2C 任务按照固定周期自主采样，FOC 中断只读快照。

## 9. 当前限制和后续扩展点

### 9.1 当前限制

- 只支持 I2C1 和 I2C2。
- 只支持 7 位设备地址。
- 只支持 8 位寄存器地址。
- 只提供寄存器式 Memory Read/Write，不提供原始 Master Transmit/Receive。
- 公开 API 是任务同步接口，没有异步请求队列。
- 不支持从 ISR 发起传输。
- 没有自动重试策略。
- HAL 重新初始化不能保证释放被从机持续拉低的 SDA。
- `init()` 按系统启动阶段设计，不应由多个任务并发调用。

### 9.2 如果以后需要异步接口

不建议把当前 `read_bytes()` 简单改造成 ISR API。更合理的扩展方式是新增独立的 I2C Worker：

```text
任务或低优先级 ISR
        │
        │ 提交固定大小请求或发送任务通知
        ▼
I2C Worker Task
        │
        │ 调用现有同步 i2c_bus API
        ▼
完成后更新设备状态、快照或通知请求任务
```

这样可以继续复用当前经过 Mutex、DMA 完成信号量和错误恢复保护的底层实现，同时把异步调度放在更适合的上层。

请求结构不能随意携带指向 ISR 栈或调用者临时栈的缓冲区指针。更安全的做法是使用固定大小数据副本、静态缓冲池，或者让 Worker 按设备编号执行预定义操作。

## 10. 使用检查清单

接入一个新 I2C 设备时，建议逐项确认：

- 使用的是 7 位地址，没有提前左移。
- 调用了对应 `i2c_bus` 的 `init()`。
- `read_bytes()` / `write_bytes()` 只从任务调用。
- 设备寄存器地址确实是 8 位。
- 默认 10 ms 锁超时和 50 ms 传输超时适合该设备。
- 检查并处理每次调用返回的 `i2c_result`。
- 不在 FOC、ADC、PWM 或定时器 ISR 中直接访问 I2C。
- FOC 使用的数据通过完整快照发布，并检查时间戳和有效标志。
- 如果 ISR 使用任务通知，确认其中断数字优先级不小于 5。
- 示波器或逻辑分析仪验证实际 SCL、SDA 时序和上拉是否正常。

## 11. 总体评价

当前实现适合“多个 FreeRTOS 设备任务共享一条 I2C 总线”的场景。它用 Mutex 解决任务间总线竞争，用 DMA 降低数据搬运开销，用完成信号量把 HAL 回调转换成任务同步结果，并对超时和 HAL 错误进行了集中恢复。

对于 FOC 这类硬实时路径，当前不支持 ISR 直接调用是合理设计。推荐保持 I2C 在任务域内运行，通过周期采样或任务通知更新快照，让 FOC ISR 只消费已经准备好的数据。
