# SPI 总线驱动设计与中断调用说明

## 1. 结论

当前 `spi_bus` 已重构为面向 FreeRTOS 任务的同步 SPI DMA 接口，支持 DMA 接收、DMA 发送和 DMA 全双工收发。

主要结论如下：

- SPI1 的 RX 和 TX DMA 均已启用。
- `rx()`、`tx()`、`rx_tx()` 从调用者视角仍是同步接口，内部任务会阻塞等待 DMA 完成信号量。
- 阻塞期间当前任务不占用 CPU 忙等待，其他任务可以继续运行。
- Mutex 覆盖从 `cs_low()` 到 `cs_high()` 的完整片选事务，而不是只覆盖一次 DMA 调用。
- 同一片选周期内可以连续执行多段发送、接收或全双工收发，其他任务不能插入。
- 公共 API 只能在调度器已经运行的任务上下文调用，不能从 FOC、ADC、PWM 或普通外设 ISR 直接调用。
- SPI DMA 完成回调和 SPI 错误回调属于内部 ISR 路径，通过 `xSemaphoreGiveFromISR()` 唤醒等待任务。
- FOC 需要 SPI 传感器数据时，推荐由独立 SPI 任务提前采样并发布完整快照，FOC ISR 只读取快照。

相关实现：

- [`spi_bus.h`](../user_lib/drivers/bus/spi_bus.h)
- [`spi_bus.cpp`](../user_lib/drivers/bus/spi_bus.cpp)
- [`spi.c`](../Core/Src/spi.c)
- [`dma.c`](../Core/Src/dma.c)
- [`stm32f4xx_it.c`](../Core/Src/stm32f4xx_it.c)
- [`FreeRTOSConfig.h`](../Core/Inc/FreeRTOSConfig.h)

## 2. 当前硬件配置

当前工程只配置了一条 SPI 总线：

| 项目 | 当前配置 |
|---|---|
| 总线编号 | `0` |
| HAL 外设 | SPI1 |
| 工作模式 | Master、2 Lines Full Duplex |
| 数据宽度 | 8 bit |
| SPI 模式 | Mode 0，CPOL Low、CPHA 1 Edge |
| 位序 | MSB First |
| NSS | Software |
| 波特率分频 | 16 |
| CubeMX 计算速率 | 5.25 Mbit/s |
| SCK | PA5 |
| MISO | PA6 |
| MOSI | PA7 |
| RX DMA | DMA2 Stream0 Channel3 |
| TX DMA | DMA2 Stream3 Channel3 |
| DMA 模式 | Normal |
| DMA 数据宽度 | Byte |
| DMA 和 SPI1 IRQ 数字优先级 | 5 |

当前工程中的 `OLED_CS` 位于 PA4，但 `spi_bus` 并未把某个固定 CS 写死到总线层。具体设备在调用 `cs_low()` 时传入自己的 GPIO 端口和引脚，因此多个 SPI 从设备可以共享 SPI1。

## 3. 软件框架

### 3.1 分层关系

```text
设备驱动或业务任务
        │
        │ cs_low() / tx() / rx() / rx_tx() / cs_high()
        ▼
公开 spi_bus 对象
        │
        │ bus_id：当前仅支持 0
        ▼
物理总线 spi_dev 资源池
        ├── HAL SPI 句柄
        ├── 事务 Mutex
        ├── DMA 完成二值信号量
        ├── 当前事务所有者任务
        ├── 当前 CS 端口和引脚
        ├── transfer_active
        └── transfer_result
        │
        │ HAL_SPI_Receive_DMA()
        │ HAL_SPI_Transmit_DMA()
        │ HAL_SPI_TransmitReceive_DMA()
        ▼
STM32 HAL + SPI1 + RX/TX DMA
        │
        │ DMA 完成或 SPI1 Error IRQ
        ▼
HAL 回调 → xSemaphoreGiveFromISR() → 唤醒原任务
```

`spi_bus` 是公开入口，只保存 `bus_id`。真正拥有 HAL 句柄、Mutex、信号量和事务状态的是 `spi_bus.cpp` 内部的 `spi_dev`。

当前物理资源池为：

```cpp
static spi_dev spi_devs[] =
{
    spi_dev(&hspi1)
};
```

多个 `spi_bus(0)` 对象最终共享同一个 `spi_dev`，因此不会绕开 SPI1 的事务 Mutex。

### 3.2 为什么 Mutex 必须覆盖 CS 周期

SPI 设备经常需要在同一个 CS 低电平期间完成多段操作，例如：

```text
CS Low → 发送寄存器地址 → 接收寄存器数据 → CS High
```

如果 Mutex 只保护单独一次 `tx()` 或 `rx()`，可能出现以下错误交错：

```text
任务 A：CS_A Low
任务 A：发送寄存器地址
任务 B：CS_B Low
任务 B：发送其他命令
任务 A：接收数据
```

这会让 SPI 时钟和 MOSI 数据同时送到两个处于选中状态的从设备，或者破坏任务 A 的协议状态机。

当前实现采用以下规则：

- `cs_low()` 获取物理总线 Mutex，然后拉低指定 CS。
- `tx()`、`rx()`、`rx_tx()` 只允许由持有本次事务的任务调用。
- `cs_high()` 拉高同一个 CS，然后释放 Mutex。
- 其他任务的 `cs_low()` 会等待 Mutex，而不能插入当前事务。

因此一次事务可以安全包含多段 DMA 传输。

### 3.3 初始化

`spi_bus::init()` 会完成：

1. 检查总线编号。
2. 检查 SPI 实例以及 RX、TX DMA 句柄。
3. 使用静态内存创建 Mutex。
4. 使用静态内存创建二值完成信号量。
5. 标记物理 SPI 总线已经初始化。

初始化不会从 FreeRTOS heap 分配内存。对同一条总线重复调用 `init()` 会直接返回 `OK`。

可以在 `start_init_all()` 调用的设备初始化函数中执行 `spi_bus::init()`；实际片选和 DMA 传输必须等调度器启动后再由任务调用。

## 4. 一次事务的执行时序

### 4.1 建立事务

任务调用 `cs_low(port, pin)`：

1. 检查 SPI 驱动是否初始化。
2. 检查 GPIO 端口和引脚参数。
3. 检查当前不是 ISR，并确认调度器正在运行。
4. 等待获取 SPI 总线 Mutex。
5. 记录当前任务为事务所有者。
6. 记录本次事务的 CS 端口和引脚。
7. 拉低 CS。

### 4.2 DMA 传输

任务调用 `tx()`、`rx()` 或 `rx_tx()`：

1. 检查参数和任务上下文。
2. 检查当前任务是否为事务所有者。
3. 清除完成信号量中可能残留的旧事件。
4. 设置 DMA 活动标志和初始结果。
5. 调用对应 HAL DMA API。
6. 当前任务阻塞等待完成信号量。
7. DMA 完成或 SPI 错误回调记录结果。
8. 回调通过 `xSemaphoreGiveFromISR()` 唤醒任务。
9. 任务恢复运行并返回 `spi_result`。

传输完成后 Mutex 仍然由当前任务持有，可以继续发起下一段传输。

### 4.3 结束事务

任务调用 `cs_high(port, pin)`：

1. 检查当前任务是否为事务所有者。
2. 检查传入的端口和引脚是否与 `cs_low()` 一致。
3. 拉高 CS。
4. 清除事务所有者和 CS 状态。
5. 释放 Mutex。

即使中间某次 DMA 返回错误，也应调用 `cs_high()`，否则 CS 会保持低电平并且总线 Mutex 不会释放。

## 5. 公共 API

### 5.1 构造和初始化

```cpp
explicit spi_bus(uint8_t bus_id = 0);
spi_result init();
```

当前只有 `bus_id == 0` 有效，对应 SPI1。其他编号返回 `INVALID_BUS`，不会静默回退到 SPI1。

### 5.2 开始和结束片选事务

```cpp
spi_result cs_low(GPIO_TypeDef *port,
    uint16_t pin,
    uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS);

spi_result cs_high(GPIO_TypeDef *port, uint16_t pin);
```

默认等待 Mutex 的时间为 10 ms。

当前只支持低电平有效的 GPIO 软件片选。`cs_high()` 必须由调用 `cs_low()` 的同一个任务执行，并传入相同端口和引脚。

### 5.3 DMA 接收

```cpp
spi_result rx(uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
```

SPI Master 要接收数据必须同时发送数据以产生 SCK。STM32 HAL 在 2 Lines Master 模式下会使用同一个缓冲区执行全双工 DMA，因此驱动会先把接收缓冲区填充为 `0xFF`，这些字节作为 Dummy Data 发出，然后被接收结果覆盖。

如果从设备要求特定 Dummy Byte，或者返回数据与命令发送严格对齐，应改用 `rx_tx()` 明确提供发送缓冲区。

### 5.4 DMA 发送

```cpp
spi_result tx(const uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
```

在 2 Lines Full Duplex 模式下，发送过程中物理上仍会收到数据，但 `tx()` 不向调用者提供这些返回字节。

### 5.5 DMA 全双工收发

```cpp
spi_result rx_tx(const uint8_t *tx_data,
    uint8_t *rx_data,
    uint16_t size,
    uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
```

每发送一个字节就接收一个字节，适合：

- SPI 编码器。
- ADC 或传感器流水线响应。
- 需要明确控制 Dummy Byte 的寄存器读取。
- 命令和响应严格按时钟字节对齐的协议。

三个 DMA API 的默认完成超时均为 50 ms。数据长度使用 `uint16_t`，与 STM32 HAL DMA 接口保持一致。

## 6. 基本使用示例

### 6.1 单段发送

```cpp
static spi_bus display_bus(0);

/**
 * @brief 使用 DMA 向显示设备发送一段数据
 *
 * @param data 发送缓冲区
 * @param size 数据长度
 *
 * @return SPI 总线结果
 */
static spi_result display_write(const uint8_t *data, uint16_t size)
{
    spi_result result = display_bus.cs_low(OLED_CS_GPIO_Port,
        OLED_CS_Pin);
    if(result != spi_result::OK)
    {
        return result;
    }

    result = display_bus.tx(data, size);

    spi_result cs_result = display_bus.cs_high(OLED_CS_GPIO_Port,
        OLED_CS_Pin);
    if(result == spi_result::OK)
    {
        result = cs_result;
    }

    return result;
}
```

关键点是无论 `tx()` 成功还是失败，都执行 `cs_high()`。

### 6.2 同一 CS 下的多段寄存器读取

下面只演示事务结构，命令位定义需要根据实际器件修改：

```cpp
static spi_bus sensor_bus(0);

/**
 * @brief 读取 SPI 设备寄存器
 *
 * @param register_address 寄存器地址
 * @param data 接收缓冲区
 * @param size 接收长度
 *
 * @return SPI 总线结果
 */
static spi_result sensor_read_register(uint8_t register_address,
    uint8_t *data,
    uint16_t size)
{
    static constexpr uint8_t READ_FLAG = 0x80;
    uint8_t command = register_address | READ_FLAG;

    spi_result result = sensor_bus.cs_low(SENSOR_CS_GPIO_Port,
        SENSOR_CS_Pin);
    if(result != spi_result::OK)
    {
        return result;
    }

    result = sensor_bus.tx(&command, 1U);
    if(result == spi_result::OK)
    {
        result = sensor_bus.rx(data, size);
    }

    spi_result cs_result = sensor_bus.cs_high(SENSOR_CS_GPIO_Port,
        SENSOR_CS_Pin);
    if(result == spi_result::OK)
    {
        result = cs_result;
    }

    return result;
}
```

从 `cs_low()` 到 `cs_high()` 期间，其他任务无法使用 SPI1，因此命令和数据阶段不会被打断。

### 6.3 全双工收发

```cpp
uint8_t tx_data[2] = {0x80, 0xFF};
uint8_t rx_data[2]{};

spi_result result = sensor_bus.cs_low(SENSOR_CS_GPIO_Port,
    SENSOR_CS_Pin);
if(result == spi_result::OK)
{
    result = sensor_bus.rx_tx(tx_data, rx_data, sizeof(tx_data));
    spi_result cs_result = sensor_bus.cs_high(SENSOR_CS_GPIO_Port,
        SENSOR_CS_Pin);

    if(result == spi_result::OK)
    {
        result = cs_result;
    }
}
```

某些设备会在发送命令时返回无效字节，真正数据可能位于 `rx_data[1]`。具体对齐方式必须以器件数据手册为准。

## 7. 返回值

| 返回值 | 含义 | 常见原因 |
|---|---|---|
| `OK` | 操作成功 | DMA 完成或事务状态正常 |
| `INVALID_BUS` | 总线编号无效 | `bus_id` 不是 0 |
| `INVALID_ARGUMENT` | 参数无效 | 空指针、长度为 0、CS 不匹配等 |
| `NOT_INITIALIZED` | 驱动尚未初始化 | 未调用 `init()` 或恢复失败 |
| `INVALID_CONTEXT` | 调用上下文无效 | ISR 调用或调度器尚未运行 |
| `INIT_FAILED` | 同步资源或 DMA 配置无效 | RX/TX DMA 未链接等 |
| `LOCK_TIMEOUT` | 获取事务 Mutex 超时 | 其他任务长时间持有 CS 事务 |
| `TRANSACTION_NOT_ACTIVE` | 没有活动事务 | 未先调用 `cs_low()` |
| `TRANSACTION_OWNER_MISMATCH` | 当前任务不是事务所有者 | 其他任务直接调用传输或 `cs_high()` |
| `TRANSFER_TIMEOUT` | DMA 完成等待超时 | DMA IRQ 未到达、SPI 状态异常等 |
| `BUSY` | HAL SPI 或当前任务事务状态忙 | 重复 `cs_low()`、HAL 尚未 Ready |
| `DMA_ERROR` | DMA 错误 | DMA 启动或运行异常 |
| `OVERRUN_ERROR` | 接收溢出 | RX 数据未及时处理等 |
| `MODE_FAULT` | SPI 模式错误 | Master Mode Fault |
| `FRAME_ERROR` | 帧格式错误 | HAL 检测到 FRE |
| `CRC_ERROR` | CRC 错误 | 启用 CRC 时校验失败 |
| `BUS_ERROR` | 其他 SPI 错误 | 未归类的 HAL SPI 错误 |
| `RECOVERY_FAILED` | SPI 重新初始化失败 | HAL DeInit/Init 未成功 |

## 8. 错误和超时恢复

HAL DMA 启动失败或等待完成信号量超时时，驱动会：

1. 在任务临界区中清除活动传输标志。
2. 调用 `HAL_SPI_Abort()` 终止可能仍在运行的 DMA。
3. 调用 `HAL_SPI_DeInit()`。
4. 调用 `HAL_SPI_Init()`，恢复 SPI、DMA 链接和 SPI1 IRQ。
5. 清除可能迟到的完成信号量。

恢复过程不会自动拉高 CS，也不会释放事务 Mutex。调用任务仍然拥有事务，必须执行 `cs_high()` 完成设备级协议收尾并释放总线。

当前驱动不会自动重试原始传输。是否重试应由设备驱动根据协议状态决定，重试前通常需要结束当前 CS 事务并重新开始。

## 9. 是否支持中断中调用

### 9.1 API 上下文矩阵

| 接口或路径 | 任务上下文 | ISR 上下文 | 说明 |
|---|---:|---:|---|
| `spi_bus::init()` | 支持，建议启动阶段调用 | 不支持 | 创建 FreeRTOS 同步资源 |
| `spi_bus::cs_low()` | 支持 | 不支持 | 会等待 Mutex |
| `spi_bus::cs_high()` | 支持 | 不支持 | 校验任务所有者并释放 Mutex |
| `spi_bus::rx()` | 支持 | 不支持 | 会等待 DMA 完成信号量 |
| `spi_bus::tx()` | 支持 | 不支持 | 会等待 DMA 完成信号量 |
| `spi_bus::rx_tx()` | 支持 | 不支持 | 会等待 DMA 完成信号量 |
| SPI DMA 完成回调 | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部完成路径 |
| `HAL_SPI_ErrorCallback()` | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部错误路径 |

公共入口会检查：

```cpp
if(__get_IPSR() != 0U ||
    xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
{
    return spi_result::INVALID_CONTEXT;
}
```

所以从 FOC 或其他 ISR 直接调用 SPI 公共 API 会立即返回 `INVALID_CONTEXT`，不会拉低 CS，也不会启动 DMA。

### 9.2 为什么不能从 FOC ISR 直接调用

即使底层使用 DMA，当前接口仍然包含任务专用操作：

- `cs_low()` 需要等待 Mutex。
- DMA API 需要等待二值信号量。
- 超时路径会调用阻塞式 `HAL_SPI_Abort()`。
- 错误恢复会重新初始化 SPI 和 DMA。
- 多段 SPI 协议要求 CS 在整个任务事务期间保持有效。

SPI1 当前约为 5.25 Mbit/s，比 I2C 快很多，但传输耗时仍取决于字节数、从设备协议和等待状态。把它放入 FOC ISR 会让控制周期执行时间依赖外设事务长度和异常状态。

因此 FOC ISR 不应直接执行 SPI 传输。

### 9.3 当前中断优先级约束

当前 SPI1 RX DMA、TX DMA 和 SPI1 全局中断数字优先级都是 5，符合当前 FreeRTOS `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY == 5` 的要求，所以内部回调可以调用 `xSemaphoreGiveFromISR()`。

当前 TIM1 Update、TIM8 Update 以及已启用的 DMA 中断数字优先级也主要为 5。如果以后 FOC 位于优先级 5 的 ISR，可以调用符合规则的 FreeRTOS `FromISR` 通知接口，但仍不能调用 `spi_bus` 的任务 API。

如果 FOC 被调整为数字优先级 0～4，则不能调用任何 FreeRTOS `FromISR` API。此时应让 SPI 任务自主周期采样，FOC ISR 只读取快照。

## 10. FOC 使用 SPI 数据的推荐方式

### 10.1 推荐架构

```text
SPI 采样任务
    │
    ├── cs_low()
    ├── DMA rx_tx() 或 tx() + rx()
    ├── cs_high()
    ├── 校验协议和设备状态
    └── 发布完整传感器快照
                    │
                    ▼
             最新数据双缓冲
                    │
                    ▼
FOC ADC/PWM ISR ── 只读取最近一次完整快照
```

这种结构可以保证：

- SPI 超时和错误恢复不会延长 FOC ISR。
- FOC 执行时间不依赖 SPI 传输长度。
- SPI 采样频率可以独立配置。
- FOC 可以根据快照时间戳判断数据是否过期。

发布多字段数据时不要只依靠一个 `volatile bool`。推荐使用双缓冲，在任务完整写入备用缓冲后一次性切换活动索引，避免 FOC 读取到一半旧、一半新的数据。

### 10.2 由 ISR 按需通知采样任务

如果必须由某个控制周期触发采样，ISR 只发送任务通知：

```cpp
static TaskHandle_t spi_sample_task_handle = nullptr;

/**
 * @brief 从允许调用 FreeRTOS API 的 ISR 请求一次 SPI 采样
 */
void request_spi_sample_from_isr()
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    vTaskNotifyGiveFromISR(spi_sample_task_handle,
        &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 按需执行 SPI 采样的任务入口
 *
 * @param argument FreeRTOS 任务参数
 */
static void spi_sample_task_entry(void *argument)
{
    while(true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 在任务上下文完成 CS 和 SPI DMA 事务，再发布快照。
    }
}
```

必须确认发出通知的中断数字优先级为 5～15。SPI 传输缓冲区应由采样任务或设备对象长期持有，不能引用 ISR 栈上的临时数据。

## 11. 当前限制和扩展点

### 11.1 当前限制

- 只支持 SPI1。
- 只支持低电平有效的软件 GPIO CS。
- 公开 API 是任务同步接口，没有异步请求队列。
- 不支持从 ISR 发起 SPI 事务。
- 调用方必须保证所有成功的 `cs_low()` 最终对应一次 `cs_high()`。
- 未提供 C++ RAII 事务守卫，提前返回时需要人工清理 CS。
- 没有自动重试策略。
- 当前 SPI 模式、速率和位序由 CubeMX 固定配置，不支持每个设备动态切换。
- `rx()` 固定发送 `0xFF` Dummy Byte；其他 Dummy Byte 应使用 `rx_tx()`。

### 11.2 后续可选优化

如果设备数量和协议复杂度增加，可以考虑：

- 增加 RAII `spi_transaction`，析构时自动拉高 CS 并释放 Mutex。
- 为不同设备增加 SPI Mode、分频和位序配置，并只在持有事务 Mutex 时切换。
- 增加独立 SPI Worker Task 和固定大小请求队列。
- 增加设备级超时、重试次数和故障统计。
- 对大块显示数据增加分块 DMA 和刷新帧率控制。

不建议为异步需求直接开放 ISR 版 `spi_bus`。更安全的方式是让 ISR 发送任务通知或固定大小请求，由 SPI Worker 在任务上下文复用当前事务和恢复逻辑。

## 12. 使用检查清单

接入新 SPI 设备时建议确认：

- 已调用 `spi_bus::init()`。
- 所有传输都位于 `cs_low()` 和 `cs_high()` 之间。
- 无论 DMA 成功还是失败，最终都会执行 `cs_high()`。
- `cs_high()` 与 `cs_low()` 使用相同 GPIO 端口和引脚。
- 同一事务的所有操作由同一个任务完成。
- 检查每一个 `spi_result`。
- 数据长度不超过 `uint16_t` 范围。
- 缓冲区在同步 API 返回前保持有效。
- SPI Mode、速率、位序和 Dummy Byte 符合器件手册。
- 不在 FOC、ADC、PWM 或其他 ISR 中直接调用公共 API。
- ISR 使用任务通知时，其中断数字优先级不小于 5。
- 使用逻辑分析仪确认 CS、SCK、MOSI、MISO 的实际时序。

## 13. 总体评价

当前实现适合多个 FreeRTOS 设备任务共享 SPI1 的场景。事务 Mutex 保证从 CS 拉低到拉高期间的协议原子性，DMA 降低数据搬运开销，完成信号量把 HAL 回调转换为任务同步结果，超时路径则集中执行 DMA 终止和 SPI 重新初始化。

对于 FOC 等硬实时路径，SPI 应保持在任务域运行。FOC ISR 只消费已经发布的传感器快照，或者在优先级允许时发送一个轻量 `FromISR` 通知，而不直接执行 SPI DMA 事务。
