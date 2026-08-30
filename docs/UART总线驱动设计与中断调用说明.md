# UART 总线驱动设计与中断调用说明

## 1. 结论

当前 `uart_bus` 已重构为面向 FreeRTOS 任务的 UART DMA 接口：

- TX 使用“Mutex + Normal DMA + 完成二值信号量”。
- RX 使用“Circular DMA + UART IDLE/HT/TC 事件 + FreeRTOS StreamBuffer”。
- `write_bytes()` 从调用任务看是同步接口，任务会阻塞等待 DMA 完成，但不会忙等待占用 CPU。
- `read_bytes()` 从软件接收缓存取数据，可以非阻塞读取，也可以等待指定时间。
- TX 和 RX 使用独立的同步资源，因此一个任务等待 RX 时，另一个任务仍可发送；UART 可以保持全双工工作。
- 公共读写 API 只能在 FreeRTOS 调度器已经运行的任务上下文调用，不能从 FOC、ADC、PWM 或其他 ISR 直接调用。
- DMA、UART IDLE 和 UART 错误回调属于内部 ISR 路径，通过 FreeRTOS `FromISR` API 把接收数据或完成事件交给任务。
- 多个 `uart_bus(0)` 对象会共享同一个 USART1 底层对象，不会绕开 TX/RX Mutex。
- RX 是字节流，不保留“发送帧”的边界；协议分帧、长度、转义和 CRC 应由上层协议模块负责。

相关实现：

- [`uart_bus.h`](../user_lib/drivers/bus/uart_bus.h)
- [`uart_bus.cpp`](../user_lib/drivers/bus/uart_bus.cpp)
- [`usart.c`](../Core/Src/usart.c)
- [`dma.c`](../Core/Src/dma.c)
- [`stm32f4xx_it.c`](../Core/Src/stm32f4xx_it.c)
- [`FreeRTOSConfig.h`](../Core/Inc/FreeRTOSConfig.h)

## 2. 当前硬件配置

当前工程只配置了一条 UART 总线：

| 项目 | 当前配置 |
|---|---|
| 总线编号 | `0` |
| HAL 外设 | USART1 |
| 工作模式 | Asynchronous、TX/RX |
| 波特率 | 460800 |
| 数据格式 | 8 data bits、1 stop bit、no parity |
| 硬件流控 | None |
| 过采样 | 16 |
| TX | PA9 |
| RX | PA10 |
| RX DMA | DMA2 Stream2 Channel4 |
| RX DMA 模式 | Circular |
| TX DMA | DMA2 Stream7 Channel4 |
| TX DMA 模式 | Normal |
| DMA 数据宽度 | Peripheral Byte、Memory Byte |
| DMA 内存地址递增 | Enable |
| DMA 优先级 | Low |
| DMA2 Stream2 IRQ 数字优先级 | 5 |
| DMA2 Stream7 IRQ 数字优先级 | 5 |
| USART1 IRQ 数字优先级 | 5 |

USART1 全局中断不能关闭，因为 RX 的 IDLE 事件、UART 线路错误以及 TX 最后一个停止位发送完成事件都需要 USART1 IRQ。

### 2.1 波特率配置状态

截至本报告编写时：

- 实际参与 CMake 编译的 `Core/Src/usart.c` 配置为 `460800`。
- CubeMX 工程文件 `project.ioc` 保存的 USART1 波特率也是 `460800`。

两处配置目前一致，后续通过 CubeMX 重新生成代码不会仅因波特率设置而产生意外变化。根据第 12 节的实测结果，460800 也更适合当前 DAPLink CDC + USBIP 通信路径。

### 2.2 FreeRTOS 相关配置

| 配置 | 当前值 | 对驱动的影响 |
|---|---:|---|
| `configTICK_RATE_HZ` | 1000 | 一个 tick 为 1 ms |
| `configSUPPORT_STATIC_ALLOCATION` | 1 | Mutex、信号量和 StreamBuffer 控制块使用静态内存 |
| `configUSE_MUTEXES` | 1 | 支持 TX/RX 任务互斥 |
| `INCLUDE_xTaskGetSchedulerState` | 1 | 公共 API 可以检查调度器状态 |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | 5 | 数字优先级 5～15 的 ISR 才能调用 FreeRTOS `FromISR` API |

USART1、RX DMA 和 TX DMA 当前均为数字优先级 5，符合内部调用 `xStreamBufferSendFromISR()` 和 `xSemaphoreGiveFromISR()` 的要求。

## 3. 软件框架

### 3.1 整体分层

```text
协议模块、设备驱动或业务任务
        │
        │ init() / read_bytes() / write_bytes()
        ▼
公开 uart_bus 对象
        │
        │ bus_id：当前仅支持 0
        ▼
物理 UART 对象 uart_dev
        ├── HAL UART 句柄
        ├── TX Mutex
        ├── RX Mutex
        ├── TX 完成二值信号量
        ├── 256 字节 RX DMA 循环缓冲区
        ├── 1024 字节可用 RX StreamBuffer
        ├── TX/RX 活动状态
        └── RX 错误和丢弃统计
        │
        ▼
STM32 HAL + USART1 + RX/TX DMA
```

`uart_bus` 是公开的轻量访问对象，只保存 `bus_id`。真正拥有 HAL 句柄、DMA 缓冲区和 RTOS 同步对象的是 `uart_bus.cpp` 内部的 `uart_dev`。

当前物理资源池为：

```cpp
static uart_dev uart_devs[] =
{
    uart_dev(&huart1)
};
```

因此下面两个公开对象都会访问同一个 USART1：

```cpp
static uart_bus command_uart(0);
static uart_bus telemetry_uart(0);
```

它们的 TX 会被同一把 TX Mutex 串行化，RX 读取也会被同一把 RX Mutex 串行化。

需要特别注意：RX StreamBuffer 是消费型字节流。多个读取任务不会各自收到一份副本，而是谁先读取，谁就消费相应数据。如果多个模块都关心同一串口数据，应设置一个唯一的协议解析任务，再由解析任务把不同消息分发到 Queue、Task Notification 或状态快照。

## 4. 初始化流程

`uart_bus::init()` 会完成：

1. 检查 `bus_id` 是否有效。
2. 检查 HAL UART 实例以及 RX、TX DMA 句柄是否存在。
3. 检查 RX DMA 是否为 Circular 模式。
4. 使用静态内存创建 TX Mutex。
5. 使用静态内存创建 RX Mutex。
6. 使用静态内存创建 TX 完成二值信号量。
7. 创建 RX StreamBuffer。
8. 清零 RX 位置、错误和丢弃统计。
9. 调用 `HAL_UARTEx_ReceiveToIdle_DMA()` 启动持续接收。

初始化不从 FreeRTOS heap 动态分配这些同步对象。对同一条 UART 重复调用 `init()` 会直接返回 `OK`。

可以在 `start_init_all()` 间接调用的设备初始化函数中执行 `uart_bus::init()`。当前工程中该阶段位于 `osKernelInitialize()` 之后、`osKernelStart()` 之前，允许创建静态 FreeRTOS 对象并启动 UART RX DMA。

实际的 `read_bytes()` 和 `write_bytes()` 必须等调度器运行后再由任务调用。

## 5. RX：循环 DMA + IDLE/HT/TC + StreamBuffer

### 5.1 数据路径

```text
USART1 RX 引脚
    │
    ▼
DMA2 Stream2 Circular
    │
    │ 写入 256 字节 rx_dma_buffer
    ▼
HT / TC / UART IDLE 中断
    │
    ▼
HAL_UARTEx_RxEventCallback()
    │
    ├── 根据新旧 DMA 位置计算新增数据区间
    ├── 处理循环缓冲区回卷
    ├── 去除 TC 后重复出现的整缓冲区 IDLE 事件
    └── xStreamBufferSendFromISR()
                    │
                    ▼
            1024 字节软件接收流
                    │
                    ▼
          任务调用 read_bytes()
```

### 5.2 为什么同时使用 HT、TC 和 IDLE

- HT：DMA 写到 256 字节缓冲区的一半时产生事件，可把前 128 字节及时送入 StreamBuffer。
- TC：DMA 写满缓冲区时产生事件，可把后 128 字节送入 StreamBuffer，并继续从缓冲区开头接收。
- IDLE：串口线路空闲一个字符时间后产生事件，可及时提交不足 128 字节的短数据段。

只依靠 TC 会让短帧一直停留在 DMA 缓冲区中，直到累计满 256 字节；加入 IDLE 后，短命令和不定长数据可以及时交给任务。

只依靠 IDLE 则无法很好地处理没有空闲间隔的连续数据流；HT 和 TC 保证连续流仍会周期性进入 StreamBuffer。

### 5.3 DMA 位置与事件去重

HAL 回调提供的 `size` 实际表示 DMA 缓冲区中的当前写入位置，而不是“本次新增字节数”。驱动保存上一次位置，通过两者差值计算新增数据。

当 DMA 从缓冲区尾部回到开头时，驱动会拆成两段写入 StreamBuffer：

```text
旧位置 ──────────> DMA 缓冲区末尾
DMA 缓冲区开头 ──> 新位置
```

STM32 HAL 在 Circular DMA 整缓冲区完成后，还可能紧接着报告一次 `position == buffer_size` 的 IDLE 事件。驱动对该情况做了去重，避免把同一整段数据重复送入 StreamBuffer。

### 5.4 StreamBuffer 容量

驱动定义：

```cpp
static constexpr uint16_t UART_RX_DMA_BUFFER_SIZE = 256;
static constexpr uint16_t UART_RX_STREAM_STORAGE_SIZE = 1025;
```

FreeRTOS 静态 StreamBuffer 的实际可用容量比存储数组少 1 字节，因此当前软件 RX 缓冲可用容量为 1024 字节。

如果任务处理速度长期低于串口接收速度，StreamBuffer 最终会写满。`xStreamBufferSendFromISR()` 未能写入的字节会被丢弃，并累计到 `rx_dropped_bytes()`。

该计数只表示“DMA 数据送入软件 StreamBuffer 时发生的丢弃”。UART 线路错误或 DMA 错误导致的硬件级数据损失无法精确折算成字节数，应结合 `rx_error_count()` 和 `last_rx_error()` 判断。

### 5.5 UART 是字节流，不保留帧边界

一次主机 `write()` 不保证对应一次 MCU `read_bytes()`。数据可能被拆成多段，也可能多次发送被合并在一次读取中。

因此上层协议不能依赖以下假设：

```text
主机发送 32 字节 → MCU 一次 read_bytes() 必然返回 32 字节
```

可靠协议通常需要至少一种分帧方式：

- 固定长度帧。
- 帧头 + 长度字段。
- 帧尾分隔符，并对正文进行转义。
- COBS、SLIP 等编码。

对控制命令和遥测数据还建议加入序号及 CRC，以便检测丢帧、乱序和内容损坏。

## 6. TX：Mutex + DMA + 完成信号量

### 6.1 一次发送的时序

任务调用 `write_bytes()` 后：

1. 检查驱动是否初始化。
2. 检查发送指针和长度。
3. 检查当前不是 ISR，并确认 FreeRTOS 调度器正在运行。
4. 等待获取 TX Mutex。
5. 清除完成信号量中可能残留的旧事件。
6. 设置 `tx_active` 和初始发送状态。
7. 调用 `HAL_UART_Transmit_DMA()`。
8. 当前任务阻塞等待 TX 完成信号量。
9. DMA2 Stream7 完成内存数据搬运。
10. HAL 等待 USART1 真正发完最后一个停止位。
11. `HAL_UART_TxCpltCallback()` 记录结果并通过 `xSemaphoreGiveFromISR()` 唤醒任务。
12. 任务读取发送结果并释放 TX Mutex。

从 API 调用者看，函数会等待本次发送成功、错误或超时后才返回；从 CPU 看，等待期间当前任务处于阻塞态，其他任务可以运行。

### 6.2 为什么 TX DMA 完成仍需要 USART1 IRQ

DMA 完成只表示最后一个字节已经被写入 USART 数据寄存器，不代表最后一个停止位已经出现在 TX 引脚上。

STM32 HAL 的 Normal DMA TX 路径会在 DMA 完成后开启 UART Transmission Complete 中断，最后由 USART1 IRQ 确认真正发送结束并调用 `HAL_UART_TxCpltCallback()`。

所以 TX 正常完成同时依赖：

- DMA2 Stream7 IRQ。
- USART1 全局 IRQ。

缺少其中任意一个，都可能导致 `write_bytes()` 等不到完成信号量并最终返回 `TRANSFER_TIMEOUT`。

### 6.3 发送缓冲区生命周期

驱动没有额外复制 TX 数据，而是把调用方缓冲区直接交给 DMA：

```cpp
uint8_t message[] = {0xAA, 0x55, 0x01};
uart_result result = uart.write_bytes(message, sizeof(message));
```

由于当前 API 会一直等到 DMA 完成才返回，因此任务栈上的局部数组可以安全使用。函数返回前不能修改或释放该缓冲区。

### 6.4 TX 与 RX 并行

TX 使用独立 Mutex 和完成信号量，RX 使用独立 Mutex 和 StreamBuffer。USART HAL 也分别维护 `gState` 和 `RxState`，因此 RX DMA 持续运行时可以发起 TX DMA。

同一时刻只允许一笔 TX，但 RX 不会因为某个任务等待 TX Mutex 而停止。该结构适合全双工串口协议。

## 7. 公共 API

### 7.1 构造函数

```cpp
explicit uart_bus(uint8_t bus_id = 0);
```

当前只有 `bus_id == 0`，对应 USART1。其他编号会返回 `INVALID_BUS`，不会静默回退到 USART1。

### 7.2 初始化

```cpp
uart_result init();
```

初始化同步对象并立即启动 Circular RX DMA。

### 7.3 读取

```cpp
uart_result read_bytes(uint8_t *data,
    uint16_t max_size,
    uint16_t &received_size,
    uint32_t read_timeout_ms = DEFAULT_READ_TIMEOUT_MS,
    uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS);
```

参数含义：

- `data`：调用方接收缓冲区。
- `max_size`：本次最多读取的字节数。
- `received_size`：实际读取的字节数；进入函数后会先被清零。
- `read_timeout_ms`：等待接收数据的时间。
- `lock_timeout_ms`：等待 RX Mutex 的时间。

默认 `read_timeout_ms == 0`，表示非阻塞读取：

- 有数据时返回 `OK`，并填写 `received_size`。
- 没有数据时也返回 `OK`，但 `received_size == 0`。

当 `read_timeout_ms > 0` 且超时前没有任何数据时，返回 `READ_TIMEOUT`。

StreamBuffer 的触发级别为 1，因此阻塞读取会在至少一个字节可用时解除阻塞，不保证自动等待到 `max_size` 个字节。需要完整协议帧时，上层应维护解析缓存并持续读取，直到满足长度或帧尾条件。

### 7.4 写入

```cpp
uart_result write_bytes(const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS,
    uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
```

默认 TX Mutex 等待时间为 10 ms，默认发送完成等待时间为 50 ms。

### 7.5 诊断接口

```cpp
uint32_t rx_dropped_bytes() const;
uint32_t rx_error_count() const;
uart_result last_rx_error() const;
```

- `rx_dropped_bytes()`：软件 StreamBuffer 空间不足时累计丢弃的字节数。
- `rx_error_count()`：UART/DMA 错误回调累计次数。
- `last_rx_error()`：最近一次错误类型；成功恢复后不会自动清零，因此它是粘滞诊断状态。

读取这些 32 位统计值在当前 Cortex-M4 上是单次对齐访问，但它们是运行时诊断值，不应把多个 getter 的结果当作严格原子的联合快照。

## 8. 返回值

| 返回值 | 含义 | 常见原因 |
|---|---|---|
| `OK` | 操作成功 | 初始化完成、读到数据、非阻塞读取暂时无数据或 TX 完成 |
| `INVALID_BUS` | 总线编号无效 | `bus_id` 不是 0 |
| `INVALID_ARGUMENT` | 参数无效 | 空指针或长度为 0 |
| `NOT_INITIALIZED` | 驱动尚未初始化 | 未调用 `init()` |
| `INVALID_CONTEXT` | 调用上下文无效 | ISR 调用，或读写时调度器尚未运行 |
| `INIT_FAILED` | 初始化失败 | DMA 句柄缺失、RX 非 Circular 或 RTOS 对象创建失败 |
| `LOCK_TIMEOUT` | 获取 TX/RX Mutex 超时 | 其他任务正在操作同一方向 |
| `READ_TIMEOUT` | 等待接收数据超时 | 指定时间内没有新数据 |
| `TRANSFER_TIMEOUT` | 等待 TX 完成超时 | DMA/UART 完成 IRQ 未到达或链路状态异常 |
| `BUSY` | HAL UART 忙 | 前一笔发送未正确结束等 |
| `DMA_ERROR` | DMA 错误 | DMA 启动或运行异常 |
| `OVERRUN_ERROR` | UART 接收溢出 | 新字节到达时旧数据未被硬件及时读取 |
| `NOISE_ERROR` | 噪声错误 | 电气干扰或采样不稳定 |
| `FRAME_ERROR` | 帧错误 | 波特率、停止位不一致或线路异常 |
| `PARITY_ERROR` | 奇偶校验错误 | 双方校验设置不一致或数据受干扰 |
| `BUS_ERROR` | 其他 UART 错误 | 未归类的 HAL 错误 |
| `RECOVERY_FAILED` | 恢复失败 | TX Abort 或 RX DMA 重启失败 |

## 9. 错误和超时恢复

### 9.1 TX 恢复

以下情况会进入 TX 恢复：

- `HAL_UART_Transmit_DMA()` 启动失败。
- 等待 TX 完成信号量超时。

恢复流程为：

1. 在任务临界区内清除 `tx_active`。
2. 调用 `HAL_UART_AbortTransmit()` 停止可能仍在运行的 TX DMA。
3. 清除可能迟到的完成信号量。
4. 释放 TX Mutex。

当前驱动不会自动重发原始数据。串口协议如果需要可靠交付，应由上层加入帧序号、ACK、超时和有限次数重试。

### 9.2 RX 恢复

UART 或 DMA 错误进入 `HAL_UART_ErrorCallback()` 后，驱动会：

1. 保存最近错误类型。
2. 增加错误计数。
3. 如果 TX 正在等待且该错误影响当前 UART，则用错误结果唤醒 TX 任务。
4. 当 HAL 已将 `RxState` 恢复为 Ready 时，立即重新调用 `HAL_UARTEx_ReceiveToIdle_DMA()`。

如果中断内自动重启失败，`rx_active` 会保持为 false。下一次任务调用 `read_bytes()` 时，驱动会通过 `HAL_UART_AbortReceive()` 清理状态，并再次启动 Circular RX DMA。

错误发生到 DMA 重启之间到达的数据可能丢失，驱动不能恢复这些已经丢失的物理字节。上层协议应依靠 CRC、序号和超时识别不完整帧。

## 10. 并发行为

| 场景 | 当前行为 |
|---|---|
| 两个任务同时 TX | 通过 TX Mutex 串行化 |
| 两个任务同时 RX | 通过 RX Mutex 串行读取同一个消费型字节流 |
| 一个任务 TX、另一个任务 RX | 可以并行 |
| 多个 `uart_bus(0)` 对象 | 共享同一个 `uart_dev` 和全部底层资源 |
| ISR 与任务同时访问公开 API | ISR 被 `INVALID_CONTEXT` 拒绝 |
| RX StreamBuffer 写满 | 保留已写入内容，无法写入的新字节被计数并丢弃 |

RX Mutex 只能避免两个任务同时进入 `xStreamBufferReceive()`，不能把同一份串口数据复制给多个任务。如果系统同时存在命令解析、调试控制和遥测响应，推荐由一个 UART 协议任务独占 RX，再向其他模块发布结构化消息。

## 11. 是否支持在中断中调用

### 11.1 API 上下文矩阵

| 接口或路径 | 任务上下文 | ISR 上下文 | 说明 |
|---|---:|---:|---|
| `uart_bus::init()` | 支持，建议仅初始化阶段调用 | 不支持 | 创建 RTOS 对象并启动 RX DMA |
| `uart_bus::read_bytes()` | 支持 | 不支持 | 会获取 RX Mutex，并可能阻塞等待数据 |
| `uart_bus::write_bytes()` | 支持 | 不支持 | 会获取 TX Mutex 并阻塞等待完成信号量 |
| 诊断 getter | 支持 | 不建议作为 ISR 业务接口 | 只读取统计值，但未定义 ISR 级一致性契约 |
| `HAL_UARTEx_RxEventCallback()` | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部 RX DMA/IDLE 路径 |
| `HAL_UART_TxCpltCallback()` | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部 TX 完成路径 |
| `HAL_UART_ErrorCallback()` | 不由任务直接调用 | 支持且应由 HAL 调用 | 内部错误和恢复路径 |

读写入口会检查：

```cpp
if(__get_IPSR() != 0U ||
    xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
{
    return uart_result::INVALID_CONTEXT;
}
```

因此从任何 ISR 直接调用 `read_bytes()` 或 `write_bytes()`，都会立即返回 `INVALID_CONTEXT`。

### 11.2 为什么使用 DMA 仍不能从 ISR 调用

“底层使用 DMA”和“公共 API 可以从 ISR 调用”是两件不同的事情。当前接口包含：

- Mutex 获取。
- 带超时的信号量等待。
- StreamBuffer 任务读取。
- TX 超时后的阻塞式 Abort。
- RX 异常后的任务级恢复。

ISR 不能等待 Mutex 或阻塞等 DMA 完成，也不应该把自身执行时间绑定到串口传输长度和异常恢复时间。

### 11.3 FOC 中断中的推荐发送方式

FOC ISR 不应格式化字符串，也不应直接调用 `uart_bus::write_bytes()`。推荐路径为：

```text
FOC ADC/PWM ISR
    │
    ├── 采集少量定宽遥测字段
    └── FromISR 写入固定长度 Queue/StreamBuffer 或发布双缓冲快照
                        │
                        ▼
                   UART 遥测任务
                        │
                        ├── 组帧、CRC、限频
                        └── uart_bus::write_bytes()
```

如果 FOC ISR 的数字优先级为 5～15，可以使用符合规则的 FreeRTOS `FromISR` API：

```cpp
BaseType_t higher_priority_task_woken = pdFALSE;

xQueueSendFromISR(telemetry_queue,
    &sample,
    &higher_priority_task_woken);
portYIELD_FROM_ISR(higher_priority_task_woken);
```

如果以后把 FOC ISR 调整到数字优先级 0～4，它将高于 FreeRTOS 系统调用边界，不能调用任何 FreeRTOS `FromISR` API。此时应使用预分配的无锁双缓冲或只写少量原子状态，让低优先级任务稍后读取。

### 11.4 中断中的接收需求

驱动已经在 RX DMA/IDLE ISR 内把字节搬入 StreamBuffer，上层不需要也不应该从 FOC ISR 再读取 UART。

如果 UART 命令要改变控制目标，推荐由 UART 协议任务解析并校验完整命令，再发布一个定宽、带时间戳的控制快照。FOC ISR 只读取最近一次已完整发布的快照，避免在硬实时路径中处理可变长度协议。

## 12. 实际闭环测试

### 12.1 测试环境

测试日期：2026-08-11。

| 项目 | 配置 |
|---|---|
| 开发环境 | Debian 13 虚拟机 |
| USB 路径 | Windows `usbipd` 转发到 Debian |
| 调试器 | DAPLink/CMSIS-DAP，VID:PID `0416:5051` |
| 虚拟串口 | `/dev/ttyACM0` |
| 稳定设备路径 | `/dev/serial/by-id/usb-CMSIS-DAP_STM32_CMSIS-DAP_CMSIS-DAP-if01` |
| 主机程序 | Python 标准库 `termios + select` 二进制测试脚本 |
| 固件测试逻辑 | FreeRTOS 任务通过 `read_bytes()` 接收，再通过 `write_bytes()` 原样回显 |
| 测试数据 | 固定长度边界帧 + 固定随机种子的随机二进制帧 |

临时测试任务和累计计数器在测试结束后均已撤销，没有保留在正式业务代码中。当前工程正式配置已经统一为 460800。

### 12.2 460800 闭环结果

| 指标 | 结果 |
|---|---:|
| 测试帧数 | 1000 |
| 主机发送字节 | 252316 |
| MCU `read_bytes()` 累计接收 | 252316 |
| MCU `write_bytes()` 成功累计发送 | 252316 |
| 主机逐字节回显校验 | 全部一致 |
| TX API 错误 | 0 |
| UART/DMA 错误 | 0 |
| StreamBuffer 丢弃字节 | 0 |

该结果验证了以下完整路径：

```text
DAPLink CDC → USART1 RX → Circular DMA → IDLE/HT/TC
→ StreamBuffer → read_bytes() → write_bytes()
→ TX DMA → USART1 TX → DAPLink CDC → Python 校验
```

### 12.3 921600 测试现象

921600 下曾出现主机回显少字节，但板端诊断为：

| 指标 | 数值 |
|---|---:|
| 主机累计发送 | 23094 |
| MCU 累计接收 | 23094 |
| MCU 成功提交 TX DMA | 23094 |
| 主机实际收到 | 23084 |
| 板端 TX 错误 | 0 |
| 板端 UART/DMA 错误 | 0 |
| 板端 StreamBuffer 丢弃 | 0 |

这说明该轮丢失发生在 MCU 已经完成发送之后，更可能位于 DAPLink CDC、USBIP 或主机虚拟串口路径，而不是当前 `uart_bus` 的 RX DMA、StreamBuffer 或 TX DMA 同步逻辑。

这个结论不表示 STM32 USART1 本身不能稳定运行在 921600。要单独验证 MCU UART 的物理上限，应使用可靠 USB-UART、逻辑分析仪或另一块 MCU，并确认线长、电平、时钟误差和接地条件。

对当前实际开发链路，建议优先使用 460800。

## 13. 使用示例

### 13.1 初始化

```cpp
#include "drivers/bus/uart_bus.h"

static uart_bus host_uart(0);

/**
 * @brief 初始化上位机 UART 通信
 *
 * @return 初始化成功时返回 true
 */
static bool init_host_uart()
{
    return host_uart.init() == uart_result::OK;
}
```

初始化可以在系统启动阶段完成，但不要在调度器启动前调用读写 API。

### 13.2 非阻塞读取

```cpp
uint8_t rx_data[64]{};
uint16_t received_size = 0U;

uart_result result = host_uart.read_bytes(rx_data,
    sizeof(rx_data),
    received_size);

if(result == uart_result::OK && received_size > 0U)
{
    // 把新字节交给上层协议解析器。
}
```

### 13.3 带超时读取

```cpp
uint8_t rx_data[64]{};
uint16_t received_size = 0U;

uart_result result = host_uart.read_bytes(rx_data,
    sizeof(rx_data),
    received_size,
    20U);

if(result == uart_result::READ_TIMEOUT)
{
    // 20 ms 内没有收到任何新字节。
}
```

### 13.4 DMA 发送

```cpp
static const uint8_t MESSAGE[] = {0xAA, 0x55, 0x01, 0x00};

uart_result result = host_uart.write_bytes(MESSAGE,
    sizeof(MESSAGE));

if(result != uart_result::OK)
{
    // 记录错误；是否重试由协议层决定。
}
```

### 13.5 回显任务结构

下面示例只展示 API 组合方式，不建议把无协议回显作为正式通信协议：

```cpp
/**
 * @brief 接收并回显 UART 字节流
 *
 * @param argument RTOS 任务参数
 */
static void uart_echo_task_entry(void *argument)
{
    uint8_t buffer[128]{};

    while(true)
    {
        uint16_t received_size = 0U;
        uart_result result = host_uart.read_bytes(buffer,
            sizeof(buffer),
            received_size,
            1000U);

        if(result == uart_result::OK && received_size > 0U)
        {
            host_uart.write_bytes(buffer,
                received_size,
                uart_bus::DEFAULT_LOCK_TIMEOUT_MS,
                100U);
        }
    }
}
```

正式代码应检查 `write_bytes()` 返回值，并加入协议解析、限流及错误统计。

## 14. 当前限制和扩展点

### 14.1 当前限制

- 只支持 USART1。
- 只提供任务同步接口，没有异步 TX 请求队列。
- 不支持从 ISR 调用公共读写 API。
- RX StreamBuffer 可用容量固定为 1024 字节。
- 多个 RX 任务消费同一字节流，不支持广播。
- UART 只负责字节传输，不提供帧头、长度、CRC、ACK 或重试。
- 没有硬件流控，高持续流量下只能依靠上层限速和足够快的读取任务。
- TX 数据不复制，调用方缓冲区必须保持有效直到函数返回。
- RX 错误恢复窗口中到达的物理数据可能丢失。
- 诊断计数没有提供公开清零接口。
- 当前波特率由 CubeMX/HAL 固定，驱动不支持运行时切换。

STM32F407 没有 Cortex-M7 那类数据 Cache，因此当前 DMA 缓冲区不需要额外 Cache clean/invalidate。以后如果把该驱动迁移到带 D-Cache 的 MCU，必须补充 DMA 缓冲区对齐和 Cache 一致性处理。

### 14.2 后续可选优化

如果协议复杂度或吞吐量继续增加，可以考虑：

- 增加唯一 UART Worker Task，集中处理 RX 解析和 TX 队列。
- 增加固定大小的 TX 请求池，提供真正异步的任务级发送。
- 增加帧序号、CRC、ACK 和有限次数重试。
- 按业务优先级区分控制消息与低优先级日志，防止日志占满发送通道。
- 为 RX StreamBuffer 增加高水位统计。
- 增加统计清零和完整状态快照接口。
- 如果持续流量较大，根据任务最坏调度延迟调整 DMA 和 StreamBuffer 容量。

不建议为满足 FOC 路径而直接增加 `write_bytes_from_isr()`。更可靠的方案是 ISR 只发布定宽数据，UART Worker Task 负责组帧和 DMA 发送。

## 15. 使用检查清单

接入 UART 协议或设备时建议确认：

- `project.ioc` 与 `usart.c` 的波特率一致。
- 对端和 USART1 的波特率、数据位、停止位、校验位一致。
- RX DMA 是 Circular，TX DMA 是 Normal。
- USART1、DMA2 Stream2 和 DMA2 Stream7 IRQ 均已启用。
- 上述 IRQ 的数字优先级不小于 5。
- 已调用 `uart_bus::init()` 并检查返回值。
- 只在调度器运行后的任务中调用 `read_bytes()` 和 `write_bytes()`。
- 不假设一次读取等于一帧，使用可靠的上层分帧协议。
- 检查 `received_size`，尤其注意非阻塞读取可能返回 0。
- 检查每一次 `uart_result`。
- TX 缓冲区在函数返回前保持有效。
- 系统只有一个 RX 协议解析者，或者明确接受多任务竞争消费。
- 定期观察 `rx_dropped_bytes()`、`rx_error_count()` 和 `last_rx_error()`。
- 不在 FOC、ADC、PWM 或其他 ISR 中调用公共读写 API。
- ISR 需要触发发送时，只发布任务通知、Queue 项或双缓冲快照。
- 使用逻辑分析仪或可靠 USB-UART 验证最终硬件速率和波形。

## 16. 总体评价

当前 `uart_bus` 已经从裸机式“单字节接收中断 + 手工环形队列 + 后台 TX DMA 链”转为更适合 FreeRTOS 的结构。

Circular RX DMA 持续接收数据，IDLE/HT/TC 事件解决短数据和连续数据流的提交时机，StreamBuffer 把 ISR 数据安全交给任务；TX Mutex 和完成信号量则把 DMA 完成事件转换为清晰的任务同步结果。错误和超时路径也集中在总线层处理，上层协议无需直接操作 HAL 状态机。

该实现适合命令通信、遥测、设备协议和调试串口等任务域用途。对于 FOC 等硬实时 ISR，应继续采用“ISR 发布定宽数据或快照，UART 任务负责协议与 DMA”的分层方式。
