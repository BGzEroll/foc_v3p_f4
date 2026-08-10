# 原生 FreeRTOS 常用 API 使用指南

## 1. 适用范围

本文针对当前工程实际包含的 **FreeRTOS V10.3.1**，依据以下本地源码和配置整理：

- `Middlewares/Third_Party/FreeRTOS/Source/include/task.h`
- `Middlewares/Third_Party/FreeRTOS/Source/include/queue.h`
- `Middlewares/Third_Party/FreeRTOS/Source/include/semphr.h`
- `Middlewares/Third_Party/FreeRTOS/Source/include/event_groups.h`
- `Middlewares/Third_Party/FreeRTOS/Source/include/timers.h`
- `Middlewares/Third_Party/FreeRTOS/Source/include/stream_buffer.h`
- `Middlewares/Third_Party/FreeRTOS/Source/include/message_buffer.h`
- `Core/Inc/FreeRTOSConfig.h`

本文所说的“原生 API”指直接调用 `xTaskCreate()`、`xQueueSend()`、`xSemaphoreTake()` 等 FreeRTOS 接口，不经过 CMSIS-RTOS v2 的 `osThreadNew()`、`osMessageQueuePut()` 等封装。

> 当前版本是 V10.3.1，只有每任务一个 32 位 Task Notification。较新版本的 `xTaskNotifyIndexed()`、`ulTaskNotifyTakeIndexed()` 等接口不适用于本工程。

## 2. 本工程 FreeRTOS 配置摘要

| 配置项 | 当前值 | 影响 |
|---|---:|---|
| `configTICK_RATE_HZ` | 1000 | 一个 tick 为 1 ms |
| `configUSE_PREEMPTION` | 1 | 抢占式调度 |
| `configMAX_PRIORITIES` | 56 | 有效优先级为 0～55，数值越大优先级越高 |
| `configMINIMAL_STACK_SIZE` | 128 | Idle 任务栈深度为 128 个 `StackType_t` |
| `configTOTAL_HEAP_SIZE` | 15360 | `heap_4` 总堆空间为 15 KiB |
| `configSUPPORT_DYNAMIC_ALLOCATION` | 1 | 可使用动态创建 API |
| `configSUPPORT_STATIC_ALLOCATION` | 1 | 可使用静态创建 API |
| `configUSE_MUTEXES` | 1 | 支持互斥锁 |
| `configUSE_RECURSIVE_MUTEXES` | 1 | 支持递归互斥锁 |
| `configUSE_COUNTING_SEMAPHORES` | 1 | 支持计数信号量 |
| `configUSE_TIMERS` | 1 | 支持软件定时器 |
| `configTIMER_TASK_PRIORITY` | 2 | Timer Service 任务优先级为 2 |
| `configTIMER_QUEUE_LENGTH` | 10 | 定时器命令队列长度为 10 |
| `configTIMER_TASK_STACK_DEPTH` | 256 | Timer Service 栈深度为 256 words |
| `configUSE_TRACE_FACILITY` | 1 | 支持部分任务状态查询 |
| `configUSE_TASK_NOTIFICATIONS` | 默认 1 | 支持任务通知 |
| `configUSE_QUEUE_SETS` | 默认 0 | 当前不支持 Queue Set |
| `configCHECK_FOR_STACK_OVERFLOW` | 默认 0 | 当前未启用栈溢出检查 |
| `configUSE_MALLOC_FAILED_HOOK` | 默认 0 | 当前未启用内存分配失败回调 |

STM32F407 的 `StackType_t` 为 32 位，因此原生任务 API 中的栈深度单位是 **4 字节 word**，不是字节。例如栈深度 128 等于 512 字节。

## 3. 命名和返回值习惯

FreeRTOS API 前缀一般表示返回类型：

| 前缀 | 常见含义 |
|---|---|
| `v` | 返回 `void` |
| `x` | 返回句柄、tick 或 `BaseType_t` |
| `ux` | 返回无符号整数 `UBaseType_t` |
| `ul` | 返回 `uint32_t` |
| `pv` | 返回 `void *` |

常用状态值：

```c
pdPASS
pdFAIL
pdTRUE
pdFALSE
errQUEUE_FULL
errQUEUE_EMPTY
```

所有毫秒值建议使用 `pdMS_TO_TICKS()` 转换：

```c
TickType_t timeout = pdMS_TO_TICKS(100);
```

无限等待使用 `portMAX_DELAY`。本工程 `INCLUDE_vTaskSuspend == 1`，所以阻塞 API 可以将其作为无限等待时间。

## 4. 任务管理 API

需要包含：

```c
#include "FreeRTOS.h"
#include "task.h"
```

### 4.1 动态创建任务

```c
TaskHandle_t motorTaskHandle = NULL;

static void MotorTask(void *argument)
{
    for (;;)
    {
        /* task body */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

BaseType_t result = xTaskCreate(
    MotorTask,                 /* 任务入口 */
    "motor",                  /* 调试名称，最多 15 个可见字符 */
    256,                       /* 栈深度：256 words = 1024 bytes */
    NULL,                      /* 传给任务的参数 */
    tskIDLE_PRIORITY + 4,      /* 优先级 */
    &motorTaskHandle           /* 返回任务句柄，可传 NULL */
);

configASSERT(result == pdPASS);
```

动态创建会从 `heap_4` 分配 TCB 和任务栈。任务自删后，内存由 Idle 任务回收，因此不能长期饿死 Idle 任务。

### 4.2 静态创建任务

```c
static StaticTask_t motorTaskTcb;
static StackType_t motorTaskStack[256];

TaskHandle_t motorTaskHandle = xTaskCreateStatic(
    MotorTask,
    "motor",
    256,
    NULL,
    tskIDLE_PRIORITY + 4,
    motorTaskStack,
    &motorTaskTcb
);

configASSERT(motorTaskHandle != NULL);
```

静态创建不会占用 FreeRTOS heap，适合数量固定、需要确定性内存布局的任务。

### 4.3 常用任务控制

| API | 用途 | 备注 |
|---|---|---|
| `vTaskStartScheduler()` | 启动调度器 | 成功后通常不返回 |
| `vTaskDelete(handle)` | 删除任务 | `NULL` 表示删除当前任务 |
| `vTaskSuspend(handle)` | 挂起任务 | `NULL` 表示挂起当前任务 |
| `vTaskResume(handle)` | 恢复任务 | 仅用于任务上下文 |
| `xTaskResumeFromISR(handle)` | ISR 中恢复任务 | 通常任务通知更合适 |
| `vTaskPrioritySet(handle, priority)` | 修改优先级 | `NULL` 表示当前任务 |
| `uxTaskPriorityGet(handle)` | 查询优先级 | 本工程已启用 |
| `xTaskGetCurrentTaskHandle()` | 获取当前任务句柄 | 本工程已启用 |
| `eTaskGetState(handle)` | 查询任务状态 | 主要用于诊断 |
| `taskYIELD()` | 主动让出 CPU | 不等于延时或阻塞 |

不建议把 `vTaskSuspend()/vTaskResume()` 当作常规事件同步工具，因为事件可能在任务真正挂起前发生，形成竞态。任务通知、信号量或队列更安全。

## 5. 延时和周期调度

### 5.1 相对延时

```c
vTaskDelay(pdMS_TO_TICKS(10));
```

它保证任务至少阻塞指定 tick，但任务真正再次运行还取决于其他高优先级任务。循环执行时间会累积到周期里，不适合严格周期任务。

### 5.2 固定周期运行

```c
static void ControlTask(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1);

    for (;;)
    {
        RunControlLoop();
        vTaskDelayUntil(&lastWakeTime, period);
    }
}
```

`vTaskDelayUntil()` 以绝对节拍推进，能避免任务执行时间导致的累计漂移，更适合固定周期的低频控制任务。

FOC 的 PWM/ADC 高频闭环通常应由硬件定时器和中断驱动，不建议依赖 1 kHz RTOS tick 实现高频精确定时。

其他时间 API：

```c
TickType_t now = xTaskGetTickCount();
TickType_t nowFromIsr = xTaskGetTickCountFromISR();
```

## 6. 任务通知

任务通知是每个任务自带的一个 32 位值和一个通知状态，比单独创建二值信号量或长度为 1 的队列更轻量。适合“一对一”事件、计数或位标志。

### 6.1 当作二值/计数信号量

任务中等待：

```c
uint32_t count = ulTaskNotifyTake(
    pdTRUE,          /* 返回前将计数清零；pdFALSE 表示只减 1 */
    portMAX_DELAY
);
```

另一个任务发送：

```c
xTaskNotifyGive(motorTaskHandle);
```

ISR 发送：

```c
void DMA_IRQHandler(void)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    /* clear hardware interrupt flags first */
    vTaskNotifyGiveFromISR(motorTaskHandle, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
```

### 6.2 当作事件位或数值邮箱

```c
/* 设置事件位 */
xTaskNotify(taskHandle, EVENT_ADC_READY, eSetBits);

/* 数值加一 */
xTaskNotify(taskHandle, 0, eIncrement);

/* 无条件覆盖值 */
xTaskNotify(taskHandle, value, eSetValueWithOverwrite);

/* 只有尚无待处理通知时才写值 */
BaseType_t ok = xTaskNotify(taskHandle, value, eSetValueWithoutOverwrite);
```

等待事件位：

```c
uint32_t notifiedValue;

if (xTaskNotifyWait(
        0,                  /* 进入等待前清除的位 */
        UINT32_MAX,         /* 返回前清除的位 */
        &notifiedValue,
        pdMS_TO_TICKS(20)) == pdTRUE)
{
    if ((notifiedValue & EVENT_ADC_READY) != 0U)
    {
        /* handle event */
    }
}
```

限制：一个任务只有一个 notification slot；多个无关模块共用时容易互相覆盖。多生产者或需要缓存多个数据项时应使用队列。

## 7. 队列

需要包含：

```c
#include "queue.h"
```

队列按值复制固定大小的数据项，适合任务之间或 ISR 到任务的数据传递。

```c
typedef struct
{
    uint16_t adcA;
    uint16_t adcB;
    TickType_t timestamp;
} Sample_t;

QueueHandle_t sampleQueue = xQueueCreate(8, sizeof(Sample_t));
configASSERT(sampleQueue != NULL);
```

发送和接收：

```c
Sample_t tx = {0};
xQueueSend(sampleQueue, &tx, pdMS_TO_TICKS(5));

Sample_t rx;
if (xQueueReceive(sampleQueue, &rx, portMAX_DELAY) == pdPASS)
{
    /* use rx */
}
```

常用 API：

| API | 用途 |
|---|---|
| `xQueueCreate(length, itemSize)` | 动态创建队列 |
| `xQueueCreateStatic(...)` | 静态创建队列 |
| `xQueueSend()` / `xQueueSendToBack()` | 发送到队尾 |
| `xQueueSendToFront()` | 发送到队头 |
| `xQueueReceive()` | 取出一个元素 |
| `xQueuePeek()` | 查看但不移除元素 |
| `xQueueOverwrite()` | 覆盖长度为 1 的队列 |
| `uxQueueMessagesWaiting()` | 查询已有元素数 |
| `uxQueueSpacesAvailable()` | 查询剩余空间 |
| `xQueueReset()` | 清空队列，仅任务上下文使用 |
| `vQueueDelete()` | 删除动态队列 |

ISR 模板：

```c
BaseType_t higherPriorityTaskWoken = pdFALSE;
Sample_t sample = ReadAdcSample();

xQueueSendFromISR(sampleQueue, &sample, &higherPriorityTaskWoken);
portYIELD_FROM_ISR(higherPriorityTaskWoken);
```

注意：

- 队列复制的是数据内容，不是自动复制指针指向的对象。
- 如果传指针，必须确保所指对象在消费者使用完成前一直有效。
- `xQueueOverwrite()` 只应对长度为 1 的队列使用。
- 本工程 `configUSE_QUEUE_SETS == 0`，不能使用 Queue Set，除非修改配置并重新构建。

## 8. 信号量和互斥锁

需要包含：

```c
#include "semphr.h"
```

### 8.1 二值信号量

```c
SemaphoreHandle_t adcDone = xSemaphoreCreateBinary();
configASSERT(adcDone != NULL);

/* 等待事件 */
xSemaphoreTake(adcDone, portMAX_DELAY);

/* 发送事件 */
xSemaphoreGive(adcDone);
```

`xSemaphoreCreateBinary()` 创建后初始为空，第一次 `Take` 前需要某处 `Give`。

ISR 中：

```c
BaseType_t higherPriorityTaskWoken = pdFALSE;
xSemaphoreGiveFromISR(adcDone, &higherPriorityTaskWoken);
portYIELD_FROM_ISR(higherPriorityTaskWoken);
```

对于单一 ISR 唤醒单一任务，优先考虑任务通知，它更轻量。

### 8.2 计数信号量

```c
SemaphoreHandle_t resources = xSemaphoreCreateCounting(4, 4);

xSemaphoreTake(resources, portMAX_DELAY);
/* use one resource */
xSemaphoreGive(resources);
```

适合资源计数或事件累计。它不传递具体数据；需要传数据时使用队列。

### 8.3 互斥锁

```c
SemaphoreHandle_t spiMutex = xSemaphoreCreateMutex();

if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(20)) == pdTRUE)
{
    AccessSharedSpi();
    xSemaphoreGive(spiMutex);
}
```

互斥锁支持优先级继承，用来保护任务间共享资源。必须由获得锁的任务释放，不能在 ISR 中使用。

递归互斥锁必须成对调用专用 API：

```c
xSemaphoreTakeRecursive(recursiveMutex, portMAX_DELAY);
xSemaphoreGiveRecursive(recursiveMutex);
```

不要混用普通 `Take/Give` 和递归版本。

## 9. 事件组

需要包含：

```c
#include "event_groups.h"
```

事件组适合一个任务等待多个布尔状态：

```c
#define EVENT_ADC_READY   (1UL << 0)
#define EVENT_PWM_READY   (1UL << 1)
#define EVENT_FAULT       (1UL << 2)

EventGroupHandle_t systemEvents = xEventGroupCreate();
configASSERT(systemEvents != NULL);

EventBits_t bits = xEventGroupWaitBits(
    systemEvents,
    EVENT_ADC_READY | EVENT_PWM_READY,
    pdTRUE,             /* 返回时清除等待位 */
    pdTRUE,             /* 等待全部位；pdFALSE 表示任一位 */
    portMAX_DELAY
);
```

常用 API：

```c
xEventGroupSetBits(group, bits);
xEventGroupClearBits(group, bits);
xEventGroupGetBits(group);
xEventGroupWaitBits(group, bits, clearOnExit, waitAll, timeout);
xEventGroupSync(group, bitsToSet, bitsToWaitFor, timeout);
vEventGroupDelete(group);
```

本工程使用 32 位 tick，Event Group 顶部 8 位由内核保留，应用可安全使用低 24 位。

`xEventGroupSetBitsFromISR()` 和 `xEventGroupClearBitsFromISR()` 会把操作投递给 Timer Service 任务，不是在 ISR 内立即修改；定时器命令队列已满时可能失败，必须检查返回值。

## 10. 软件定时器

需要包含：

```c
#include "timers.h"
```

```c
static void StatusTimerCallback(TimerHandle_t timer)
{
    void *context = pvTimerGetTimerID(timer);
    (void)context;

    /* callback must not block */
}

TimerHandle_t statusTimer = xTimerCreate(
    "status",
    pdMS_TO_TICKS(100),
    pdTRUE,                     /* 自动重装 */
    NULL,                       /* timer ID */
    StatusTimerCallback
);

configASSERT(statusTimer != NULL);
xTimerStart(statusTimer, 0);
```

常用控制 API：

```c
xTimerStart(timer, timeout);
xTimerStop(timer, timeout);
xTimerReset(timer, timeout);
xTimerChangePeriod(timer, newPeriod, timeout);
xTimerIsTimerActive(timer);
xTimerDelete(timer, timeout);
pvTimerGetTimerID(timer);
vTimerSetTimerID(timer, id);
```

ISR 中有对应的 `xTimerStartFromISR()`、`xTimerStopFromISR()`、`xTimerResetFromISR()` 和 `xTimerChangePeriodFromISR()`。

所有回调都在同一个 Timer Service 任务中串行执行。本工程该任务优先级为 2、命令队列长度为 10，因此回调中不得调用可能长时间阻塞的 API，也不应进行耗时控制运算。

软件定时器基于 RTOS tick，不适合代替 PWM、编码器采样或 ADC 触发等硬实时外设定时器。

## 11. Stream Buffer 和 Message Buffer

### 11.1 Stream Buffer

适合连续字节流，例如 UART DMA 数据：

```c
#include "stream_buffer.h"

StreamBufferHandle_t uartStream = xStreamBufferCreate(256, 1);

size_t sent = xStreamBufferSend(
    uartStream,
    data,
    length,
    pdMS_TO_TICKS(10)
);

size_t received = xStreamBufferReceive(
    uartStream,
    buffer,
    sizeof(buffer),
    portMAX_DELAY
);
```

### 11.2 Message Buffer

适合保留消息边界的变长消息：

```c
#include "message_buffer.h"

MessageBufferHandle_t messages = xMessageBufferCreate(256);

xMessageBufferSend(messages, &message, sizeof(message), portMAX_DELAY);
xMessageBufferReceive(messages, &message, sizeof(message), portMAX_DELAY);
```

两者都有 `SendFromISR` 和 `ReceiveFromISR` 版本。

V10.3.1 的 Stream/Message Buffer 按单写入者、单读取者设计。多个写入者或多个读取者共享时，调用方必须额外串行化访问，并将 block time 设为 0 后自行处理等待；更常见的做法是改用队列。

## 12. 内存管理

本工程使用 `heap_4.c`：支持动态分配、释放和相邻空闲块合并，但不是通用 libc heap。

```c
void *buffer = pvPortMalloc(128);
if (buffer != NULL)
{
    /* use buffer */
    vPortFree(buffer);
}
```

诊断 API：

```c
size_t freeNow = xPortGetFreeHeapSize();
size_t minimumEver = xPortGetMinimumEverFreeHeapSize();

HeapStats_t stats;
vPortGetHeapStats(&stats);
```

建议：

- 初始化阶段创建全部长期对象，运行阶段尽量避免频繁分配。
- 控制链路优先静态分配，避免内存耗尽和不可预测的失败路径。
- 当前 heap 只有 15 KiB，任务栈、队列存储和动态 RTOS 对象都会占用它。
- 当前未启用 `configUSE_MALLOC_FAILED_HOOK`，建议后续启用并实现 `vApplicationMallocFailedHook()`。

## 13. 临界区、调度锁和中断

### 13.1 任务临界区

```c
taskENTER_CRITICAL();
/* very short non-blocking critical section */
taskEXIT_CRITICAL();
```

临界区内绝不能调用会阻塞或切换任务的 API。临界区应尽可能短，避免增加中断延迟。

### 13.2 ISR 临界区

当前 Cortex-M4F port 提供：

```c
UBaseType_t savedMask = portSET_INTERRUPT_MASK_FROM_ISR();
/* short ISR critical section */
portCLEAR_INTERRUPT_MASK_FROM_ISR(savedMask);
```

### 13.3 暂停调度器

```c
vTaskSuspendAll();
/* other tasks cannot run, but interrupts remain enabled */
xTaskResumeAll();
```

暂停调度器不等于关闭中断，也不能在暂停期间调用阻塞 API。

### 13.4 ISR 调用规则

在 ISR 中只能调用名字带 `FromISR` 的 FreeRTOS API，并使用标准唤醒模板：

```c
BaseType_t higherPriorityTaskWoken = pdFALSE;

xQueueSendFromISR(queue, &item, &higherPriorityTaskWoken);

portYIELD_FROM_ISR(higherPriorityTaskWoken);
```

本工程：

```c
configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
```

STM32 数字越小，中断优先级越高：

- 数值优先级 0～4：不能调用任何 FreeRTOS API，包括 `FromISR` API。
- 数值优先级 5～15：可以调用 `FromISR` API。

当前 USART、ADC/DMA 和主要定时器中断大多配置为 5，允许调用 `FromISR` API。修改 CubeMX NVIC 优先级时必须保持这一约束。

## 14. 诊断 API

```c
UBaseType_t freeStackWords = uxTaskGetStackHighWaterMark(taskHandle);
UBaseType_t taskCount = uxTaskGetNumberOfTasks();
eTaskState state = eTaskGetState(taskHandle);
```

`uxTaskGetStackHighWaterMark()` 返回任务历史最小剩余栈空间，单位是 words，不是字节。STM32F407 上乘以 4 才是字节数。

本工程 `configUSE_TRACE_FACILITY == 1`，可使用 `uxTaskGetSystemState()`。但 `configUSE_STATS_FORMATTING_FUNCTIONS` 和 `configGENERATE_RUN_TIME_STATS` 当前未启用，因此不要直接依赖 `vTaskList()` 或 `vTaskGetRunTimeStats()`。

建议后续启用：

```c
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK   1
```

并实现：

```c
void vApplicationStackOverflowHook(TaskHandle_t task, char *name);
void vApplicationMallocFailedHook(void);
```

## 15. CMSIS-RTOS v2 与原生 FreeRTOS 对照

本节用于理解 CubeMX 已生成代码与原生 API 的关系，不要求替换现有 CMSIS 启动框架。

| CMSIS-RTOS v2 | 原生 FreeRTOS | 备注 |
|---|---|---|
| `osKernelInitialize()` | 无直接对应 | 原生对象可在调度器启动前创建 |
| `osKernelStart()` | `vTaskStartScheduler()` | 启动调度器 |
| `osThreadNew()` | `xTaskCreate()` / `xTaskCreateStatic()` | 注意栈单位不同 |
| `osThreadTerminate()` | `vTaskDelete()` | `NULL` 表示当前任务 |
| `osThreadYield()` | `taskYIELD()` | 主动让出 CPU |
| `osThreadSuspend()` | `vTaskSuspend()` | 不建议作为常规同步手段 |
| `osThreadResume()` | `vTaskResume()` | ISR 使用专用接口 |
| `osDelay()` | `vTaskDelay()` | 使用 `pdMS_TO_TICKS()` |
| `osDelayUntil()` | `vTaskDelayUntil()` | 参数语义不同，不能机械替换 |
| `osKernelGetTickCount()` | `xTaskGetTickCount()` | 返回 tick |
| Thread Flags | Task Notification | V10.3.1 每任务只有一个通知槽 |
| `osMessageQueueNew()` | `xQueueCreate()` | 原生队列按值复制 |
| `osMessageQueuePut()` | `xQueueSend()` | ISR 使用 `xQueueSendFromISR()` |
| `osMessageQueueGet()` | `xQueueReceive()` | ISR 使用 `xQueueReceiveFromISR()` |
| `osSemaphoreNew()` | Binary/Counting Semaphore | 根据最大计数选择 |
| `osSemaphoreAcquire()` | `xSemaphoreTake()` | 超时单位为 tick |
| `osSemaphoreRelease()` | `xSemaphoreGive()` | ISR 使用 FromISR 版本 |
| `osMutexNew()` | `xSemaphoreCreateMutex()` | 支持优先级继承 |
| `osEventFlagsNew()` | `xEventGroupCreate()` | 应用可用低 24 位 |
| `osEventFlagsWait()` | `xEventGroupWaitBits()` | 明确等待任一/全部和是否清除 |
| `osTimerNew()` | `xTimerCreate()` | 回调运行于 Timer Service 任务 |
| `osTimerStart()` | `xTimerStart()` | 原生参数为相对周期和命令等待时间 |
| `osMemoryPool*()` | 无完全对应 | 可使用静态池、自建 pool 或 FreeRTOS heap |
| `osWaitForever` | `portMAX_DELAY` | 本工程可作为无限等待 |

当前 CMSIS wrapper 直接把 `osPriority_t` 数值作为 FreeRTOS 优先级。`osPriorityNormal` 的数值为 24，因此当前 defaultTask 实际优先级也是 24。使用原生 API 创建新任务时，必须结合这个既有优先级安排相对优先级，不能误以为 defaultTask 位于 `tskIDLE_PRIORITY + 1`。

当前 CMSIS 配置：

```c
.stack_size = 128 * 4,
.priority = osPriorityNormal,
```

等价的原生动态创建约为：

```c
TaskHandle_t defaultTaskHandle;

configASSERT(xTaskCreate(
    StartDefaultTask,
    "defaultTask",
    128,               /* 128 words = 512 bytes */
    NULL,
    24,                /* 保持现有 CMSIS Normal 优先级 */
    &defaultTaskHandle
) == pdPASS);
```

任务内：

```c
for (;;)
{
    vTaskDelay(pdMS_TO_TICKS(1));
}
```

## 16. 本工程推荐使用方式

本工程不需要替换 FreeRTOS，也不需要删除 CMSIS-RTOS v2 wrapper。推荐保持 CubeMX 生成的启动结构不变：

```c
osKernelInitialize();
MX_FREERTOS_Init();
osKernelStart();
```

继续保留以下文件和功能：

- `CMSIS_RTOS_V2/cmsis_os2.c`
- `cmsis_os.h` / `cmsis_os2.h`
- `osKernelInitialize()` / `osKernelStart()`
- CMSIS wrapper 提供的 `SysTick_Handler()`
- CMSIS wrapper 提供的静态 Idle/Timer 任务内存回调
- CubeMX 自动生成的 defaultTask

业务代码直接使用原生 API 即可。两套 API 最终运行在同一个 FreeRTOS V10.3.1 内核上，可以同时存在：

- CubeMX 生成的 defaultTask 继续由 `osThreadNew()` 创建。
- 用户任务可以用 `xTaskCreate()` / `xTaskCreateStatic()` 创建。
- 用户队列、信号量、通知、事件组和定时器均可使用原生 API。
- 原生创建的对象建议始终使用原生 API 操作。
- 不要在未核对 wrapper 实现时，把 CMSIS handle 随意传入原生 API。

业务文件直接包含所需的原生头文件：

建议业务文件直接包含需要的原生头文件：

```c
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"
```

例如，可以在 `MX_FREERTOS_Init()` 的 `USER CODE` 区域中创建原生对象：

```c
/* USER CODE BEGIN RTOS_QUEUES */
sampleQueue = xQueueCreate(8, sizeof(Sample_t));
configASSERT(sampleQueue != NULL);
/* USER CODE END RTOS_QUEUES */

/* USER CODE BEGIN RTOS_THREADS */
configASSERT(xTaskCreate(
    MotorTask,
    "motor",
    256,
    NULL,
    25,
    &motorTaskHandle
) == pdPASS);
/* USER CODE END RTOS_THREADS */
```

这里示例优先级 25 比当前 defaultTask 的 24 高一级。实际 FOC 工程应统一规划所有任务优先级，不要直接照搬示例值。

这种方式不需要修改 SysTick、调度器启动、CMake 中的 FreeRTOS 源文件，也不会妨碍以后使用 CubeMX 重新生成外设初始化代码。

## 17. 选型速查

| 需求 | 首选原生机制 |
|---|---|
| ISR 唤醒唯一任务 | Task Notification |
| 单任务到单任务的轻量事件/计数 | Task Notification |
| 传递固定大小结构体并缓存多项 | Queue |
| 保护 SPI/I2C/UART 等共享资源 | Mutex |
| ISR/任务发送“发生了一次”事件 | Binary Semaphore 或 Task Notification |
| 管理 N 个同类资源 | Counting Semaphore |
| 等待多个布尔条件 | Event Group |
| 低精度周期性后台动作 | Software Timer |
| UART 等连续字节流 | Stream Buffer |
| 带边界的变长消息 | Message Buffer |
| 高频 FOC/PWM/ADC 精确定时 | 硬件 Timer + DMA/ISR，不使用软件定时器 |

对于本项目，建议优先组合：硬件定时器触发 ADC/PWM，ISR 用 Task Notification 唤醒处理任务，任务之间的数据快照使用长度为 1 的 Queue 或双缓冲，共享外设使用 Mutex，故障状态使用 Event Group。
