#include "spi_bus.h"

#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

enum class spi_transfer_direction : uint8_t
{
    RECEIVE = 0,
    TRANSMIT,
    TRANSMIT_RECEIVE
};

// 管理一条物理 SPI 总线的事务互斥、DMA 状态和完成同步。
class spi_dev
{
    public:
        explicit spi_dev(SPI_HandleTypeDef *handle);

    public:
        spi_result init();

    public:
        spi_result cs_low(GPIO_TypeDef *port,
            uint16_t pin,
            uint32_t lock_timeout_ms);
        spi_result cs_high(GPIO_TypeDef *port, uint16_t pin);
        spi_result rx(uint8_t *data,
            uint16_t size,
            uint32_t transfer_timeout_ms);
        spi_result tx(const uint8_t *data,
            uint16_t size,
            uint32_t transfer_timeout_ms);
        spi_result rx_tx(const uint8_t *tx_data,
            uint8_t *rx_data,
            uint16_t size,
            uint32_t transfer_timeout_ms);
        bool matches_handle(SPI_HandleTypeDef *target_handle) const;
        void complete_from_isr(spi_result result);

    private:
        spi_result transfer_bytes(spi_transfer_direction direction,
            const uint8_t *tx_data,
            uint8_t *rx_data,
            uint16_t size,
            uint32_t transfer_timeout_ms);
        spi_result validate_task_context() const;
        spi_result validate_transaction_owner() const;
        bool recover_bus();
        void cancel_active_transfer();

    private:
        SPI_HandleTypeDef *handle;
        SemaphoreHandle_t mutex = nullptr;
        StaticSemaphore_t mutex_storage{};
        SemaphoreHandle_t completion_semaphore = nullptr;
        StaticSemaphore_t completion_semaphore_storage{};
        TaskHandle_t transaction_owner = nullptr;
        GPIO_TypeDef *active_cs_port = nullptr;
        uint16_t active_cs_pin = 0U;
        bool initialized = false;
        bool transaction_active = false;
        volatile bool transfer_active = false;
        volatile spi_result transfer_result = spi_result::NOT_INITIALIZED;
};

static spi_dev spi_devs[] =
{
    spi_dev(&hspi1)
};

static constexpr uint8_t SPI_DEV_COUNT =
    (uint8_t)(sizeof(spi_devs) / sizeof(spi_devs[0]));

/**
 * @brief 将毫秒超时转换为 FreeRTOS tick
 *
 * @param timeout_ms 超时时间，单位毫秒
 *
 * @return FreeRTOS tick 数
 */
static TickType_t milliseconds_to_ticks(uint32_t timeout_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    if(timeout_ms > 0U && ticks == 0U)
    {
        ticks = 1U;
    }

    return ticks;
}

/**
 * @brief 根据 HAL SPI 错误码生成总线结果
 *
 * @param handle HAL SPI 句柄
 *
 * @return SPI 总线结果
 */
static spi_result map_hal_error(SPI_HandleTypeDef *handle)
{
    uint32_t error = HAL_SPI_GetError(handle);

    if((error & HAL_SPI_ERROR_DMA) != 0U)
    {
        return spi_result::DMA_ERROR;
    }

    if((error & HAL_SPI_ERROR_OVR) != 0U)
    {
        return spi_result::OVERRUN_ERROR;
    }

    if((error & HAL_SPI_ERROR_MODF) != 0U)
    {
        return spi_result::MODE_FAULT;
    }

    if((error & HAL_SPI_ERROR_FRE) != 0U)
    {
        return spi_result::FRAME_ERROR;
    }

    if((error & HAL_SPI_ERROR_CRC) != 0U)
    {
        return spi_result::CRC_ERROR;
    }

    return spi_result::BUS_ERROR;
}

/**
 * @brief 将 HAL 状态转换为 SPI 总线结果
 *
 * @param handle HAL SPI 句柄
 * @param status HAL 调用状态
 *
 * @return SPI 总线结果
 */
static spi_result map_hal_status(SPI_HandleTypeDef *handle,
    HAL_StatusTypeDef status)
{
    switch(status)
    {
        case HAL_OK:
            return spi_result::OK;

        case HAL_BUSY:
            return spi_result::BUSY;

        case HAL_TIMEOUT:
            return spi_result::TRANSFER_TIMEOUT;

        case HAL_ERROR:
        default:
            return map_hal_error(handle);
    }
}

/**
 * @brief 根据总线编号获取物理 SPI 设备
 *
 * @param bus_id SPI 总线编号
 *
 * @return 有效编号对应的设备指针，无效编号返回 nullptr
 */
static spi_dev *get_dev(uint8_t bus_id)
{
    if(bus_id >= SPI_DEV_COUNT)
    {
        return nullptr;
    }

    return &spi_devs[bus_id];
}

/**
 * @brief 根据 HAL 句柄获取物理 SPI 设备
 *
 * @param handle HAL SPI 句柄
 *
 * @return 匹配的设备指针，未匹配时返回 nullptr
 */
static spi_dev *get_dev(SPI_HandleTypeDef *handle)
{
    for(uint8_t index = 0; index < SPI_DEV_COUNT; index++)
    {
        if(spi_devs[index].matches_handle(handle))
        {
            return &spi_devs[index];
        }
    }

    return nullptr;
}

/**
 * @brief 创建物理 SPI 总线管理对象
 *
 * @param handle HAL SPI 句柄
 */
spi_dev::spi_dev(SPI_HandleTypeDef *handle)
    : handle(handle)
{
}

/**
 * @brief 初始化 SPI 总线的 FreeRTOS 同步对象
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::init()
{
    if(initialized)
    {
        return spi_result::OK;
    }

    if(!handle || !handle->Instance || !handle->hdmarx || !handle->hdmatx)
    {
        return spi_result::INIT_FAILED;
    }

    mutex = xSemaphoreCreateMutexStatic(&mutex_storage);
    completion_semaphore =
        xSemaphoreCreateBinaryStatic(&completion_semaphore_storage);

    if(!mutex || !completion_semaphore)
    {
        mutex = nullptr;
        completion_semaphore = nullptr;
        return spi_result::INIT_FAILED;
    }

    transfer_result = spi_result::OK;
    initialized = true;
    return spi_result::OK;
}

/**
 * @brief 拉低片选并锁定 SPI 事务
 *
 * @param port 片选 GPIO 端口
 * @param pin 片选 GPIO 引脚
 * @param lock_timeout_ms 等待总线 Mutex 的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::cs_low(GPIO_TypeDef *port,
    uint16_t pin,
    uint32_t lock_timeout_ms)
{
    if(!initialized)
    {
        return spi_result::NOT_INITIALIZED;
    }

    if(!port || pin == 0U)
    {
        return spi_result::INVALID_ARGUMENT;
    }

    spi_result context_result = validate_task_context();
    if(context_result != spi_result::OK)
    {
        return context_result;
    }

    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
    if(transaction_active && transaction_owner == current_task)
    {
        return spi_result::BUSY;
    }

    TickType_t lock_timeout = milliseconds_to_ticks(lock_timeout_ms);
    if(xSemaphoreTake(mutex, lock_timeout) != pdTRUE)
    {
        return spi_result::LOCK_TIMEOUT;
    }

    transaction_owner = current_task;
    active_cs_port = port;
    active_cs_pin = pin;
    transaction_active = true;
    HAL_GPIO_WritePin(active_cs_port, active_cs_pin, GPIO_PIN_RESET);
    return spi_result::OK;
}

/**
 * @brief 拉高片选并释放 SPI 事务
 *
 * @param port 片选 GPIO 端口
 * @param pin 片选 GPIO 引脚
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::cs_high(GPIO_TypeDef *port, uint16_t pin)
{
    spi_result context_result = validate_task_context();
    if(context_result != spi_result::OK)
    {
        return context_result;
    }

    spi_result owner_result = validate_transaction_owner();
    if(owner_result != spi_result::OK)
    {
        return owner_result;
    }

    if(port != active_cs_port || pin != active_cs_pin)
    {
        return spi_result::INVALID_ARGUMENT;
    }

    HAL_GPIO_WritePin(active_cs_port, active_cs_pin, GPIO_PIN_SET);
    active_cs_port = nullptr;
    active_cs_pin = 0U;
    transaction_owner = nullptr;
    transaction_active = false;
    xSemaphoreGive(mutex);
    return spi_result::OK;
}

/**
 * @brief 使用 DMA 接收 SPI 数据
 *
 * @param data 接收缓冲区
 * @param size 接收长度
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::rx(uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    return transfer_bytes(spi_transfer_direction::RECEIVE,
        nullptr,
        data,
        size,
        transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 发送 SPI 数据
 *
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::tx(const uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    return transfer_bytes(spi_transfer_direction::TRANSMIT,
        data,
        nullptr,
        size,
        transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 同时发送和接收 SPI 数据
 *
 * @param tx_data 发送缓冲区
 * @param rx_data 接收缓冲区
 * @param size 收发长度
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::rx_tx(const uint8_t *tx_data,
    uint8_t *rx_data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    return transfer_bytes(spi_transfer_direction::TRANSMIT_RECEIVE,
        tx_data,
        rx_data,
        size,
        transfer_timeout_ms);
}

/**
 * @brief 判断 HAL SPI 句柄是否属于当前物理总线
 *
 * @param target_handle 待匹配的 HAL SPI 句柄
 *
 * @return 句柄匹配时返回 true
 */
bool spi_dev::matches_handle(SPI_HandleTypeDef *target_handle) const
{
    return handle == target_handle;
}

/**
 * @brief 执行一次由片选事务保护的 SPI DMA 传输
 *
 * @param direction DMA 传输方向
 * @param tx_data 发送缓冲区
 * @param rx_data 接收缓冲区
 * @param size 数据长度
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::transfer_bytes(spi_transfer_direction direction,
    const uint8_t *tx_data,
    uint8_t *rx_data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    if(!initialized)
    {
        return spi_result::NOT_INITIALIZED;
    }

    if(size == 0U ||
        (direction == spi_transfer_direction::RECEIVE && !rx_data) ||
        (direction == spi_transfer_direction::TRANSMIT && !tx_data) ||
        (direction == spi_transfer_direction::TRANSMIT_RECEIVE &&
            (!tx_data || !rx_data)))
    {
        return spi_result::INVALID_ARGUMENT;
    }

    spi_result context_result = validate_task_context();
    if(context_result != spi_result::OK)
    {
        return context_result;
    }

    spi_result owner_result = validate_transaction_owner();
    if(owner_result != spi_result::OK)
    {
        return owner_result;
    }

    while(xSemaphoreTake(completion_semaphore, 0U) == pdTRUE)
    {
    }

    transfer_result = spi_result::BUSY;
    transfer_active = true;
    HAL_StatusTypeDef hal_status;

    if(direction == spi_transfer_direction::RECEIVE)
    {
        memset(rx_data, 0xFF, size);
        hal_status = HAL_SPI_Receive_DMA(handle, rx_data, size);
    }
    else if(direction == spi_transfer_direction::TRANSMIT)
    {
        hal_status = HAL_SPI_Transmit_DMA(handle, tx_data, size);
    }
    else
    {
        hal_status = HAL_SPI_TransmitReceive_DMA(handle,
            tx_data,
            rx_data,
            size);
    }

    if(hal_status != HAL_OK)
    {
        spi_result result = map_hal_status(handle, hal_status);
        cancel_active_transfer();
        bool recovered = recover_bus();
        return recovered ? result : spi_result::RECOVERY_FAILED;
    }

    TickType_t transfer_timeout =
        milliseconds_to_ticks(transfer_timeout_ms);
    if(xSemaphoreTake(completion_semaphore, transfer_timeout) != pdTRUE)
    {
        cancel_active_transfer();
        bool recovered = recover_bus();
        return recovered ? spi_result::TRANSFER_TIMEOUT :
            spi_result::RECOVERY_FAILED;
    }

    return transfer_result;
}

/**
 * @brief 检查当前是否处于可阻塞的任务上下文
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::validate_task_context() const
{
    if(__get_IPSR() != 0U ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return spi_result::INVALID_CONTEXT;
    }

    return spi_result::OK;
}

/**
 * @brief 检查当前任务是否拥有 SPI 片选事务
 *
 * @return SPI 总线结果
 */
spi_result spi_dev::validate_transaction_owner() const
{
    if(!transaction_active)
    {
        return spi_result::TRANSACTION_NOT_ACTIVE;
    }

    if(transaction_owner != xTaskGetCurrentTaskHandle())
    {
        return spi_result::TRANSACTION_OWNER_MISMATCH;
    }

    return spi_result::OK;
}

/**
 * @brief 在异常传输后终止 DMA 并重新初始化 SPI 外设
 *
 * @return 恢复成功时返回 true
 */
bool spi_dev::recover_bus()
{
    HAL_SPI_Abort(handle);
    bool recovered = HAL_SPI_DeInit(handle) == HAL_OK &&
        HAL_SPI_Init(handle) == HAL_OK;

    while(xSemaphoreTake(completion_semaphore, 0U) == pdTRUE)
    {
    }

    if(!recovered)
    {
        initialized = false;
    }

    return recovered;
}

/**
 * @brief 在任务上下文中取消当前活动传输标记
 */
void spi_dev::cancel_active_transfer()
{
    taskENTER_CRITICAL();
    transfer_active = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief 在中断中完成当前 SPI DMA 传输
 *
 * @param result DMA 传输结果
 */
void spi_dev::complete_from_isr(spi_result result)
{
    if(!initialized || !transfer_active)
    {
        return;
    }

    transfer_result = result;
    transfer_active = false;

    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(completion_semaphore,
        &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * @brief 创建 SPI 总线访问对象
 *
 * @param bus_id SPI 总线编号
 */
spi_bus::spi_bus(uint8_t bus_id)
    : bus_id(bus_id)
{
}

/**
 * @brief 初始化 SPI 总线的 FreeRTOS 同步资源
 *
 * @return SPI 总线结果
 */
spi_result spi_bus::init()
{
    spi_dev *device = get_dev(bus_id);
    if(!device)
    {
        return spi_result::INVALID_BUS;
    }

    return device->init();
}

/**
 * @brief 拉低片选并锁定 SPI 事务
 *
 * @param port 片选 GPIO 端口
 * @param pin 片选 GPIO 引脚
 * @param lock_timeout_ms 等待总线 Mutex 的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_bus::cs_low(GPIO_TypeDef *port,
    uint16_t pin,
    uint32_t lock_timeout_ms)
{
    spi_dev *device = get_dev(bus_id);
    if(!device)
    {
        return spi_result::INVALID_BUS;
    }

    return device->cs_low(port, pin, lock_timeout_ms);
}

/**
 * @brief 拉高片选并释放 SPI 事务
 *
 * @param port 片选 GPIO 端口
 * @param pin 片选 GPIO 引脚
 *
 * @return SPI 总线结果
 */
spi_result spi_bus::cs_high(GPIO_TypeDef *port, uint16_t pin)
{
    spi_dev *device = get_dev(bus_id);
    if(!device)
    {
        return spi_result::INVALID_BUS;
    }

    return device->cs_high(port, pin);
}

/**
 * @brief 使用 DMA 接收 SPI 数据
 *
 * @param data 接收缓冲区
 * @param size 接收长度
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_bus::rx(uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    spi_dev *device = get_dev(bus_id);
    if(!device)
    {
        return spi_result::INVALID_BUS;
    }

    return device->rx(data, size, transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 发送 SPI 数据
 *
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_bus::tx(const uint8_t *data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    spi_dev *device = get_dev(bus_id);
    if(!device)
    {
        return spi_result::INVALID_BUS;
    }

    return device->tx(data, size, transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 同时发送和接收 SPI 数据
 *
 * @param tx_data 发送缓冲区
 * @param rx_data 接收缓冲区
 * @param size 收发长度
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return SPI 总线结果
 */
spi_result spi_bus::rx_tx(const uint8_t *tx_data,
    uint8_t *rx_data,
    uint16_t size,
    uint32_t transfer_timeout_ms)
{
    spi_dev *device = get_dev(bus_id);
    if(!device)
    {
        return spi_result::INVALID_BUS;
    }

    return device->rx_tx(tx_data,
        rx_data,
        size,
        transfer_timeout_ms);
}

/**
 * @brief 处理 SPI DMA 发送完成事件
 *
 * @param handle HAL SPI 句柄
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *handle)
{
    spi_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(spi_result::OK);
    }
}

/**
 * @brief 处理 SPI DMA 接收完成事件
 *
 * @param handle HAL SPI 句柄
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *handle)
{
    spi_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(spi_result::OK);
    }
}

/**
 * @brief 处理 SPI DMA 全双工收发完成事件
 *
 * @param handle HAL SPI 句柄
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *handle)
{
    spi_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(spi_result::OK);
    }
}

/**
 * @brief 处理 SPI 或 DMA 错误事件
 *
 * @param handle HAL SPI 句柄
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *handle)
{
    spi_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(map_hal_error(handle));
    }
}
