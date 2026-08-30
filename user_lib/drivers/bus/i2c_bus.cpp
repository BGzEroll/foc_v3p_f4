#include "i2c_bus.h"

#include "i2c.h"
#include "system/sys_time.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

enum class i2c_transfer_direction : uint8_t
{
    READ = 0,
    WRITE
};

static constexpr uint8_t I2C_TRANSFER_ATTEMPT_COUNT = 2;
static constexpr uint8_t I2C_RECOVERY_CLOCK_PULSES = 9;
static constexpr uint32_t I2C_RECOVERY_DELAY_US = 5;
static constexpr uint32_t I2C_RECOVERY_FORCE_DELAY_US = 20;

// 管理一条物理 I2C 总线的互斥访问、DMA 状态、完成同步和自动恢复。
class i2c_dev
{
    public:
        explicit i2c_dev(I2C_HandleTypeDef *handle);

    public:
        i2c_result init();
        i2c_result read_bytes(uint8_t device_address,
            uint8_t register_address,
            uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms,
            uint32_t transfer_timeout_ms);
        i2c_result write_bytes(uint8_t device_address,
            uint8_t register_address,
            const uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms,
            uint32_t transfer_timeout_ms);
        bool matches_handle(I2C_HandleTypeDef *target_handle) const;
        void complete_from_isr(i2c_result result);

    private:
        i2c_result transfer_bytes(i2c_transfer_direction direction,
            uint8_t device_address,
            uint8_t register_address,
            uint8_t *data,
            uint16_t size,
            uint32_t lock_timeout_ms,
            uint32_t transfer_timeout_ms);
        bool recover_bus();
        void cancel_active_transfer();

    private:
        I2C_HandleTypeDef *handle;
        SemaphoreHandle_t mutex = nullptr;
        StaticSemaphore_t mutex_storage{};
        SemaphoreHandle_t completion_semaphore = nullptr;
        StaticSemaphore_t completion_semaphore_storage{};
        bool initialized = false;
        volatile bool transfer_active = false;
        volatile i2c_result transfer_result = i2c_result::NOT_INITIALIZED;
};

static i2c_dev i2c_devs[] =
{
    i2c_dev(&hi2c1),
    i2c_dev(&hi2c2)
};

static constexpr uint8_t I2C_DEV_COUNT =
    (uint8_t)(sizeof(i2c_devs) / sizeof(i2c_devs[0]));

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

    if(timeout_ms > 0 && ticks == 0)
    {
        ticks = 1;
    }

    return ticks;
}

/**
 * @brief 根据 HAL I2C 错误码生成总线结果
 *
 * @param handle HAL I2C 句柄
 *
 * @return I2C 总线结果
 */
static i2c_result map_hal_error(I2C_HandleTypeDef *handle)
{
    uint32_t error = HAL_I2C_GetError(handle);

    if((error & HAL_I2C_ERROR_AF) != 0)
    {
        return i2c_result::NACK;
    }

    if((error & (HAL_I2C_ERROR_DMA | HAL_I2C_ERROR_DMA_PARAM)) != 0)
    {
        return i2c_result::DMA_ERROR;
    }

    if((error & HAL_I2C_ERROR_TIMEOUT) != 0)
    {
        return i2c_result::TRANSFER_TIMEOUT;
    }

    return i2c_result::BUS_ERROR;
}

/**
 * @brief 将 HAL 状态转换为 I2C 总线结果
 *
 * @param handle HAL I2C 句柄
 * @param status HAL 调用状态
 *
 * @return I2C 总线结果
 */
static i2c_result map_hal_status(I2C_HandleTypeDef *handle,
    HAL_StatusTypeDef status)
{
    switch(status)
    {
        case HAL_OK:
            return i2c_result::OK;

        case HAL_BUSY:
            return i2c_result::BUSY;

        case HAL_TIMEOUT:
            return i2c_result::TRANSFER_TIMEOUT;

        case HAL_ERROR:
        default:
            return map_hal_error(handle);
    }
}

/**
 * @brief 根据总线编号获取物理 I2C 设备
 *
 * @param bus_id I2C 总线编号
 *
 * @return 有效编号对应的设备指针，无效编号返回 nullptr
 */
static i2c_dev *get_dev(uint8_t bus_id)
{
    if(bus_id >= I2C_DEV_COUNT)
    {
        return nullptr;
    }

    return &i2c_devs[bus_id];
}

/**
 * @brief 根据 HAL 句柄获取物理 I2C 设备
 *
 * @param handle HAL I2C 句柄
 *
 * @return 匹配的设备指针，未匹配时返回 nullptr
 */
static i2c_dev *get_dev(I2C_HandleTypeDef *handle)
{
    for(uint8_t index = 0; index < I2C_DEV_COUNT; index++)
    {
        if(i2c_devs[index].matches_handle(handle))
        {
            return &i2c_devs[index];
        }
    }

    return nullptr;
}

/**
 * @brief 根据 I2C 外设获取总线恢复所需的 GPIO 引脚
 *
 * @param target_handle I2C 外设句柄
 * @param gpio_port 用于输出时钟和数据的 GPIO 端口
 * @param scl_pin SCL 引脚
 * @param sda_pin SDA 引脚
 *
 * @return 找到对应引脚配置时返回 true
 */
static bool get_i2c_gpio_config(I2C_HandleTypeDef *target_handle,
    GPIO_TypeDef *&gpio_port,
    uint16_t &scl_pin,
    uint16_t &sda_pin)
{
    if(!target_handle)
    {
        return false;
    }

    gpio_port = GPIOB;
    if(target_handle->Instance == I2C1)
    {
        scl_pin = GPIO_PIN_6;
        sda_pin = GPIO_PIN_7;
        return true;
    }

    if(target_handle->Instance == I2C2)
    {
        scl_pin = GPIO_PIN_10;
        sda_pin = GPIO_PIN_11;
        return true;
    }

    return false;
}

/**
 * @brief 通过 APB 外设复位清除 STM32 I2C 外设内部状态机
 *
 * @param target_handle 待复位的 I2C 外设句柄
 *
 * @return 找到对应外设并完成复位时返回 true
 */
static bool reset_i2c_peripheral(I2C_HandleTypeDef *target_handle)
{
    if(!target_handle || !target_handle->Instance)
    {
        return false;
    }

    if(target_handle->Instance == I2C1)
    {
        __HAL_RCC_I2C1_FORCE_RESET();
        __DSB();
        sys_time::delay_us(I2C_RECOVERY_DELAY_US);
        __HAL_RCC_I2C1_RELEASE_RESET();
        __DSB();
        return true;
    }

    if(target_handle->Instance == I2C2)
    {
        __HAL_RCC_I2C2_FORCE_RESET();
        __DSB();
        sys_time::delay_us(I2C_RECOVERY_DELAY_US);
        __HAL_RCC_I2C2_RELEASE_RESET();
        __DSB();
        return true;
    }

    return false;
}

/**
 * @brief 判断 I2C 时钟线和数据线是否都已释放
 *
 * @param target_handle I2C 外设句柄
 *
 * @return 两条线路均为高电平时返回 true
 */
static bool i2c_lines_released(I2C_HandleTypeDef *target_handle)
{
    GPIO_TypeDef *gpio_port = nullptr;
    uint16_t scl_pin = 0;
    uint16_t sda_pin = 0;

    if(!get_i2c_gpio_config(target_handle,
        gpio_port,
        scl_pin,
        sda_pin))
    {
        return false;
    }

    return HAL_GPIO_ReadPin(gpio_port, scl_pin) == GPIO_PIN_SET &&
           HAL_GPIO_ReadPin(gpio_port, sda_pin) == GPIO_PIN_SET;
}

/**
 * @brief 判断 I2C 外设是否仍处于忙状态
 *
 * @param target_handle I2C 外设句柄
 *
 * @return HAL 状态或硬件 BUSY 标志未释放时返回 true
 */
static bool i2c_peripheral_busy(I2C_HandleTypeDef *target_handle)
{
    if(!target_handle || !target_handle->Instance)
    {
        return true;
    }

    return HAL_I2C_GetState(target_handle) != HAL_I2C_STATE_READY ||
           (target_handle->Instance->SR2 & I2C_SR2_BUSY) != 0;
}

/**
 * @brief 创建物理 I2C 总线管理对象
 *
 * @param handle HAL I2C 句柄
 */
i2c_dev::i2c_dev(I2C_HandleTypeDef *handle)
    : handle(handle)
{
}

/**
 * @brief 初始化 I2C 总线的 FreeRTOS 同步对象
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::init()
{
    if(initialized)
    {
        return i2c_result::OK;
    }

    if(!handle || !handle->Instance || !handle->hdmarx || !handle->hdmatx)
    {
        return i2c_result::INIT_FAILED;
    }

    if(!recover_bus())
    {
        return i2c_result::RECOVERY_FAILED;
    }

    mutex = xSemaphoreCreateMutexStatic(&mutex_storage);
    completion_semaphore =
        xSemaphoreCreateBinaryStatic(&completion_semaphore_storage);

    if(!mutex || !completion_semaphore)
    {
        mutex = nullptr;
        completion_semaphore = nullptr;
        return i2c_result::INIT_FAILED;
    }

    transfer_result = i2c_result::OK;
    initialized = true;
    return i2c_result::OK;
}

/**
 * @brief 使用 DMA 读取 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 接收长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::read_bytes(uint8_t device_address,
    uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    return transfer_bytes(i2c_transfer_direction::READ,
        device_address,
        register_address,
        data,
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 写入 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::write_bytes(uint8_t device_address,
    uint8_t register_address,
    const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    return transfer_bytes(i2c_transfer_direction::WRITE,
        device_address,
        register_address,
        const_cast<uint8_t *>(data),
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 判断 HAL I2C 句柄是否属于当前物理总线
 *
 * @param target_handle 待匹配的 HAL I2C 句柄
 *
 * @return 句柄匹配时返回 true
 */
bool i2c_dev::matches_handle(I2C_HandleTypeDef *target_handle) const
{
    return handle == target_handle;
}

/**
 * @brief 执行一次受互斥锁保护的 I2C DMA 传输
 *
 * @param direction DMA 传输方向
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 数据缓冲区
 * @param size 数据长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_dev::transfer_bytes(i2c_transfer_direction direction,
    uint8_t device_address,
    uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    if(device_address > 0x7F || !data || size == 0)
    {
        return i2c_result::INVALID_ARGUMENT;
    }
    if(__get_IPSR() != 0 ||
        xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return i2c_result::INVALID_CONTEXT;
    }

    // 总线恢复失败后允许后续任务重新初始化 HAL 和 DMA，避免永久停留在未初始化状态。
    if(!initialized)
    {
        i2c_result init_result = init();
        if(init_result != i2c_result::OK)
        {
            return init_result;
        }
    }

    TickType_t lock_timeout = milliseconds_to_ticks(lock_timeout_ms);
    if(xSemaphoreTake(mutex, lock_timeout) != pdTRUE)
    {
        return i2c_result::LOCK_TIMEOUT;
    }

    while(xSemaphoreTake(completion_semaphore, 0) == pdTRUE)
    {
    }

    uint16_t hal_device_address = (uint16_t)(device_address << 1);
    i2c_result result = i2c_result::BUS_ERROR;
    for(uint8_t attempt = 0;
        attempt < I2C_TRANSFER_ATTEMPT_COUNT;
        attempt++)
    {
        while(xSemaphoreTake(completion_semaphore, 0) == pdTRUE)
        {
        }

        // 先检查线路和外设状态，避免 HAL 在残留 BUSY 状态下长时间等待。
        if((!i2c_lines_released(handle) || i2c_peripheral_busy(handle)) &&
            !recover_bus())
        {
            result = i2c_result::RECOVERY_FAILED;
            break;
        }

        if(!i2c_lines_released(handle) || i2c_peripheral_busy(handle))
        {
            result = i2c_result::RECOVERY_FAILED;
            break;
        }

        transfer_result = i2c_result::BUSY;
        transfer_active = true;

        HAL_StatusTypeDef hal_status;
        if(direction == i2c_transfer_direction::READ)
        {
            hal_status = HAL_I2C_Mem_Read_DMA(handle,
                hal_device_address,
                register_address,
                I2C_MEMADD_SIZE_8BIT,
                data,
                size);
        }
        else
        {
            hal_status = HAL_I2C_Mem_Write_DMA(handle,
                hal_device_address,
                register_address,
                I2C_MEMADD_SIZE_8BIT,
                data,
                size);
        }

        if(hal_status != HAL_OK)
        {
            result = map_hal_status(handle, hal_status);
            cancel_active_transfer();
        }
        else
        {
            TickType_t transfer_timeout =
                milliseconds_to_ticks(transfer_timeout_ms);
            if(xSemaphoreTake(completion_semaphore,
                transfer_timeout) != pdTRUE)
            {
                result = i2c_result::TRANSFER_TIMEOUT;
                cancel_active_transfer();
            }
            else
            {
                result = transfer_result;
            }
        }

        if(result == i2c_result::OK)
        {
            break;
        }

        if(!recover_bus())
        {
            result = i2c_result::RECOVERY_FAILED;
            break;
        }

        if(attempt + 1 >= I2C_TRANSFER_ATTEMPT_COUNT)
        {
            break;
        }
    }

    xSemaphoreGive(mutex);
    return result;
}

/**
 * @brief 在异常传输后重新初始化 I2C 外设及其 DMA
 *
 * @return 恢复成功时返回 true
 */
bool i2c_dev::recover_bus()
{
    GPIO_TypeDef *gpio_port = nullptr;
    uint16_t scl_pin = 0;
    uint16_t sda_pin = 0;
    if(!get_i2c_gpio_config(handle,
        gpio_port,
        scl_pin,
        sda_pin))
    {
        return false;
    }

    if(HAL_I2C_DeInit(handle) != HAL_OK)
    {
        return false;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = scl_pin | sda_pin;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_WritePin(gpio_port,
        scl_pin | sda_pin,
        GPIO_PIN_RESET);
    HAL_GPIO_Init(gpio_port, &gpio_init);

    // 先用推挽输出制造低电平和 STOP，清除从设备残留的半帧状态。
    HAL_GPIO_WritePin(gpio_port,
        scl_pin | sda_pin,
        GPIO_PIN_RESET);
    sys_time::delay_us(I2C_RECOVERY_FORCE_DELAY_US);
    HAL_GPIO_WritePin(gpio_port, scl_pin, GPIO_PIN_SET);
    sys_time::delay_us(I2C_RECOVERY_FORCE_DELAY_US);
    HAL_GPIO_WritePin(gpio_port, sda_pin, GPIO_PIN_SET);
    sys_time::delay_us(I2C_RECOVERY_FORCE_DELAY_US);

    // 切回开漏释放模式；若 SDA 被从设备拉低，则发送最多 9 个时钟脉冲。
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(gpio_port, &gpio_init);
    HAL_GPIO_WritePin(gpio_port,
        scl_pin | sda_pin,
        GPIO_PIN_SET);
    sys_time::delay_us(I2C_RECOVERY_DELAY_US);
    for(uint8_t pulse = 0;
        pulse < I2C_RECOVERY_CLOCK_PULSES;
        pulse++)
    {
        if(HAL_GPIO_ReadPin(gpio_port, scl_pin) == GPIO_PIN_SET &&
           HAL_GPIO_ReadPin(gpio_port, sda_pin) == GPIO_PIN_SET)
        {
            break;
        }

        HAL_GPIO_WritePin(gpio_port, scl_pin, GPIO_PIN_RESET);
        sys_time::delay_us(I2C_RECOVERY_DELAY_US);
        HAL_GPIO_WritePin(gpio_port, scl_pin, GPIO_PIN_SET);
        sys_time::delay_us(I2C_RECOVERY_DELAY_US);
    }

    // SCL 为高时先拉低再释放 SDA，形成一个明确的 STOP 条件。
    HAL_GPIO_WritePin(gpio_port, scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(gpio_port, sda_pin, GPIO_PIN_RESET);
    sys_time::delay_us(I2C_RECOVERY_DELAY_US);
    HAL_GPIO_WritePin(gpio_port, scl_pin, GPIO_PIN_SET);
    sys_time::delay_us(I2C_RECOVERY_DELAY_US);
    HAL_GPIO_WritePin(gpio_port, sda_pin, GPIO_PIN_SET);
    sys_time::delay_us(I2C_RECOVERY_DELAY_US);

    bool lines_released =
        HAL_GPIO_ReadPin(gpio_port, scl_pin) == GPIO_PIN_SET &&
        HAL_GPIO_ReadPin(gpio_port, sda_pin) == GPIO_PIN_SET;
    HAL_GPIO_DeInit(gpio_port, scl_pin | sda_pin);

    bool peripheral_reset = reset_i2c_peripheral(handle);
    bool i2c_initialized = peripheral_reset && HAL_I2C_Init(handle) == HAL_OK;
    bool recovered = lines_released && i2c_initialized &&
        i2c_lines_released(handle) && !i2c_peripheral_busy(handle);

    if(completion_semaphore)
    {
        while(xSemaphoreTake(completion_semaphore, 0) == pdTRUE)
        {
        }
    }

    transfer_active = false;
    transfer_result = i2c_result::OK;

    if(!recovered)
    {
        initialized = false;
    }

    return recovered;
}

/**
 * @brief 在任务上下文中取消当前活动传输标记
 */
void i2c_dev::cancel_active_transfer()
{
    taskENTER_CRITICAL();
    transfer_active = false;
    taskEXIT_CRITICAL();
}

/**
 * @brief 在中断中完成当前 I2C DMA 传输
 *
 * @param result DMA 传输结果
 */
void i2c_dev::complete_from_isr(i2c_result result)
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
 * @brief 创建 I2C 总线访问对象
 *
 * @param bus_id I2C 总线编号
 */
i2c_bus::i2c_bus(uint8_t bus_id)
    : bus_id(bus_id)
{
}

/**
 * @brief 初始化 I2C 总线的 FreeRTOS 同步资源
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::init()
{
    i2c_dev *device = get_dev(bus_id);
    if(!device)
    {
        return i2c_result::INVALID_BUS;
    }

    return device->init();
}

/**
 * @brief 使用 DMA 连续读取 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 接收长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::read_bytes(uint8_t device_address,
    uint8_t register_address,
    uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    i2c_dev *device = get_dev(bus_id);
    if(!device)
    {
        return i2c_result::INVALID_BUS;
    }

    return device->read_bytes(device_address,
        register_address,
        data,
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 使用 DMA 连续写入 I2C 设备寄存器
 *
 * @param device_address 7 位设备地址
 * @param register_address 起始寄存器地址
 * @param data 发送缓冲区
 * @param size 发送长度
 * @param lock_timeout_ms 等待总线互斥锁的超时时间，单位毫秒
 * @param transfer_timeout_ms 等待 DMA 完成的超时时间，单位毫秒
 *
 * @return I2C 总线结果
 */
i2c_result i2c_bus::write_bytes(uint8_t device_address,
    uint8_t register_address,
    const uint8_t *data,
    uint16_t size,
    uint32_t lock_timeout_ms,
    uint32_t transfer_timeout_ms)
{
    i2c_dev *device = get_dev(bus_id);
    if(!device)
    {
        return i2c_result::INVALID_BUS;
    }

    return device->write_bytes(device_address,
        register_address,
        data,
        size,
        lock_timeout_ms,
        transfer_timeout_ms);
}

/**
 * @brief 处理 I2C DMA 寄存器读取完成事件
 *
 * @param handle HAL I2C 句柄
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *handle)
{
    i2c_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(i2c_result::OK);
    }
}

/**
 * @brief 处理 I2C DMA 寄存器写入完成事件
 *
 * @param handle HAL I2C 句柄
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *handle)
{
    i2c_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(i2c_result::OK);
    }
}

/**
 * @brief 处理 I2C DMA 或总线错误事件
 *
 * @param handle HAL I2C 句柄
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *handle)
{
    i2c_dev *device = get_dev(handle);
    if(device)
    {
        device->complete_from_isr(map_hal_error(handle));
    }
}
