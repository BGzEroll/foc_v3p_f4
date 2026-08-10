#include "i2c_bus.h"

#define QUEUE_SIZE           16

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

class i2c_dev
{
    public:
        i2c_dev(I2C_HandleTypeDef *i2c_handle)
            : i2c_handle(i2c_handle){}

        void init()
        {
            // i2c 初始化由 CubeMX 管理
            if(is_init || !i2c_handle){return;}
            is_init = true;
        }
        
        bool submit_dma_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len, volatile int8_t *dma_done)
        {
            if(!i2c_handle || !buf || !len){return false;}

            dma_req req = {addr, reg, buf, len, dma_done};
            bool need_start = false;

            __disable_irq();
            bool ok = queue_push(req);
            if(ok && !dma_rx_busy)
            {
                dma_rx_busy = true;
                need_start = true;
            }
            __enable_irq();

            if(need_start)
            {
                start_next_transfer();
            }

            return ok;
        }

        bool read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
        {
            if(!i2c_handle || !buf || !len){return false;}
            if(dma_rx_busy){return false;}

            if(HAL_I2C_Mem_Read(i2c_handle, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY) == HAL_OK)
            {
                return true;
            }
            
            return false;
        }

        void write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
        {
            if(!i2c_handle || (!buf && len > 0)){return;}
            if(dma_rx_busy){return;}

            HAL_I2C_Mem_Write(i2c_handle, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, HAL_MAX_DELAY);
        }

    private:
        struct dma_req
        {
            uint8_t addr;
            uint8_t reg;
            uint8_t *buf;
            uint8_t len;
            volatile int8_t *dma_done;
        };

    private:
        bool queue_empty() const
        {
            return queue_head == queue_tail;
        }

        bool queue_full() const
        {
            return (queue_tail + 1) % QUEUE_SIZE == queue_head;
        }

        bool queue_push(const dma_req &req)
        {
            if(queue_full()){return false;}
            queue[queue_tail] = req;
            queue_tail = (queue_tail + 1) % QUEUE_SIZE;
            return true;
        }

        bool queue_pop(dma_req &req)
        {
            if(queue_empty()){return false;}
            req = queue[queue_head];
            queue_head = (queue_head + 1) % QUEUE_SIZE;
            return true;
        }

        void start_next_transfer()
        {
            dma_req req;

            __disable_irq();
            bool ok = queue_pop(req);
            __enable_irq();

            if(!ok)
            {
                dma_rx_busy = false;
                return;
            }

            current_req = req;

            if(HAL_I2C_Mem_Read_DMA(i2c_handle, current_req.addr << 1, current_req.reg, I2C_MEMADD_SIZE_8BIT, current_req.buf, current_req.len) == HAL_OK)
            {
                if(current_req.dma_done){*current_req.dma_done = I2C_DMA_BUSY;}
                return;
            }

            if(current_req.dma_done){*current_req.dma_done = I2C_DMA_ERROR;}

            dma_rx_busy = false;
        }

        void on_dma_done()
        {
            if(current_req.dma_done){*current_req.dma_done = I2C_DMA_OK;}
            start_next_transfer();
        }

        void on_dma_error()
        {
            if(current_req.dma_done){*current_req.dma_done = I2C_DMA_ERROR;}
            start_next_transfer();
        }

    private:
        I2C_HandleTypeDef *i2c_handle;
        bool is_init = false;
        volatile bool dma_rx_busy = false;
        dma_req queue[QUEUE_SIZE];
        volatile uint8_t queue_head = 0;
        volatile uint8_t queue_tail = 0;
        dma_req current_req;

    private:
        friend void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
        friend void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
};

// 静态 i2c 设备表（资源池）
static i2c_dev i2c_devs[] =
{
    i2c_dev(&hi2c1),
    i2c_dev(&hi2c2)
};
static constexpr uint8_t I2C_DEV_NUM = sizeof(i2c_devs) / sizeof(i2c_devs[0]);

/**
 * @brief 根据 bus_id 获取对应底层设备
 *
 * @note 超出范围时默认返回 bus0；
 * @note 保证始终返回有效指针；
 */
static i2c_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < I2C_DEV_NUM)
    {
        return &i2c_devs[bus_id];
    }
    return &i2c_devs[0];
}

/**
 * @brief i2c 总线构造函数
 * 
 * @param bus_id i2c 总线编号
 */
i2c_bus::i2c_bus(uint8_t bus_id)
    : bus_id(bus_id){}

/**
 * @brief 初始化 i2c 总线
 */
void i2c_bus::init()
{
    get_dev(bus_id)->init();
}

/**
 * @brief 提交 dma 读取请求
 * 
 * @param addr 设备地址
 * @param reg  起始寄存器地址
 * @param buf  接收缓冲区
 * @param len  读取长度
 * @param dma_done  是否成功完成指针
 * 
 * @return true 成功
 * @return false 失败
 * 
 * @note dma_done 有三种状态：
 * @note 0 未完成
 * @note 1 完成
 * @note -1 错误
 */
bool i2c_bus::submit_dma_read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len, volatile int8_t *dma_done)
{
    return get_dev(bus_id)->submit_dma_read_bytes(addr, reg, buf, len, dma_done);
}

/**
 * @brief 连续读取寄存器数据
 * 
 * @param addr 设备地址
 * @param reg  起始寄存器地址
 * @param buf  接收缓冲区
 * @param len  读取长度
 * 
 * @return true 成功
 * @return false 失败
 */
bool i2c_bus::read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    return get_dev(bus_id)->read_bytes(addr, reg, buf, len);
}

/**
 * @brief 连续写入寄存器数据
 * 
 * @param addr 设备地址
 * @param reg  起始寄存器地址
 * @param buf  数据缓冲区
 * @param len  写入长度
 */
void i2c_bus::write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    get_dev(bus_id)->write_bytes(addr, reg, buf, len);
}

/**
 * @brief i2c dma 接收完成回调
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_dev *p = nullptr;
    if(hi2c->Instance == I2C1){p = get_dev(0);}
    else if(hi2c->Instance == I2C2){p = get_dev(1);}

    if(!p || !p->is_init){return;}

    p->on_dma_done();
}

/**
 * @brief i2c dma 接收错误回调
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_dev *p = nullptr;
    if(hi2c->Instance == I2C1){p = get_dev(0);}
    else if(hi2c->Instance == I2C2){p = get_dev(1);}
    
    if(!p || !p->is_init){return;}

    p->on_dma_error();
}
