#include "spi_bus.h"

extern SPI_HandleTypeDef hspi1;

class spi_dev
{
    public:
        spi_dev(SPI_HandleTypeDef *spi_handle)
            : spi_handle(spi_handle){}

        void init()
        {
            // spi 初始化由 CubeMX 管理
            if(is_init || !spi_handle){return;}
            is_init = true;
        }

        void cs_low(GPIO_TypeDef *port, uint16_t pin)
        {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        }

        void cs_high(GPIO_TypeDef *port, uint16_t pin)
        {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        }

        void rx(uint8_t *buf, uint32_t len)
        {
            HAL_SPI_Receive(spi_handle, buf, len, HAL_MAX_DELAY);
        }

        void tx(const uint8_t *buf, uint32_t len)
        {
            HAL_SPI_Transmit(spi_handle, buf, len, HAL_MAX_DELAY);
        }

        void rx_tx(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
        {
            HAL_SPI_TransmitReceive(spi_handle, tx_buf, rx_buf, len, HAL_MAX_DELAY);
        }
    
    private:
        SPI_HandleTypeDef *spi_handle;
        bool is_init = false;
};

// 静态 spi 设备表（资源池）
static spi_dev spi_devs[] = 
{
    spi_dev(&hspi1)
};
static constexpr uint8_t SPI_DEV_NUM = sizeof(spi_devs) / sizeof(spi_devs[0]);

/**
 * @brief 根据 bus_id 获取对应底层设备
 *
 * @note 超出范围时默认返回 bus0；
 * @note 保证始终返回有效指针；
 */
static spi_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < SPI_DEV_NUM)
    {
        return &spi_devs[bus_id];
    }
    return &spi_devs[0];
}

/**
 * @brief spi 总线构造函数
 * 
 * @param bus_id spi 总线编号
 */
spi_bus::spi_bus(uint8_t bus_id)
    : bus_id(bus_id){}

/**
 * @brief 初始化 spi 总线
 */
void spi_bus::init()
{
    get_dev(bus_id)->init();
}

/**
 * @brief spi 拉低 cs 电平
 * 
 * @param port cs 引脚端口
 * @param pin cs 引脚编号
 */
void spi_bus::cs_low(GPIO_TypeDef *port, uint16_t pin)
{
    get_dev(bus_id)->cs_low(port, pin);
}

/**
 * @brief spi 拉高 cs 电平
 * 
 * @param port cs 引脚端口
 * @param pin cs 引脚编号
 */
void spi_bus::cs_high(GPIO_TypeDef *port, uint16_t pin)
{
    get_dev(bus_id)->cs_high(port, pin);
}

/**
 * @brief spi 接收数据
 * 
 * @param buf 接收缓冲区
 * @param len 接收长度
 */
void spi_bus::rx(uint8_t *buf, uint32_t len)
{
    get_dev(bus_id)->rx(buf, len);
}

/**
 * @brief spi 发送数据
 * 
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void spi_bus::tx(const uint8_t *buf, uint32_t len)
{
    get_dev(bus_id)->tx(buf, len);
}

/**
 * @brief spi 发送接收数据
 * 
 * @param tx_buf 发送缓冲区
 * @param rx_buf 接收缓冲区
 * @param len 数据长度
 */
void spi_bus::rx_tx(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    get_dev(bus_id)->rx_tx(tx_buf, rx_buf, len);
}
