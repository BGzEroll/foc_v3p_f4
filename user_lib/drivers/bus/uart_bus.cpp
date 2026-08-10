#include "uart_bus.h"

#define UART_RX_BUFFER_SIZE     1024
#define UART_TX_BUFFER_SIZE     1024

extern UART_HandleTypeDef huart1;

class uart_dev
{
    public:
        uart_dev(UART_HandleTypeDef *uart_handle)
            : uart_handle(uart_handle){}

    public:
        void init()
        {
            if(is_init || !uart_handle){return;}

            if(HAL_UART_Receive_IT(uart_handle, &rx_byte, 1) == HAL_OK)
            {
                is_init = true;
            }
        }

        uint32_t read_bytes(uint8_t *buf, uint32_t max_len)
        {
            uint32_t count = 0;

            while(count < max_len)
            {
                if(rx_head == rx_tail){break;}

                buf[count++] = rx_buffer[rx_tail];
                rx_tail = (rx_tail + 1) % UART_RX_BUFFER_SIZE;
            }

            return count;
        }

        void write_bytes(const uint8_t *buf, uint32_t len)
        {
            bool need_kick = false;

            uint32_t primask = __get_PRIMASK();
            __disable_irq();

            for(uint32_t i = 0; i < len; i++)
            {
                uint32_t next = (tx_head + 1) % UART_TX_BUFFER_SIZE;
                if(next == tx_tail){break;}     // 缓冲区满，丢弃数据
                tx_buffer[tx_head] = buf[i];
                tx_head = next;
            }

            if(!dma_tx_busy)
            {
                dma_tx_busy = true;
                need_kick = true;
            }

            __set_PRIMASK(primask);

            if(need_kick)
            {
                dma_start_send();
            }
        }
        
    private:
        void on_rx_byte()
        {
            uint32_t next = (rx_head + 1) % UART_RX_BUFFER_SIZE;
            if(next == rx_tail)
            {
                rx_overflow++;
                return;
            }

            rx_buffer[rx_head] = rx_byte;
            rx_head = next;
        }
        
        void dma_start_send()
        {
            // 无数据可发送
            if(tx_head == tx_tail)
            {
                dma_tx_busy = false;
                return;
            }

            uint32_t len;
            if(tx_head > tx_tail)
            {
                len = tx_head - tx_tail;       // 数据连续 
            }
            else
            {
                len = UART_TX_BUFFER_SIZE - tx_tail;        // 跨界，先发送到缓冲区末尾
            }

            dma_tx_chunk_size = len;        // 保存本次 dma 发送的长度

            if(HAL_UART_Transmit_DMA(uart_handle, &tx_buffer[tx_tail], len) != HAL_OK)
            {
                dma_tx_busy = false;
                dma_tx_chunk_size = 0;
            }
        }

    private:
        UART_HandleTypeDef *uart_handle = nullptr;
        bool is_init = false;
        uint8_t rx_buffer[UART_RX_BUFFER_SIZE];
        volatile uint32_t rx_head = 0;
        volatile uint32_t rx_tail = 0;
        uint8_t rx_byte = 0;
        volatile uint32_t rx_overflow = 0;
        uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
        volatile uint32_t tx_head = 0;
        volatile uint32_t tx_tail = 0;
        volatile bool dma_tx_busy = false;
        volatile uint32_t dma_tx_chunk_size = 0;

    private:
        friend void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
        friend void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
        friend void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
};

// 静态 uart 设备表（资源池）
static uart_dev uart_devs[] =
{
    uart_dev(&huart1)
};
static constexpr uint8_t UART_DEV_NUM = sizeof(uart_devs) / sizeof(uart_devs[0]);

/**
 * @brief 根据 bus_id 获取对应底层设备
 *
 * @note 超出范围时默认返回 bus0；
 * @note 保证始终返回有效指针；
 */
static uart_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < UART_DEV_NUM)
    {
        return &uart_devs[bus_id];
    }
    return &uart_devs[0];
}

/**
 * @brief uart 总线构造函数
 * 
 * @param bus_id uart 总线编号
 */
uart_bus::uart_bus(uint8_t bus_id)
    : bus_id(bus_id){}

/**
 * @brief uart 总线初始化
 */
void uart_bus::init()
{
    get_dev(bus_id)->init();
}

/**
 * @brief uart 总线读取数据
 * 
 * @param buf 数据缓冲区
 * @param max_len 最大读取长度
 * 
 * @return uint32_t 实际读取长度
 */
uint32_t uart_bus::read_bytes(uint8_t *buf, uint32_t max_len)
{
    return get_dev(bus_id)->read_bytes(buf, max_len);
}

/**
 * @brief uart 总线写入数据
 * 
 * @param buf 数据缓冲区
 * @param len 数据长度
 */
void uart_bus::write_bytes(const uint8_t *buf, uint32_t len)
{
    get_dev(bus_id)->write_bytes(buf, len);
}

/**
 * @brief uart 接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_dev *p = nullptr;
    if(huart->Instance == USART1){p = &uart_devs[0];}

    if(!p || !p->is_init){return;}

    p->on_rx_byte();

    HAL_UART_Receive_IT(huart, &p->rx_byte, 1);
}

/**
 * @brief uart 错误回调
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uart_dev *p = nullptr;
    if(huart->Instance == USART1){p = &uart_devs[0];}

    if(!p || !p->is_init){return;}

    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(huart, &p->rx_byte, 1);
}

/**
 * @brief uart dma 发送完成回调
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_dev *p = nullptr;
    if(huart->Instance == USART1){p = &uart_devs[0];}

    if(!p || !p->is_init){return;}

    p->tx_tail = (p->tx_tail + p->dma_tx_chunk_size) % UART_TX_BUFFER_SIZE;
    p->dma_tx_chunk_size = 0;

    p->dma_start_send();        // 发送剩余数据
}
