#ifndef SPI_BUS_H
#define SPI_BUS_H

#include "stm32f4xx_hal.h"

enum class spi_result : uint8_t
{
    OK = 0,
    INVALID_BUS,
    INVALID_ARGUMENT,
    NOT_INITIALIZED,
    INVALID_CONTEXT,
    INIT_FAILED,
    LOCK_TIMEOUT,
    TRANSACTION_NOT_ACTIVE,
    TRANSACTION_OWNER_MISMATCH,
    TRANSFER_TIMEOUT,
    BUSY,
    DMA_ERROR,
    OVERRUN_ERROR,
    MODE_FAULT,
    FRAME_ERROR,
    CRC_ERROR,
    BUS_ERROR,
    RECOVERY_FAILED
};

class spi_bus
{
    public:
        static constexpr uint32_t DEFAULT_LOCK_TIMEOUT_MS = 10;
        static constexpr uint32_t DEFAULT_TRANSFER_TIMEOUT_MS = 50;

        explicit spi_bus(uint8_t bus_id = 0);

    public:
        spi_result init();

    public:
        spi_result cs_low(GPIO_TypeDef *port,
            uint16_t pin,
            uint32_t lock_timeout_ms = DEFAULT_LOCK_TIMEOUT_MS);
        spi_result cs_high(GPIO_TypeDef *port, uint16_t pin);
        spi_result rx(uint8_t *data,
            uint16_t size,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
        spi_result tx(const uint8_t *data,
            uint16_t size,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);
        spi_result rx_tx(const uint8_t *tx_data,
            uint8_t *rx_data,
            uint16_t size,
            uint32_t transfer_timeout_ms = DEFAULT_TRANSFER_TIMEOUT_MS);

    private:
        uint8_t bus_id;
};

#endif
