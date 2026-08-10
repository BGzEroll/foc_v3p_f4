#ifndef SSD1306_H
#define SSD1306_H

#include "drivers/bus/spi_bus.h"

class ssd1306
{
    public:
        ssd1306(uint8_t spi_bus_num,
            GPIO_TypeDef *cs_port,
            uint16_t cs_pin,
            GPIO_TypeDef *res_port,
            uint16_t res_pin,
            GPIO_TypeDef *dc_port,
            uint16_t dc_pin);

    public:
        spi_result init();

    public:
        void clear();
        spi_result flush();
        spi_result display_on();
        spi_result display_off();
        void show_string(uint8_t line, uint8_t column, const char *string);
        void show_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
        void show_signed_num(uint8_t line, uint8_t column, int32_t number, uint8_t length);
        void show_hex_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
        void show_bin_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

    private:
        spi_result write_commands(const uint8_t *commands, uint16_t size);
        spi_result write_data(const uint8_t *data, uint16_t size);
        void show_char(uint8_t line, uint8_t column, char ascii_char);
        uint32_t pow_uint(uint32_t x, uint32_t y);

    private:
        spi_bus spi;
        GPIO_TypeDef *cs_port;
        uint16_t cs_pin;
        GPIO_TypeDef *res_port;
        uint16_t res_pin;
        GPIO_TypeDef *dc_port;
        uint16_t dc_pin;
        bool initialized = false;
        uint8_t fb[8][128];
        uint8_t dirty[8];
};

#endif
