#ifndef SSD1306_H
#define SSD1306_H

#include "stm32f4xx_hal.h"
#include "bus/spi_bus.h"

class ssd1306
{
    public:
        ssd1306(uint8_t spi_bus_num,
                GPIO_TypeDef *cs_port, uint16_t cs_pin,
                GPIO_TypeDef *res_port, uint16_t res_pin,
                GPIO_TypeDef *dc_port, uint16_t dc_pin);

    public:
        void init();
        void clear();
        void flush();
        void display_on();
        void display_off();
        void show_string(uint8_t line, uint8_t column, const char *string);
        void show_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
        void show_signed_num(uint8_t line, uint8_t column, int32_t number, uint8_t length);
        void show_hex_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);
        void show_bin_num(uint8_t line, uint8_t column, uint32_t number, uint8_t length);

    private:
        void write_command(uint8_t command);
        void write_commands(const uint8_t *commands, uint8_t len);
        void write_data(const uint8_t *data, uint8_t len);
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
        bool is_init = false;
        uint8_t fb[8][128];
        uint8_t dirty[8];
};

#endif
