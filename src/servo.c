#include <machine/endian.h>
#include <pico/printf.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define I2C i2c1
#define I2C_SDA_PIN (26)
#define I2C_SCL_PIN (27)
#define  AS5601_ADDR 0x36

uint16_t as5601ReadReg(int addr, bool wide, uint16_t mask, bool *error) {
    uint16_t buf;
    int result = i2c_write_blocking(I2C, AS5601_ADDR , (uint8_t *) &addr, 1, true);
    result = i2c_read_blocking(I2C, AS5601_ADDR , (uint8_t *) &buf, (wide ? 2 : 1), false);
    *error = result <= 0 ;
    if (wide) {
        return __bswap16(buf) & mask;
    } else {
        return buf & mask;
    }
}

void AS5601_print_reg8(const char *formatStr, int addr, uint8_t mask) {
    bool error;
    uint8_t result = (uint8_t) as5601ReadReg(addr, false, mask, &error);
    if (error) {
        printf("ERROR\n\r");
    } else {
        printf(formatStr, result & mask);
    }
}

void AS5601_print_reg16(const char *formatStr, int addr, uint16_t mask) {
    bool error;
    uint16_t result = as5601ReadReg(addr, true, mask, &error);
    if (error) {
        printf("ERROR\n\r");
    } else {
        printf(formatStr, result & mask);
    }
}

void sensorLoop() {
    AS5601_print_reg16("CONF: %04x; ", 0x7,0xFFFF);
    AS5601_print_reg8("Status: %02x; ", 0xb,0x38);
    AS5601_print_reg8("AGC: %3x; ", 0x1a,0xff);
    AS5601_print_reg16("Raw Angle: %04x; ", 0x0c, 0xFFF);
    AS5601_print_reg16("Angle: %04x\n\r", 0x0e, 0xFFF);

}


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
int main() {
    stdio_init_all();
    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));

    while (1) {
        sensorLoop();
        sleep_ms(500);
    }

}
#pragma clang diagnostic pop