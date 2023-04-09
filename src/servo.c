#include <machine/endian.h>
#include <pico/assert.h>
#include <pico/printf.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/structs/clocks.h"
#include "hardware/clocks.h"

#define I2C i2c1
#define I2C_SDA_PIN (26)
#define I2C_SCL_PIN (27)
#define LED_PIN (25)
#define AS5601_ADDR 0x36
#define MOTOR_A_PIN (16)
#define MOTOR_B_PIN (17)

uint16_t as5601ReadReg(int addr, bool wide, uint16_t mask, bool *error) {
    uint16_t buf;
    int result = i2c_write_blocking(I2C, AS5601_ADDR, (uint8_t *) &addr, 1, true);
    *error = result <= 0;
    if (*error) {
        return 0;
    }
    result = i2c_read_blocking(I2C, AS5601_ADDR, (uint8_t *) &buf, (wide ? 2 : 1), false);
    *error = result <= 0;
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
    AS5601_print_reg8("Status: %02x; ", 0xb, 0x38);
    AS5601_print_reg8("AGC: %3x; ", 0x1a, 0xff);
    AS5601_print_reg16("Angle: %04x\n\r", 0x0c, 0xFFF);
}


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

uint motor_pwm_slice_num;
uint motor_a_channel;
uint motor_b_channel;
void init_pwm() {
    // Find out which PWM slice is connected to MOTOR_GPIO_A
    gpio_set_function(MOTOR_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_B_PIN, GPIO_FUNC_PWM);
    motor_pwm_slice_num = pwm_gpio_to_slice_num(MOTOR_A_PIN);
    uint slice_num_b = pwm_gpio_to_slice_num(MOTOR_A_PIN);
    hard_assert(motor_pwm_slice_num == slice_num_b, "Motor GPIOs have to use same PWM channels");

    motor_a_channel = pwm_gpio_to_channel(MOTOR_A_PIN);
    motor_b_channel = pwm_gpio_to_channel(MOTOR_B_PIN);


    double f_sys_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    // Set PWM base freq 2 MHz, PWM freq 20 kHz,
    pwm_set_clkdiv(motor_pwm_slice_num, (float) (f_sys_khz / 2000.0));
    pwm_set_wrap(motor_pwm_slice_num, 100);

    // Set the PWM running
    pwm_set_enabled(motor_pwm_slice_num, true);
    pwm_set_chan_level(motor_pwm_slice_num, motor_a_channel, 0);
    pwm_set_chan_level(motor_pwm_slice_num, motor_b_channel, 0);

}

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    init_pwm();

    i2c_init(i2c1, 100 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C))

    while (1) {
        sensorLoop();
        gpio_put(LED_PIN, 1);
        pwm_set_chan_level(motor_pwm_slice_num, motor_a_channel, 75);
        pwm_set_chan_level(motor_pwm_slice_num, motor_b_channel, 0);
        for (int i = 0; i < 30; ++i) {
            sleep_ms(20);
            sensorLoop();
        }
        gpio_put(LED_PIN, 0);
        pwm_set_chan_level(motor_pwm_slice_num, motor_a_channel, 0);
        pwm_set_chan_level(motor_pwm_slice_num, motor_b_channel, 65);
        sleep_ms(20);
        for (int i = 0; i < 130; ++i) {
            sleep_ms(20);
            sensorLoop();
        }
    }

}

#pragma clang diagnostic pop