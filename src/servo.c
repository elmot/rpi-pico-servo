#include "servo.h"

#define I2C_TIMEOUT_US (100000)

#define AS5601_ADDR 0x36
#define AS5601_STATUS_REG (0x0B)
#define AS5601_RAW_ANGLE_REG (0x0C)
#define AS5601_ANGLE_MAX (0xFFFL)
#define AS5601_STATUS_MAGNET_DETECTED (0x20)
#define AS5601_STATUS_MAGNET_HIGH (0x08)
#define AS5601_STATUS_MAGNET_LOW (0x10)

//degrees
#define ANGLE_TOLERANCE  (35)
#define DEAD_ANGLE  (5)
#define SLOW_ANGLE  (30)

#define SLOW_PWM  (60)
#define FAST_PWM  (75)

_Noreturn void i2cError();

_Noreturn void magnetError();

uint16_t as5601ReadReg(int addr, bool wide, uint16_t mask);

/** Debug purposes only
 */
__unused void sensorLoop();

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
int target_angle = 270;//todo

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

    while(1) {
        printf("PWM_IN: %d\n\r", pwm_count);
    }
    while (1) {
        uint8_t status = (uint8_t) as5601ReadReg(AS5601_STATUS_REG, false, 0x38);
        if (!(status & AS5601_STATUS_MAGNET_DETECTED)) {
            printf("ERROR\n\r");
            magnetError();
        }
        //todo check getting stuck
        int angle = as5601ReadReg(AS5601_RAW_ANGLE_REG, true, 0xFFF) * 360L / AS5601_ANGLE_MAX;
        int angle_delta = (360 + target_angle - angle) % 360;
        if (angle_delta > 180) { angle_delta = angle_delta - 360; }
        int angle_delta_abs = abs(angle_delta);
        unsigned int pwm_a = 0, pwm_b = 0;
        if (angle_delta_abs > ANGLE_TOLERANCE) {
            gpio_put(LED_PIN, 1);
            int pwm = angle_delta_abs > SLOW_ANGLE ? FAST_PWM : SLOW_PWM;
            if (angle_delta > 0) {
                pwm_a = pwm;
            } else {
                pwm_b = pwm;
            }
        } else {
            gpio_put(LED_PIN, 0);
        }
        setMotorPwm(pwm_a, pwm_b);
    }
}

uint16_t as5601ReadReg(int addr, bool wide, uint16_t mask) {
    uint16_t buf;
    int result = i2c_write_timeout_us(I2C, AS5601_ADDR, (uint8_t *) &addr, 1, true, I2C_TIMEOUT_US);
    if (result <= 0) {
        i2cError();
    }
    result = i2c_read_timeout_us(I2C, AS5601_ADDR, (uint8_t *) &buf, (wide ? 2 : 1), false, I2C_TIMEOUT_US);
    if (result <= 0) {
        i2cError();
    }
    if (wide) {
        return __bswap16(buf) & mask;
    } else {
        return buf & mask;
    }
}

void AS5601_print_reg16(const char *formatStr, int addr, uint16_t mask) {
    uint16_t result = as5601ReadReg(addr, true, mask);
    printf(formatStr, result & mask);
}

void AS5601_print_reg8(const char *formatStr, int addr, uint8_t mask) {
    uint8_t result = (uint8_t) as5601ReadReg(addr, false, mask);
    printf(formatStr, result & mask);
}

__unused void sensorLoop() {
    AS5601_print_reg8("Status: %02x; ", 0xb, 0x38);
    AS5601_print_reg8("AGC: %3x; ", 0x1a, 0xff);
    AS5601_print_reg16("Angle: %04x\n\r", 0x0c, 0xFFF);
}

_Noreturn static inline void error(int phase_on_ms, int phase_off_ms) {
    watchdog_enable(500, 1);
    while (1) {
        gpio_put(LED_PIN, 1);
        sleep_ms(phase_on_ms);
        gpio_put(LED_PIN, 0);
        sleep_ms(phase_off_ms);
    }
}

_Noreturn void i2cError() {
    error(300, 30);
}

_Noreturn void magnetError() {
    error(700, 300);
}

#pragma clang diagnostic pop