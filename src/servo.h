#ifndef RPI_PICO_SERVO_SERVO_H
#define RPI_PICO_SERVO_SERVO_H
#include <pico/assert.h>
#include <pico/printf.h>
#include <hardware/watchdog.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/structs/clocks.h"
#include "hardware/clocks.h"

#define PWM_IN_PIN (21)

#define I2C i2c1
#define I2C_SDA_PIN (26)
#define I2C_SCL_PIN (27)

#define LED_PIN (25)
#define MOTOR_A_PIN (16)
#define MOTOR_B_PIN (17)

#define DEFAULT_PWM (1500)

extern volatile uint16_t pwm_count;

void init_pwm(void);

void setMotorPwm(unsigned int pwm_a,unsigned int pwm_b);

#endif //RPI_PICO_SERVO_SERVO_H
