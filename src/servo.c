#include "servo.h"
#include "params.h"
#include "as560x.h"

_Noreturn void magnetError();

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

volatile bool slow_start = true;
int64_t slow_start_alarm_handler(__unused alarm_id_t id,__unused void *user_data) {
    slow_start = false;
    return 0;
}

bool moving = true;

int main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    init_pwm();

    as560x_init();
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C))
    alarm_id_t slow_start_alarm = 0;
    while (1) {
        uint8_t status = (uint8_t) as560xGetStatus();
        if (!(status & AS560x_STATUS_MAGNET_DETECTED)) {
            printf("ERROR\n\r");
            magnetError();
        }
        if(pwm_count == 0) continue;
        else if(pwm_count < 1000) pwm_count = 1000;
        else if(pwm_count > 2000) pwm_count = 2000;
        int target_angle = ZERO_RESTRICTED_ANGLE + ((pwm_count - 1000) * (360 - 2 * ZERO_RESTRICTED_ANGLE)) / 1000;
        //todo check getting stuck
        int angle = as560xReadAngle() * 360L / AS5601_ANGLE_MAX;

        int angle_delta = target_angle - angle;
        int angle_delta_abs = abs(angle_delta);
        int angle_tolerance = moving ? ANGLE_TOLERANCE : DEAD_ANGLE;
        if (angle_delta_abs > angle_tolerance) {
            gpio_put(LED_PIN, 1);
            if(!moving) {
                slow_start = true;
                slow_start_alarm = add_alarm_in_ms(SLOW_START_MS, slow_start_alarm_handler,NULL,true);
            }
            moving = true;
            int pwm = (slow_start || (angle_delta_abs < SLOW_ANGLE)) ? SLOW_PWM : FAST_PWM;
            if (angle_delta > 0) {
                setMotorPwm(pwm, NO_PWM);
            } else {
                setMotorPwm(NO_PWM, pwm);
            }
        } else {
            gpio_put(LED_PIN, 0);
            setMotorPwm(NO_PWM, NO_PWM);
            moving = false;
            cancel_alarm(slow_start_alarm);
//            printf("Target angle: %d; Current angle: %d\n\r",target_angle,angle);
        }
    }
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