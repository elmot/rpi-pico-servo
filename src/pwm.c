#include "servo.h"

volatile uint pwm_in_slice_num;
volatile uint16_t pwm_count;

uint motor_pwm_slice_num;
uint motor_a_channel;
uint motor_b_channel;

void pwm_fall_handler() {
    pwm_count = pwm_get_counter(pwm_in_slice_num);
    pwm_set_counter(pwm_in_slice_num, 0);
}

void pwm_irq_handler() {
    pwm_clear_irq(pwm_in_slice_num);
    pwm_count = 0;
}

static inline void init_pwm_out() {
    // Find out which PWM slice is connected to MOTOR_GPIO_A
    gpio_set_function(MOTOR_A_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_B_PIN, GPIO_FUNC_PWM);
    motor_pwm_slice_num = pwm_gpio_to_slice_num(MOTOR_A_PIN);
    assert(motor_pwm_slice_num == pwm_gpio_to_slice_num(MOTOR_B_PIN));

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

static inline void init_pwm_in() {
    assert(pwm_gpio_to_channel(PWM_IN) == PWM_CHAN_B);
    pwm_in_slice_num = pwm_gpio_to_slice_num(PWM_IN_PIN);
    // Count once for every 100 cycles the PWM B input is high
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_B_HIGH);
    double f_sys_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    // Set base freq 1 MHz,
    pwm_config_set_clkdiv(&cfg, (float) (f_sys_khz / 1000.0));
    pwm_init(pwm_in_slice_num, &cfg, false);
    gpio_set_function(PWM_IN_PIN, GPIO_FUNC_PWM);
    pwm_clear_irq(pwm_in_slice_num);
    pwm_set_irq_enabled(pwm_in_slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    gpio_set_irq_enabled_with_callback(PWM_IN_PIN, GPIO_IRQ_EDGE_FALL, true, pwm_fall_handler);
    pwm_set_enabled(pwm_in_slice_num, true);
}

void init_pwm() {
    init_pwm_in();
    init_pwm_out();
    assert(motor_pwm_slice_num != pwm_in_slice_num);
}

void setMotorPwm(unsigned int pwm_a,unsigned int pwm_b) {
    pwm_set_chan_level(motor_pwm_slice_num, motor_a_channel, pwm_a);
    pwm_set_chan_level(motor_pwm_slice_num, motor_b_channel, pwm_b);
}
