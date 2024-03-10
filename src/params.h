//
// Created by elmot on 23 Apr 2023.
//

#ifndef RPI_PICO_SERVO_PARAMS_H
#define RPI_PICO_SERVO_PARAMS_H
//degrees
// Dead angle around zero position
#define ZERO_RESTRICTED_ANGLE  (5)

//Stop moving when the axle angle is less or equal than:
#define ANGLE_TOLERANCE  (2)
//Start moving when the axle angle is more than:
#define DEAD_ANGLE  (5)
//Slow down the motor when actual angle is close to the target
#define SLOW_ANGLE  (20)

//PWM percents: 100 => max power
#define NO_PWM  (30)
#define SLOW_START_PWM  (30)
#define SLOW_PWM  (10)
#define FAST_PWM  (100)

// Test settings for prototype device
//#define SLOW_PWM  (75)
//#define FAST_PWM  (60)

//Slow start milliseconds
#define SLOW_START_MS (200)

#endif //RPI_PICO_SERVO_PARAMS_H
