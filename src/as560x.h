
#ifndef RPI_PICO_SERVO_AS560X_H
#define RPI_PICO_SERVO_AS560X_H
#define I2C_TIMEOUT_US (100000)

#define AS5601_ANGLE_MAX (0xFFFL)
#define AS560x_STATUS_MAGNET_DETECTED (0x20)
#define AS560x_STATUS_MAGNET_HIGH (0x08)
#define AS560x_STATUS_MAGNET_LOW (0x10)

/** Initializes sensor (i2c) bus
 */
void as560x_init();

_Noreturn void i2cError();

/** Returns raw angle from the sensor
 *
 * @return Raw angle value [0...AS5601_ANGLE_MAX]
 */
int as560xReadAngle();

/** Reads sensor status
 *
 * @return combo of AS560x_STATUS_MAGNET_DETECTED, AS560x_STATUS_MAGNET_HIGH, AS560x_STATUS_MAGNET_LOW
 */
uint8_t as560xGetStatus();

/** Debug purposes only
 */
__unused void sensorData();

#endif //RPI_PICO_SERVO_AS560X_H
