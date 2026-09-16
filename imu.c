#include "imu.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "robot.h"
#include "status.h"

static bno055_t sensor;
static bool     ready;

bool imu_begin(void) {
    i2c_init(IMU_I2C, IMU_I2C_BAUD);
    gpio_set_function(LINK101_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LINK101_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LINK101_PIN_SDA);
    gpio_pull_up(LINK101_PIN_SCL);

    // bno055_init retries for a few hundred ms -- the chip boots slowly.
    ready = bno055_init(&sensor, IMU_I2C, IMU_ADDR);
    status_printf("[imu  ] BNO055 at 0x%02X: %s\n", IMU_ADDR,
                  ready ? "online" : "no response");
    return ready;
}

bool imu_online(void) {
    return ready;
}

bool imu_read(bno055_sample_t *out) {
    return ready && bno055_read(&sensor, out);
}
