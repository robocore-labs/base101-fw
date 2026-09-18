#include "imu.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "link101/lsm6dsox.h"
#include "link101/mmc5983.h"

#include "robot.h"
#include "status.h"
#ifdef IMU_DIAGNOSTIC
#include "imu_diagnostic_trace.h"
#endif

static link101_lsm6dsox_t sensor;
#ifndef IMU_DIAGNOSTIC_NO_MAG
static link101_mmc5983_t  magnetometer;
#endif
static bool               sensor_ready;
static bool               mag_ready;
static imu_health_t       health;
static uint64_t next_retry_us;
#ifdef IMU_DIAGNOSTIC
static uint32_t diagnostic_baud = IMU_I2C_BAUD;
void imu_diagnostic_set_baud(uint32_t baud) {
    diagnostic_baud = baud;
    i2c_set_baudrate(IMU_I2C, baud);
}
uint32_t imu_diagnostic_get_baud(void) { return diagnostic_baud; }
#define ACTIVE_I2C_BAUD diagnostic_baud
#else
#define ACTIVE_I2C_BAUD IMU_I2C_BAUD
#endif

static void init_bus(void) {
    unsigned actual_baud = i2c_init(IMU_I2C, ACTIVE_I2C_BAUD);
    gpio_set_function(LINK101_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(LINK101_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(LINK101_PIN_SDA);
    gpio_pull_up(LINK101_PIN_SCL);
    status_printf("[bus] ordinary i2c1 init: requested=%uHz actual=%uHz SDA=GP%u SCL=GP%u; no bus recovery\n",
                  (unsigned)ACTIVE_I2C_BAUD, actual_baud, LINK101_PIN_SDA, LINK101_PIN_SCL);
}

static void init_sensor(void) {
    health.init_attempts++;
#ifdef IMU_DIAGNOSTIC
    imu_diagnostic_trace_set(true);
#endif
    status_printf("[init] LSM6DSOX attempt #%lu: primary address 0x%02X; WHO_AM_I expected=0x6C\n",
                  (unsigned long)health.init_attempts, IMU_ADDR);
    status_printf("[init] driver sequence: identity -> BDU/auto-increment -> accel 208Hz +/-4g -> gyro 208Hz +/-500dps -> 20ms sample wait\n");
    sensor_ready = link101_lsm6dsox_init(&sensor, IMU_I2C, IMU_ADDR);
    if (!sensor_ready) {
        // SDO/SA0 picks the low address bit. If the board strap doesn't
        // match what robot.h assumes -- a revision difference, or a strap
        // this firmware guessed wrong -- the chip answers at the other
        // address instead of not answering at all. Try it before giving up.
        status_printf("[init] primary initialization FAILED; trying alternate 0x%02X\n", LINK101_LSM6DSOX_ADDR_ALT);
        sensor_ready = link101_lsm6dsox_init(&sensor, IMU_I2C, LINK101_LSM6DSOX_ADDR_ALT);
        if (sensor_ready) {
            status_printf("[imu  ] LSM6DSOX answered at 0x%02X, not the expected 0x%02X "
                          "-- SDO/SA0 strap differs from robot.h\n",
                          LINK101_LSM6DSOX_ADDR_ALT, IMU_ADDR);
        }
    }
    status_printf("[imu  ] LSM6DSOX: %s\n",
                  sensor_ready ? "online" : "no response at 0x6B or 0x6A");

    // Refresh identity information for ROS diagnostics even with no USB log.
    health.primary_ack = link101_lsm6dsox_probe(IMU_I2C, IMU_ADDR, &health.primary_id);
    health.alternate_ack = link101_lsm6dsox_probe(IMU_I2C, LINK101_LSM6DSOX_ADDR_ALT,
                                                &health.alternate_id);
    if (!sensor_ready) {
        status_printf("[imu  ] 0x%02X: ack=%u WHO_AM_I=0x%02X; "
                      "0x%02X: ack=%u WHO_AM_I=0x%02X (expected 0x%02X)\n",
                      IMU_ADDR, health.primary_ack, health.primary_id,
                      LINK101_LSM6DSOX_ADDR_ALT, health.alternate_ack,
                      health.alternate_id, LINK101_LSM6DSOX_WHO_AM_I);
    }

#ifdef IMU_DIAGNOSTIC
    imu_diagnostic_trace_set(false);
#endif
    next_retry_us = time_us_64() + 1000000u;
}

bool imu_begin(void) {
    health = (imu_health_t){0};
    init_bus();

    init_sensor();

#ifdef IMU_DIAGNOSTIC_NO_MAG
    mag_ready = false;
    status_printf("[init] MMC5983MA DISABLED: no probe, reset, configuration or sampling\n");
#else
#ifdef IMU_DIAGNOSTIC
    imu_diagnostic_trace_set(true);
#endif
    status_printf("[init] MMC5983MA: address 0x%02X; product ID reg=0x2F expected=0x30\n", MAG_ADDR);
    status_printf("[init] driver sequence: identity -> software reset -> 15ms wait -> automatic SET/RESET -> continuous 100Hz -> 20ms sample wait\n");
    mag_ready = link101_mmc5983_init(&magnetometer, IMU_I2C, MAG_ADDR);
    status_printf("[imu  ] MMC5983MA at 0x%02X: %s\n", MAG_ADDR,
                  mag_ready ? "online" : "no response");

#ifdef IMU_DIAGNOSTIC
    imu_diagnostic_trace_set(false);
#endif
#endif
    status_printf("[init] COMPLETE: LSM6DSOX=%s MMC5983MA=%s\n",
                  sensor_ready ? "online" : "offline", mag_ready ? "online" : "offline");
    return sensor_ready || mag_ready;
}

void imu_update(void) {
    if (!sensor_ready && time_us_64() >= next_retry_us) {
        init_sensor();
    }
}

void imu_get_health(imu_health_t *out) {
    *out = health;
    out->sensor_online = sensor_ready;
    out->mag_online = mag_ready;
}

bool imu_online(void) {
    return sensor_ready;
}

bool imu_mag_online(void) {
    return mag_ready;
}

bool imu_read(imu_sample_t *out) {
    if (!sensor_ready) {
        return false;
    }
    if (!link101_lsm6dsox_read(&sensor, out->accel, out->gyro)) {
        health.read_failures++;
        return false;
    }
    // Temperature is a separate register block and nobody minds if it is a
    // sample behind, so a failure here doesn't spoil the reading.
    if (!link101_lsm6dsox_read_temperature(&sensor, &out->temp_c)) {
        health.temperature_failures++;
        out->temp_c = 0.0f;
    }
    health.read_successes++;
    return true;
}

bool imu_read_magnetic_field(float tesla[3]) {
#ifdef IMU_DIAGNOSTIC_NO_MAG
    (void)tesla;
    return false;
#else
    return mag_ready && link101_mmc5983_read(&magnetometer, tesla);
#endif
}
