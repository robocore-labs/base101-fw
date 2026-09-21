/* Standalone IMU bring-up on USB CDC #0 (/dev/link101-debug).
 * Uses production sensor initialization; never initializes motors or ROS.
 */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "tusb.h"
#include "robot.h"
#include "pico/stdlib.h"
#include "imu.h"
#include "io.h"
#include "status.h"

// Report each I2C phase separately: a failed read is not necessarily a NACK.
static void raw_register(uint8_t addr, uint8_t reg) {
    uint8_t value = 0;
    int written = i2c_write_timeout_us(IMU_I2C, addr, &reg, 1, true, 1000);
    int read = written == 1 ? i2c_read_timeout_us(IMU_I2C, addr, &value, 1, false, 1000) : 0;
    status_printf("[reg] addr=0x%02X reg=0x%02X write=%d read=%d value=0x%02X valid=%u\n",
                  addr, reg, written, read, value, written == 1 && read == 1);
}

// Compare the former 1ms budget with the corrected driver's 5ms budget.
// Individual reads are transport tests, not a coherent sensor sample.
static bool sample_transfer(uint8_t addr, uint8_t reg, unsigned count, unsigned timeout) {
    uint8_t data[12] = {0};
    uint64_t start = time_us_64();
    int written = i2c_write_timeout_us(IMU_I2C, addr, &reg, 1, true, timeout);
    int read = written == 1 ? i2c_read_timeout_us(IMU_I2C, addr, data, count, false, timeout) : 0;
    status_printf("[sample-test] addr=0x%02X reg=0x%02X len=%u timeout=%uus write=%d read=%d attempted=%u elapsed=%lluus valid=%u\n",
                  addr, reg, count, timeout, written, read, written == 1,
                  (unsigned long long)(time_us_64()-start), written == 1 && read == (int)count);
    if (read > 0) {
        char bytes[37]; unsigned used = 0;
        for (unsigned i=0; i < count && i < (unsigned)read; i++)
            used += (unsigned)snprintf(bytes+used, sizeof(bytes)-used, "%02X ", data[i]);
        status_printf("[sample-bytes] %s\n", bytes);
    }
    return written == 1 && read == (int)count;
}

static void sample_comparison(void) {
    status_printf("[sample-test] BEGIN baud=%u; -1=generic abort, -2=timeout; no reset/config writes\n",
                  (unsigned)imu_diagnostic_get_baud());
    for (uint8_t addr=0x6A; addr<=0x6B; addr++) {
        uint8_t id=0, reg=0x0F;
        int w=i2c_write_timeout_us(IMU_I2C,addr,&reg,1,true,5000);
        int r=w==1 ? i2c_read_timeout_us(IMU_I2C,addr,&id,1,false,5000):0;
        status_printf("[sample-test] identity addr=0x%02X write=%d read=%d who=0x%02X\n",addr,w,r,id);
        if(w!=1 || r!=1 || id!=0x6C) continue;
        unsigned singles=0;
        for(uint8_t reg=0x22;reg<=0x2D;reg++) singles += sample_transfer(addr,reg,1,1000);
        status_printf("[sample-test] single-byte successes=%u/12\n",singles);
        sample_transfer(addr,0x22,2,1000);
        sample_transfer(addr,0x22,6,1000);
        sample_transfer(addr,0x28,6,1000);
        sample_transfer(addr,0x22,12,1000);
        sample_transfer(addr,0x22,12,5000);
        sample_transfer(addr,0x20,2,1000); // temperature
    }
    status_printf("[sample-test] END\n");
}

static void probes(void) {
    status_printf("[probe] baud=%u SDA=%u SCL=%u; WHO_AM_I expected=0x6C\n",
                  (unsigned)imu_diagnostic_get_baud(),
                  gpio_get(LINK101_PIN_SDA), gpio_get(LINK101_PIN_SCL));
    raw_register(0x6B, 0x0F);
    raw_register(0x6A, 0x0F);
}

static void scan(void) {
    unsigned found = 0, timeouts = 0;
    status_printf("[scan] i2c1 at %uHz, addresses 0x08..0x77, one-byte READ probes; 0x30 skipped (mag disabled)\n", (unsigned)imu_diagnostic_get_baud());
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (addr == MAG_ADDR) continue;
        uint8_t value;
        int result = i2c_read_timeout_us(IMU_I2C, addr, &value, 1, false, 1000);
        if (result == 1) {
            found++;
            status_printf("[scan] address=0x%02X responded value=0x%02X (not an identity read)\n", addr, value);
        } else if (result == PICO_ERROR_TIMEOUT) {
            timeouts++;
            status_printf("[scan] address=0x%02X TIMEOUT\n", addr);
        }
        io_poll();
    }
    status_printf("[scan] COMPLETE responding=%u timeouts=%u SDA=%u SCL=%u\n",
                  found, timeouts, gpio_get(LINK101_PIN_SDA), gpio_get(LINK101_PIN_SCL));
}

static void registers(void) {
    for (uint8_t addr = 0x6A; addr <= 0x6B; addr++) {
        raw_register(addr, 0x10); // CTRL1_XL expected 0x58
        raw_register(addr, 0x11); // CTRL2_G expected 0x54
        raw_register(addr, 0x12); // CTRL3_C expected 0x44
    }
}

static void terminal(void) {
    static char line[48];
    static unsigned used;
    static bool overflow;
    while (tud_cdc_n_available(0)) {
        char ch;
        if (tud_cdc_n_read(0, &ch, 1) != 1) break;
        if (ch == '\n' || ch == '\r') {
            line[used] = 0;
            if (overflow) status_printf("[cmd] line too long\n");
            else if (!strcmp(line, "probe")) probes();
            else if (!strcmp(line, "regs")) registers();
            else if (!strcmp(line, "samples")) sample_comparison();
            else if (!strcmp(line, "scan")) scan();
            else if (!strcmp(line, "rate 100000") || !strcmp(line, "rate 400000")) {
                imu_diagnostic_set_baud(!strcmp(line, "rate 100000") ? 100000 : 400000);
                status_printf("[cmd] rate=%uHz; retries preserve this rate\n", (unsigned)imu_diagnostic_get_baud());
                imu_begin(); probes();
            }
            else if (!strcmp(line, "retry")) { imu_begin(); probes(); }
            else if (!strcmp(line, "help"))
                status_printf("[cmd] probe | regs | scan | samples | retry (ordinary I2C + LSM6DSOX config); rate 100000 | rate 400000 (reinitialize at selected rate)\n");
            else if (used) status_printf("[cmd] unknown; help\n");
            used = 0; overflow = false;
        } else if (ch == 8 || ch == 127) { if (used) used--; }
        else if (used + 1 < sizeof(line)) line[used++] = ch;
        else overflow = true;
    }
}

int main(void) {
    io_begin();
    status_begin();
    status_printf("\n=== base101 IMU diagnostic v8 (USB CDC #0) ===\n");
    gpio_init(LINK101_PIN_BUTTON);
    gpio_set_dir(LINK101_PIN_BUTTON, GPIO_IN);
    gpio_pull_up(LINK101_PIN_BUTTON);
    uint64_t next_wait_us = 0, pressed_us = 0;
    bool released = false;
    while (true) {
        io_poll();
        uint64_t now = time_us_64();
        if (now >= next_wait_us) {
            next_wait_us = now + 1000000;
            status_printf("[wait] imu_diagnostic v8 %s %s: open serial, then press CONFIG GP%u (active LOW); no I2C initialized yet; button=%u\n",
                          __DATE__, __TIME__, LINK101_PIN_BUTTON, gpio_get(LINK101_PIN_BUTTON));
        }
        if (gpio_get(LINK101_PIN_BUTTON)) { released = true; pressed_us = 0; }
        else if (released) {
            if (!pressed_us) pressed_us = now;
            if (now - pressed_us >= 30000 && tud_mounted()) break;
        }
        sleep_ms(1);
    }
    status_printf("\n[button] CONFIG pressed, debounced 30ms. BEGIN full initialization\n");
    status_printf("[init] build=%s %s i2c=i2c1 baud=%u SDA=GP%u SCL=GP%u primary=0x%02X alternate=0x6A mag=DISABLED\n",
                  __DATE__, __TIME__, IMU_I2C_BAUD, LINK101_PIN_SDA, LINK101_PIN_SCL, IMU_ADDR);
    imu_begin();
    status_set_mode(STATUS_READY);

    uint64_t next_sample_us = 0;
    uint64_t next_status_us = 0;
    uint32_t compared_attempt = 0;
    while (true) {
        io_poll();
        terminal();
        imu_update();
        uint64_t now = time_us_64();
        // Repeat status for terminals opened after boot.
        if (now >= next_status_us) {
            next_status_us = now + 1000000;
            imu_health_t health;
            imu_get_health(&health);
            status_printf("[build] imu_diagnostic v8 %s %s\n", __DATE__, __TIME__);
            status_printf("[status] t=%llu ms LSM6DSOX=%s MMC5983MA=DISABLED\n",
                          (unsigned long long)(now / 1000),
                          imu_online() ? "online" : "offline");
            status_printf("[health] init_attempts=%lu reads_ok=%lu reads_failed=%lu temp_failed=%lu\n",
                          (unsigned long)health.init_attempts,
                          (unsigned long)health.read_successes, (unsigned long)health.read_failures,
                          (unsigned long)health.temperature_failures);
            probes();
        }
        if (now < next_sample_us) {
            sleep_ms(1);
            continue;
        }
        next_sample_us = now + 100000; // 10 Hz

        imu_sample_t sample;
        if (imu_read(&sample)) {
            float magnitude = sqrtf(sample.accel[0] * sample.accel[0] +
                                    sample.accel[1] * sample.accel[1] +
                                    sample.accel[2] * sample.accel[2]);
            status_printf("[imu] accel m/s^2: %+.4f %+.4f %+.4f |a|=%.4f; "
                          "gyro rad/s: %+.4f %+.4f %+.4f; temp C: %.2f\n",
                          sample.accel[0], sample.accel[1], sample.accel[2],
                          magnitude, sample.gyro[0], sample.gyro[1],
                          sample.gyro[2], sample.temp_c);
        } else {
            status_printf("[imu] %s\n", imu_online() ? "READ FAILED" : "offline");
            imu_health_t health;
            imu_get_health(&health);
            if (health.sensor_online && compared_attempt != health.init_attempts) {
                compared_attempt = health.init_attempts;
                sample_comparison();
            }
        }
    }
}
