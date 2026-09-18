#include "imu_diagnostic_trace.h"
#include "pico/stdlib.h"
#include "status.h"
static bool enabled;
static uint8_t reg[128];
void imu_diagnostic_trace_set(bool value) { enabled = value; }
int imu_diagnostic_i2c_write(i2c_inst_t *i2c, uint8_t addr, const uint8_t *data,
                             size_t len, bool nostop, uint timeout) {
    if (len && addr < 128) reg[addr] = data[0];
    if (enabled) status_printf("[I2C] WRITE addr=0x%02X reg=0x%02X len=%u value=0x%02X repeated_start=%u timeout=%uus BEGIN\n",
        addr, len ? data[0] : 0, (unsigned)len, len > 1 ? data[1] : 0, nostop, timeout);
    uint64_t start = time_us_64();
    int result = i2c_write_timeout_us(i2c, addr, data, len, nostop, timeout);
    if (enabled || result != (int)len) status_printf("[I2C] WRITE addr=0x%02X reg=0x%02X result=%d expected=%u elapsed=%lluus %s\n", addr, len ? data[0] : 0, result,
        (unsigned)len, (unsigned long long)(time_us_64()-start), result == (int)len ? "OK" : "FAILED");
    return result;
}
int imu_diagnostic_i2c_read(i2c_inst_t *i2c, uint8_t addr, uint8_t *data,
                            size_t len, bool nostop, uint timeout) {
    if (enabled) status_printf("[I2C] READ addr=0x%02X reg=0x%02X len=%u timeout=%uus BEGIN\n",
        addr, addr < 128 ? reg[addr] : 0, (unsigned)len, timeout);
    uint64_t start = time_us_64();
    int result = i2c_read_timeout_us(i2c, addr, data, len, nostop, timeout);
    if (enabled || result != (int)len) status_printf("[I2C] READ addr=0x%02X reg=0x%02X result=%d expected=%u elapsed=%lluus first=0x%02X %s\n", addr, addr < 128 ? reg[addr] : 0, result,
        (unsigned)len, (unsigned long long)(time_us_64()-start), result > 0 ? data[0] : 0,
        result == (int)len ? "OK" : "FAILED");
    return result;
}
