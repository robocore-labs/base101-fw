#ifndef TEST_TUSB_H
#define TEST_TUSB_H
#include <stdbool.h>
#include <stdint.h>
void tusb_init(void);
bool tud_mounted(void);
uint32_t tud_cdc_n_write_available(unsigned instance);
uint32_t tud_cdc_n_write(unsigned instance, const void *data, uint32_t length);
void tud_cdc_n_write_flush(unsigned instance);
uint32_t tud_cdc_n_available(unsigned instance);
uint32_t tud_cdc_n_read(unsigned instance, void *data, uint32_t length);
#endif
