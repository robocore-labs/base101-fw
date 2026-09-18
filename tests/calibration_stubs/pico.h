#ifndef TEST_PICO_H
#define TEST_PICO_H
#include <stddef.h>
#include "pico/stdlib.h"
// Host regression checks can compile the SDK's actual lightweight formatter.
#define WRAPPER_FUNC(name) __wrap_##name
#define __printflike(format_index, first_arg) __attribute__((format(printf, format_index, first_arg)))
#endif
