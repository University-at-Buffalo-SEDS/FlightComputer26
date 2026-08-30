#ifndef PANIC_MATCH_H
#define PANIC_MATCH_H

#include <stdbool.h>
#include <stddef.h>

bool fc_panic_message_contains(const char *message, size_t message_size,
                               const char *needle);

#endif
