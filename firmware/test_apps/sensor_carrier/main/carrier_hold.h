#pragma once
#include <stdbool.h>

/* UART-only app orchestration. Neither function accesses sensors or gates. */
enum { CARRIER_HOLD_AUTO, CARRIER_HOLD_GUIDED, CARRIER_HOLD_EXPLORATION };
int carrier_hold_select(void); /* -1 invalid/incomplete */
bool carrier_hold_wait(const char *name, int mode);
