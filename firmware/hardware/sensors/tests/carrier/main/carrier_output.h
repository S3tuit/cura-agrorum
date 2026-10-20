#pragma once

int carrier_output_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
int carrier_output_puts(const char *text);
