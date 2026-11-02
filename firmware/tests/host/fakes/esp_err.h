#pragma once

#include <stdint.h>

typedef int32_t esp_err_t;

#define ESP_OK INT32_C(0)
#define ESP_FAIL INT32_C(-1)

const char *esp_err_to_name(esp_err_t code);

#define ESP_ERR_INVALID_STATE INT32_C(0x103)
#define ESP_ERR_TIMEOUT INT32_C(0x107)
#define ESP_ERR_INVALID_RESPONSE INT32_C(0x108)
