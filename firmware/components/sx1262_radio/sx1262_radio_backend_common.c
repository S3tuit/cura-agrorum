#include "sx1262_radio_backend.h"

#include <string.h>

void sx1262_radio_backend_error_clear(sx1262_radio_backend_error_t *error) {
  if (error != NULL) {
    memset(error, 0, sizeof(*error));
  }
}
