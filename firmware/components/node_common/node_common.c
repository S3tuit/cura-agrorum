#include "node_common.h"

#include <string.h>

void curag_diagnostic_context_clear(diagn_context_t *context) {
  if (context != NULL) {
    memset(context, 0, sizeof(*context));
  }
}
