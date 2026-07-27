#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "fault.h"
#include "wire.h"

/* Starts the fault context for one wake cycle.
 *
 * This does not modify a pending fault retained from an earlier wake cycle.
 * It always updates the context used by a new fault recorded later in this wake
 * cycle. Until fault_set_sample_id() is called, new faults use UINT32_MAX to
 * indicate that no sample id was available.
 */
void fault_init(uint32_t bootno);

/* Updates the current wake-cycle context after sample id allocation succeeds.
 *
 * This does not modify a fault that is already pending, but it updates the
 * context used if that fault is cleared and another fault occurs this wake.
 */
void fault_set_sample_id(uint32_t sample_id);

/* Retains the first fault observed until fault_clear() is called.
 *
 * This does not modify a pending fault retained from an earlier wake cycle.
 * posix_errno must and esp_err can be set to 0 when not applicable.
 */
void fault_record(fault_operation_t operation, esp_err_t esp_err,
                  int posix_errno);

/* Appends the pending fault to builder without clearing it.
 *
 * On success, appended reports whether a fault was added.
 */
esp_err_t fault_append_to_builder(wire_builder_t *builder, bool *appended);

/* Clears the pending RTC-retained fault. */
void fault_clear(void);
