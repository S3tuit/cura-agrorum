#include "node_persistence.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "node_common.h"
#include "node_persistence_backend.h"
#include "node_persistence_record.h"
#include "protocol_v2_lora_schema_generated.h"

#define PENDING_LOG_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.log"
#define PENDING_COMPACT_PATH NODE_PERSISTENCE_MOUNT_PATH "/pending.compact"
#define QUARANTINE_LOG_PATH NODE_PERSISTENCE_MOUNT_PATH "/quarantine.log"
#define DIAGNOSTIC_LOG_PATH NODE_PERSISTENCE_MOUNT_PATH "/diagnostic.log"
#define DELIVERY_LOG_PATH NODE_PERSISTENCE_MOUNT_PATH "/delivery.log"
#define PERSISTENCE_RECORD_TYPE_OFFSET 5U

#ifndef NODE_PERSISTENCE_PENDING_LOG_LIMIT
#define NODE_PERSISTENCE_PENDING_LOG_LIMIT (UINT64_C(512) * UINT64_C(1024))
#endif
#ifndef NODE_PERSISTENCE_PENDING_COMPACT_RETAIN_LIMIT
#define NODE_PERSISTENCE_PENDING_COMPACT_RETAIN_LIMIT                          \
  (NODE_PERSISTENCE_PENDING_LOG_LIMIT / UINT64_C(2))
#endif
#ifndef NODE_PERSISTENCE_QUARANTINE_LOG_LIMIT
#define NODE_PERSISTENCE_QUARANTINE_LOG_LIMIT (UINT64_C(192) * UINT64_C(1024))
#endif
#ifndef NODE_PERSISTENCE_DIAGNOSTIC_LOG_LIMIT
#define NODE_PERSISTENCE_DIAGNOSTIC_LOG_LIMIT (UINT64_C(256) * UINT64_C(1024))
#endif
#ifndef NODE_PERSISTENCE_DELIVERY_LOG_LIMIT
#define NODE_PERSISTENCE_DELIVERY_LOG_LIMIT (UINT64_C(256) * UINT64_C(1024))
#endif

typedef enum {
  BACKEND_UNINITIALIZED = 0,
  BACKEND_READY,
  BACKEND_FAILED,
} backend_state_t;

typedef struct {
  bool open;
  node_persistence_file_handle_t handle;
} log_file_state_t;

typedef struct {
  backend_state_t nvs_state;
  err_curag_t nvs_init_error;
  diagn_context_t nvs_init_diag;
  bool nvs_handle_open;
  node_persistence_nvs_handle_t nvs_handle;

  backend_state_t littlefs_state;
  err_curag_t littlefs_init_error;
  diagn_context_t littlefs_init_diag;
  log_file_state_t logs[NODE_PERSISTENCE_LOG_COUNT];
} node_persistence_state_t;

typedef struct {
  bool found;
  uint64_t start;
  size_t length;
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
} tail_record_t;

static node_persistence_state_t s_state;

static const char *const LOG_PATHS[NODE_PERSISTENCE_LOG_COUNT] = {
    PENDING_LOG_PATH,
    QUARANTINE_LOG_PATH,
    DIAGNOSTIC_LOG_PATH,
    DELIVERY_LOG_PATH,
};

static const node_persistence_resource_t
    LOG_RESOURCES[NODE_PERSISTENCE_LOG_COUNT] = {
        NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
        NODE_PERSISTENCE_RESOURCE_QUARANTINE_LOG,
        NODE_PERSISTENCE_RESOURCE_DIAGNOSTIC_LOG,
        NODE_PERSISTENCE_RESOURCE_DELIVERY_LOG,
};

static const uint64_t LOG_LIMITS[NODE_PERSISTENCE_LOG_COUNT] = {
    NODE_PERSISTENCE_PENDING_LOG_LIMIT,
    NODE_PERSISTENCE_QUARANTINE_LOG_LIMIT,
    NODE_PERSISTENCE_DIAGNOSTIC_LOG_LIMIT,
    NODE_PERSISTENCE_DELIVERY_LOG_LIMIT,
};

static void set_diag(diagn_context_t *out_diag, curag_operation_t operation,
                     node_persistence_resource_t resource,
                     node_persistence_stage_t stage,
                     node_persistence_backend_status_kind_t status_kind,
                     int32_t backend_status) {
  if (out_diag == NULL) {
    return;
  }
  curag_diagnostic_context_clear(out_diag);
  out_diag->operation = operation;
  out_diag->context_length = CURAG_PERSISTENCE_CONTEXT_V1_SIZE;
  out_diag->context_schema = CURAG_PERSISTENCE_CONTEXT_V1;
  out_diag->context[0] = resource;
  out_diag->context[1] = stage;
  out_diag->context[2] = status_kind;
  node_persistence_store_le32(out_diag->context + 3U, (uint32_t)backend_status);
}

static void copy_diag(diagn_context_t *output, const diagn_context_t *source) {
  if (output != NULL) {
    *output = *source;
  }
}

static err_curag_t fail(err_curag_t error, diagn_context_t *out_diag,
                        curag_operation_t operation,
                        node_persistence_resource_t resource,
                        node_persistence_stage_t stage,
                        node_persistence_backend_status_kind_t status_kind,
                        int32_t backend_status) {
  set_diag(out_diag, operation, resource, stage, status_kind, backend_status);
  return error;
}

static bool valid_log_kind(node_persistence_log_kind_t log_kind) {
  return log_kind < NODE_PERSISTENCE_LOG_COUNT;
}

static err_curag_t ensure_nvs(node_persistence_resource_t resource,
                              diagn_context_t *out_diag) {
  if (s_state.nvs_state == BACKEND_READY) {
    return CURAG_OK;
  }
  if (s_state.nvs_state == BACKEND_FAILED) {
    copy_diag(out_diag, &s_state.nvs_init_diag);
    return s_state.nvs_init_error;
  }

  const node_persistence_backend_t *backend = node_persistence_backend();
  int32_t status = backend->nvs_init();
  if (status != 0) {
    s_state.nvs_state = BACKEND_FAILED;
    s_state.nvs_init_error = CURAG_ENVS_INIT;
    set_diag(&s_state.nvs_init_diag, CURAG_OP_INITIALIZE, resource,
             NODE_PERSISTENCE_STAGE_INITIALIZE,
             NODE_PERSISTENCE_BACKEND_ESP_ERR, status);
    copy_diag(out_diag, &s_state.nvs_init_diag);
    return s_state.nvs_init_error;
  }

  status = backend->nvs_open(&s_state.nvs_handle);
  if (status != 0) {
    s_state.nvs_state = BACKEND_FAILED;
    s_state.nvs_init_error = CURAG_ENVS_ACCESS;
    set_diag(&s_state.nvs_init_diag, CURAG_OP_INITIALIZE, resource,
             NODE_PERSISTENCE_STAGE_OPEN, NODE_PERSISTENCE_BACKEND_ESP_ERR,
             status);
    copy_diag(out_diag, &s_state.nvs_init_diag);
    return s_state.nvs_init_error;
  }

  s_state.nvs_handle_open = true;
  s_state.nvs_state = BACKEND_READY;
  return CURAG_OK;
}

static err_curag_t ensure_littlefs(diagn_context_t *out_diag) {
  if (s_state.littlefs_state == BACKEND_READY) {
    return CURAG_OK;
  }
  if (s_state.littlefs_state == BACKEND_FAILED) {
    copy_diag(out_diag, &s_state.littlefs_init_diag);
    return s_state.littlefs_init_error;
  }

  const int32_t status = node_persistence_backend()->littlefs_mount();
  if (status != 0) {
    s_state.littlefs_state = BACKEND_FAILED;
    s_state.littlefs_init_error = CURAG_ELITTLEFS_INIT;
    set_diag(&s_state.littlefs_init_diag, CURAG_OP_INITIALIZE,
             NODE_PERSISTENCE_RESOURCE_LITTLEFS,
             NODE_PERSISTENCE_STAGE_INITIALIZE,
             NODE_PERSISTENCE_BACKEND_ESP_ERR, status);
    copy_diag(out_diag, &s_state.littlefs_init_diag);
    return s_state.littlefs_init_error;
  }

  s_state.littlefs_state = BACKEND_READY;
  return CURAG_OK;
}

static err_curag_t path_stat(const char *path,
                             node_persistence_resource_t resource,
                             curag_operation_t operation, bool *out_exists,
                             uint64_t *out_size, diagn_context_t *out_diag) {
  const int32_t status =
      node_persistence_backend()->path_stat(path, out_exists, out_size);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, operation, resource,
                NODE_PERSISTENCE_STAGE_STAT, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return CURAG_OK;
}

static err_curag_t open_log(node_persistence_log_kind_t log_kind, bool create,
                            curag_operation_t operation,
                            diagn_context_t *out_diag) {
  if (!valid_log_kind(log_kind)) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_LITTLEFS, NODE_PERSISTENCE_STAGE_NONE,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  log_file_state_t *file = &s_state.logs[log_kind];
  if (file->open) {
    return CURAG_OK;
  }
  const int32_t status = node_persistence_backend()->file_open(
      LOG_PATHS[log_kind], create, false, &file->handle);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, operation, LOG_RESOURCES[log_kind],
                NODE_PERSISTENCE_STAGE_OPEN, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  file->open = true;
  return CURAG_OK;
}

static err_curag_t log_size(node_persistence_log_kind_t log_kind,
                            curag_operation_t operation, uint64_t *out_size,
                            diagn_context_t *out_diag) {
  const int32_t status = node_persistence_backend()->file_size(
      s_state.logs[log_kind].handle, out_size);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, operation, LOG_RESOURCES[log_kind],
                NODE_PERSISTENCE_STAGE_STAT, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return CURAG_OK;
}

static err_curag_t read_log_exact(node_persistence_log_kind_t log_kind,
                                  uint64_t offset, uint8_t *output,
                                  size_t length, curag_operation_t operation,
                                  diagn_context_t *out_diag) {
  const int32_t status = node_persistence_backend()->file_read_exact(
      s_state.logs[log_kind].handle, offset, output, length);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, operation, LOG_RESOURCES[log_kind],
                NODE_PERSISTENCE_STAGE_READ, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return CURAG_OK;
}

static err_curag_t recover_truncate(node_persistence_log_kind_t log_kind,
                                    uint64_t boundary,
                                    err_curag_t recovery_result,
                                    diagn_context_t *out_diag) {
  const node_persistence_backend_t *backend = node_persistence_backend();
  int32_t status =
      backend->file_truncate(s_state.logs[log_kind].handle, boundary);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_RECOVER, LOG_RESOURCES[log_kind],
                NODE_PERSISTENCE_STAGE_TRUNCATE, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  status = backend->file_sync(s_state.logs[log_kind].handle);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_RECOVER, LOG_RESOURCES[log_kind],
                NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return fail(recovery_result, out_diag, CURAG_OP_RECOVER,
              LOG_RESOURCES[log_kind], NODE_PERSISTENCE_STAGE_TRUNCATE,
              NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
}

static err_curag_t read_candidate(node_persistence_log_kind_t log_kind,
                                  uint64_t end, uint16_t total_length,
                                  uint8_t *record, diagn_context_t *out_diag) {
  if ((uint64_t)total_length > end) {
    return CURAG_ECORRUPT_RECORD;
  }
  return read_log_exact(log_kind, end - total_length, record, total_length,
                        CURAG_OP_RECOVER, out_diag);
}

static err_curag_t read_record_ending_at(node_persistence_log_kind_t log_kind,
                                         uint64_t end,
                                         tail_record_t *out_record,
                                         diagn_context_t *out_diag) {
  memset(out_record, 0, sizeof(*out_record));
  if (end < NODE_PERSISTENCE_RECORD_FOOTER_SIZE) {
    return CURAG_ECORRUPT_RECORD;
  }
  uint8_t footer[NODE_PERSISTENCE_RECORD_FOOTER_SIZE];
  err_curag_t result =
      read_log_exact(log_kind, end - NODE_PERSISTENCE_RECORD_FOOTER_SIZE,
                     footer, sizeof(footer), CURAG_OP_READ, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  const uint16_t length = node_persistence_load_le16(footer);
  if (length < NODE_PERSISTENCE_RECORD_OVERHEAD ||
      length > NODE_PERSISTENCE_RECORD_MAX_SIZE || (uint64_t)length > end) {
    return CURAG_ECORRUPT_RECORD;
  }
  result = read_candidate(log_kind, end, length, out_record->record, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  if (node_persistence_record_validate(node_persistence_backend(), log_kind,
                                       out_record->record, length) !=
      NODE_PERSISTENCE_RECORD_VALID) {
    return CURAG_ECORRUPT_RECORD;
  }
  out_record->found = true;
  out_record->start = end - length;
  out_record->length = length;
  return CURAG_OK;
}

static err_curag_t
pending_binding_matches_preceding_reading(const tail_record_t *binding,
                                          bool *out_matches,
                                          diagn_context_t *out_diag) {
  *out_matches = false;
  tail_record_t reading_record;
  err_curag_t result = read_record_ending_at(
      NODE_PERSISTENCE_LOG_PENDING, binding->start, &reading_record, out_diag);
  if (result != CURAG_OK) {
    return result;
  }

  uint32_t binding_sample_id = 0U;
  uint32_t message_id = 0U;
  uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
  uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t reading;
  if (!node_persistence_record_decode_backlog_binding(
          binding->record, binding->length, &binding_sample_id, &message_id,
          frame) ||
      !node_persistence_record_decode_reading(reading_record.record,
                                              reading_record.length, body) ||
      cura_lora_v2_decode_reading(&reading, body, sizeof(body)) !=
          CURA_LORA_V2_CODEC_OK) {
    return CURAG_OK;
  }
  *out_matches = binding_sample_id == reading.sample_id;
  return CURAG_OK;
}

static err_curag_t recover_tail(node_persistence_log_kind_t log_kind,
                                tail_record_t *out_tail,
                                diagn_context_t *out_diag) {
  memset(out_tail, 0, sizeof(*out_tail));
  bool exists = false;
  uint64_t path_size_value = 0U;
  err_curag_t result =
      path_stat(LOG_PATHS[log_kind], LOG_RESOURCES[log_kind], CURAG_OP_RECOVER,
                &exists, &path_size_value, out_diag);
  if (result != CURAG_OK || !exists) {
    return result;
  }

  result = open_log(log_kind, false, CURAG_OP_RECOVER, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  uint64_t file_size = 0U;
  result = log_size(log_kind, CURAG_OP_RECOVER, &file_size, out_diag);
  if (result != CURAG_OK || file_size == 0U) {
    return result;
  }

  const node_persistence_backend_t *backend = node_persistence_backend();
  if (file_size >= NODE_PERSISTENCE_RECORD_FOOTER_SIZE) {
    uint8_t footer[NODE_PERSISTENCE_RECORD_FOOTER_SIZE];
    result = read_log_exact(log_kind,
                            file_size - NODE_PERSISTENCE_RECORD_FOOTER_SIZE,
                            footer, sizeof(footer), CURAG_OP_RECOVER, out_diag);
    if (result != CURAG_OK) {
      return result;
    }
    const uint16_t total_length = node_persistence_load_le16(footer);
    if (total_length >= NODE_PERSISTENCE_RECORD_OVERHEAD &&
        total_length <= NODE_PERSISTENCE_RECORD_MAX_SIZE &&
        (uint64_t)total_length <= file_size) {
      result = read_candidate(log_kind, file_size, total_length,
                              out_tail->record, out_diag);
      if (result != CURAG_OK && result != CURAG_ECORRUPT_RECORD) {
        return result;
      }
      if (result == CURAG_OK && node_persistence_record_validate_structural(
                                    backend, out_tail->record, total_length) ==
                                    NODE_PERSISTENCE_RECORD_VALID) {
        const node_persistence_record_result_t validation =
            node_persistence_record_validate(backend, log_kind,
                                             out_tail->record, total_length);
        if (validation == NODE_PERSISTENCE_RECORD_VALID) {
          out_tail->found = true;
          out_tail->start = file_size - total_length;
          out_tail->length = total_length;
          if (log_kind == NODE_PERSISTENCE_LOG_PENDING &&
              out_tail->record[PERSISTENCE_RECORD_TYPE_OFFSET] ==
                  NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING) {
            bool matches = false;
            result = pending_binding_matches_preceding_reading(
                out_tail, &matches, out_diag);
            if (result != CURAG_OK && result != CURAG_ECORRUPT_RECORD) {
              return result;
            }
            if (result != CURAG_OK || !matches) {
              return recover_truncate(log_kind, out_tail->start,
                                      CURAG_ECORRUPT_RECORD, out_diag);
            }
          }
          return CURAG_OK;
        }
        const err_curag_t semantic_error =
            validation == NODE_PERSISTENCE_RECORD_UNSUPPORTED
                ? CURAG_EUNSUPPORTED_RECORD
                : CURAG_ECORRUPT_RECORD;
        return recover_truncate(log_kind, file_size - total_length,
                                semantic_error, out_diag);
      }
    }
  }

  const uint64_t minimum_end =
      file_size > NODE_PERSISTENCE_RECORD_MAX_SIZE
          ? file_size - NODE_PERSISTENCE_RECORD_MAX_SIZE
          : 0U;
  if (file_size > 0U) {
    uint64_t candidate_end = file_size - 1U;
    for (;;) {
      if (candidate_end >= NODE_PERSISTENCE_RECORD_FOOTER_SIZE) {
        uint8_t footer[NODE_PERSISTENCE_RECORD_FOOTER_SIZE];
        result = read_log_exact(
            log_kind, candidate_end - NODE_PERSISTENCE_RECORD_FOOTER_SIZE,
            footer, sizeof(footer), CURAG_OP_RECOVER, out_diag);
        if (result != CURAG_OK) {
          return result;
        }
        const uint16_t total_length = node_persistence_load_le16(footer);
        if (total_length >= NODE_PERSISTENCE_RECORD_OVERHEAD &&
            total_length <= NODE_PERSISTENCE_RECORD_MAX_SIZE &&
            (uint64_t)total_length <= candidate_end) {
          result = read_candidate(log_kind, candidate_end, total_length,
                                  out_tail->record, out_diag);
          if (result != CURAG_OK && result != CURAG_ECORRUPT_RECORD) {
            return result;
          }
          if (result == CURAG_OK &&
              node_persistence_record_validate_structural(
                  backend, out_tail->record, total_length) ==
                  NODE_PERSISTENCE_RECORD_VALID) {
            return recover_truncate(log_kind, candidate_end,
                                    CURAG_ECORRUPT_RECORD, out_diag);
          }
        }
      }
      if (candidate_end == minimum_end) {
        break;
      }
      --candidate_end;
    }
  }

  if (file_size <= NODE_PERSISTENCE_RECORD_MAX_SIZE) {
    return recover_truncate(log_kind, 0U, CURAG_ECORRUPT_RECORD, out_diag);
  }
  return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_RECOVER,
              LOG_RESOURCES[log_kind], NODE_PERSISTENCE_STAGE_TAIL_SCAN,
              NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
}

static err_curag_t validate_file_forward(node_persistence_log_kind_t log_kind,
                                         curag_operation_t operation,
                                         diagn_context_t *out_diag) {
  uint64_t file_size = 0U;
  err_curag_t result = log_size(log_kind, operation, &file_size, out_diag);
  if (result != CURAG_OK) {
    return result;
  }

  const node_persistence_backend_t *backend = node_persistence_backend();
  uint64_t offset = 0U;
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  bool previous_pending_is_unbound = false;
  uint32_t previous_pending_sample_id = 0U;
  while (offset < file_size) {
    if (file_size - offset < NODE_PERSISTENCE_RECORD_HEADER_SIZE) {
      return fail(CURAG_ECORRUPT_RECORD, out_diag, operation,
                  LOG_RESOURCES[log_kind], NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    result = read_log_exact(log_kind, offset, record,
                            NODE_PERSISTENCE_RECORD_HEADER_SIZE, operation,
                            out_diag);
    if (result != CURAG_OK) {
      return result;
    }
    const size_t payload_length = node_persistence_load_le16(record + 6U);
    if (node_persistence_load_le32(record) != NODE_PERSISTENCE_RECORD_MAGIC ||
        payload_length > NODE_PERSISTENCE_RECORD_MAX_PAYLOAD_SIZE) {
      return fail(CURAG_ECORRUPT_RECORD, out_diag, operation,
                  LOG_RESOURCES[log_kind], NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    const size_t total_length =
        payload_length + NODE_PERSISTENCE_RECORD_OVERHEAD;
    if ((uint64_t)total_length > file_size - offset) {
      return fail(CURAG_ECORRUPT_RECORD, out_diag, operation,
                  LOG_RESOURCES[log_kind], NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    result = read_log_exact(log_kind, offset, record, total_length, operation,
                            out_diag);
    if (result != CURAG_OK) {
      return result;
    }
    const node_persistence_record_result_t validation =
        node_persistence_record_validate(backend, log_kind, record,
                                         total_length);
    if (validation != NODE_PERSISTENCE_RECORD_VALID) {
      const err_curag_t error =
          validation == NODE_PERSISTENCE_RECORD_UNSUPPORTED
              ? CURAG_EUNSUPPORTED_RECORD
              : CURAG_ECORRUPT_RECORD;
      return fail(error, out_diag, operation, LOG_RESOURCES[log_kind],
                  NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    if (log_kind == NODE_PERSISTENCE_LOG_PENDING) {
      const uint8_t record_type = record[PERSISTENCE_RECORD_TYPE_OFFSET];
      if (record_type == NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING) {
        uint8_t body[CURA_LORA_V2_READING_BODY_SIZE];
        cura_lora_v2_reading_t reading;
        if (!node_persistence_record_decode_reading(record, total_length,
                                                    body) ||
            cura_lora_v2_decode_reading(&reading, body, sizeof(body)) !=
                CURA_LORA_V2_CODEC_OK) {
          return fail(CURAG_ECORRUPT_RECORD, out_diag, operation,
                      NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                      NODE_PERSISTENCE_STAGE_DECODE,
                      NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
        }
        previous_pending_is_unbound = true;
        previous_pending_sample_id = reading.sample_id;
      } else {
        uint32_t sample_id = 0U;
        uint32_t message_id = 0U;
        uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE];
        if (!previous_pending_is_unbound ||
            !node_persistence_record_decode_backlog_binding(
                record, total_length, &sample_id, &message_id, frame) ||
            sample_id != previous_pending_sample_id) {
          return fail(CURAG_ECORRUPT_RECORD, out_diag, operation,
                      NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                      NODE_PERSISTENCE_STAGE_DECODE,
                      NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
        }
        previous_pending_is_unbound = false;
      }
    }
    offset += total_length;
  }
  return CURAG_OK;
}

static err_curag_t cleanup_stale_pending_compact(diagn_context_t *out_diag) {
  bool compact_exists = false;
  uint64_t compact_size = 0U;
  err_curag_t result =
      path_stat(PENDING_COMPACT_PATH, NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
                CURAG_OP_RECOVER, &compact_exists, &compact_size, out_diag);
  if (result != CURAG_OK || !compact_exists) {
    return result;
  }

  bool pending_exists = false;
  uint64_t pending_size = 0U;
  result =
      path_stat(PENDING_LOG_PATH, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                CURAG_OP_RECOVER, &pending_exists, &pending_size, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  if (!pending_exists) {
    return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_RECOVER,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }

  tail_record_t tail;
  result = recover_tail(NODE_PERSISTENCE_LOG_PENDING, &tail, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  result = validate_file_forward(NODE_PERSISTENCE_LOG_PENDING, CURAG_OP_RECOVER,
                                 out_diag);
  if (result != CURAG_OK) {
    return result;
  }

  const int32_t status =
      node_persistence_backend()->path_remove(PENDING_COMPACT_PATH);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_RECOVER,
                NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
                NODE_PERSISTENCE_STAGE_REMOVE, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return CURAG_OK;
}

static err_curag_t close_log_for_compaction(diagn_context_t *out_diag) {
  log_file_state_t *pending = &s_state.logs[NODE_PERSISTENCE_LOG_PENDING];
  if (!pending->open) {
    return CURAG_OK;
  }
  const int32_t status =
      node_persistence_backend()->file_close(pending->handle);
  pending->open = false;
  pending->handle = NODE_PERSISTENCE_INVALID_FILE_HANDLE;
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_COMPACT,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_CLOSE, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return CURAG_OK;
}

static err_curag_t compact_pending(diagn_context_t *out_diag) {
  err_curag_t result = validate_file_forward(NODE_PERSISTENCE_LOG_PENDING,
                                             CURAG_OP_COMPACT, out_diag);
  if (result != CURAG_OK) {
    return result;
  }

  uint64_t file_size = 0U;
  result = log_size(NODE_PERSISTENCE_LOG_PENDING, CURAG_OP_COMPACT, &file_size,
                    out_diag);
  if (result != CURAG_OK) {
    return result;
  }

  uint64_t retained_size = 0U;
  uint64_t retained_start = file_size;
  while (retained_start > 0U) {
    tail_record_t item_tail;
    result = read_record_ending_at(NODE_PERSISTENCE_LOG_PENDING, retained_start,
                                   &item_tail, out_diag);
    if (result != CURAG_OK) {
      return result;
    }
    uint64_t item_start = item_tail.start;
    uint64_t item_size = item_tail.length;
    if (item_tail.record[PERSISTENCE_RECORD_TYPE_OFFSET] ==
        NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING) {
      tail_record_t reading_record;
      result =
          read_record_ending_at(NODE_PERSISTENCE_LOG_PENDING, item_tail.start,
                                &reading_record, out_diag);
      if (result != CURAG_OK ||
          reading_record.record[PERSISTENCE_RECORD_TYPE_OFFSET] !=
              NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING) {
        return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_COMPACT,
                    NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                    NODE_PERSISTENCE_STAGE_DECODE,
                    NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
      }
      item_start = reading_record.start;
      item_size += reading_record.length;
    }
    if (retained_size > NODE_PERSISTENCE_PENDING_COMPACT_RETAIN_LIMIT ||
        item_size >
            NODE_PERSISTENCE_PENDING_COMPACT_RETAIN_LIMIT - retained_size) {
      break;
    }
    retained_start = item_start;
    retained_size += item_size;
  }

  node_persistence_file_handle_t temporary =
      NODE_PERSISTENCE_INVALID_FILE_HANDLE;
  const node_persistence_backend_t *backend = node_persistence_backend();
  int32_t status =
      backend->file_open(PENDING_COMPACT_PATH, true, true, &temporary);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_COMPACT,
                NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
                NODE_PERSISTENCE_STAGE_OPEN, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }

  uint8_t buffer[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  uint64_t copied = 0U;
  while (copied < retained_size) {
    size_t chunk = sizeof(buffer);
    if ((uint64_t)chunk > retained_size - copied) {
      chunk = (size_t)(retained_size - copied);
    }
    status = backend->file_read_exact(
        s_state.logs[NODE_PERSISTENCE_LOG_PENDING].handle,
        retained_start + copied, buffer, chunk);
    if (status != 0) {
      (void)backend->file_close(temporary);
      return fail(CURAG_EIO, out_diag, CURAG_OP_COMPACT,
                  NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                  NODE_PERSISTENCE_STAGE_READ, NODE_PERSISTENCE_BACKEND_ERRNO,
                  status);
    }
    status = backend->file_write_exact(temporary, copied, buffer, chunk);
    if (status != 0) {
      (void)backend->file_close(temporary);
      return fail(CURAG_EIO, out_diag, CURAG_OP_COMPACT,
                  NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
                  NODE_PERSISTENCE_STAGE_WRITE, NODE_PERSISTENCE_BACKEND_ERRNO,
                  status);
    }
    copied += chunk;
  }

  status = backend->file_sync(temporary);
  if (status != 0) {
    (void)backend->file_close(temporary);
    return fail(CURAG_EIO, out_diag, CURAG_OP_COMPACT,
                NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
                NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  status = backend->file_close(temporary);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_COMPACT,
                NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
                NODE_PERSISTENCE_STAGE_CLOSE, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }

  result = close_log_for_compaction(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  status = backend->path_rename(PENDING_COMPACT_PATH, PENDING_LOG_PATH);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_COMPACT,
                NODE_PERSISTENCE_RESOURCE_PENDING_COMPACT,
                NODE_PERSISTENCE_STAGE_RENAME, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return CURAG_OK;
}

static err_curag_t append_record(node_persistence_log_kind_t log_kind,
                                 const uint8_t *record, size_t record_length,
                                 bool durable, diagn_context_t *out_diag) {
  err_curag_t result = ensure_littlefs(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  if (log_kind == NODE_PERSISTENCE_LOG_PENDING) {
    result = cleanup_stale_pending_compact(out_diag);
    if (result != CURAG_OK) {
      return result;
    }
  }

  tail_record_t tail;
  result = recover_tail(log_kind, &tail, out_diag);
  if (result != CURAG_OK) {
    return result;
  }

  bool exists = false;
  uint64_t size = 0U;
  result = path_stat(LOG_PATHS[log_kind], LOG_RESOURCES[log_kind],
                     CURAG_OP_READ, &exists, &size, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  if (s_state.logs[log_kind].open) {
    result = log_size(log_kind, CURAG_OP_READ, &size, out_diag);
    if (result != CURAG_OK) {
      return result;
    }
  }

  if ((uint64_t)record_length > UINT64_MAX - size) {
    return fail(CURAG_ELOG_FULL, out_diag, CURAG_OP_APPEND,
                LOG_RESOURCES[log_kind], NODE_PERSISTENCE_STAGE_QUOTA_CHECK,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  if (size + record_length > LOG_LIMITS[log_kind]) {
    if (log_kind != NODE_PERSISTENCE_LOG_PENDING) {
      return fail(CURAG_ELOG_FULL, out_diag, CURAG_OP_APPEND,
                  LOG_RESOURCES[log_kind], NODE_PERSISTENCE_STAGE_QUOTA_CHECK,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    if (!s_state.logs[log_kind].open) {
      result = open_log(log_kind, false, CURAG_OP_COMPACT, out_diag);
      if (result != CURAG_OK) {
        return result;
      }
    }
    result = compact_pending(out_diag);
    if (result != CURAG_OK) {
      return result;
    }
    size = 0U;
    result = path_stat(PENDING_LOG_PATH, NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                       CURAG_OP_READ, &exists, &size, out_diag);
    if (result != CURAG_OK) {
      return result;
    }
    if (size + record_length > NODE_PERSISTENCE_PENDING_LOG_LIMIT) {
      return fail(CURAG_ELOG_FULL, out_diag, CURAG_OP_COMPACT,
                  NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                  NODE_PERSISTENCE_STAGE_QUOTA_CHECK,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
  }

  result = open_log(log_kind, true, CURAG_OP_APPEND, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  result = log_size(log_kind, CURAG_OP_READ, &size, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  const int32_t write_status = node_persistence_backend()->file_write_exact(
      s_state.logs[log_kind].handle, size, record, record_length);
  if (write_status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_APPEND, LOG_RESOURCES[log_kind],
                NODE_PERSISTENCE_STAGE_WRITE, NODE_PERSISTENCE_BACKEND_ERRNO,
                write_status);
  }
  if (!durable) {
    return CURAG_OK;
  }
  const int32_t sync_status =
      node_persistence_backend()->file_sync(s_state.logs[log_kind].handle);
  if (sync_status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_SYNC, LOG_RESOURCES[log_kind],
                NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO,
                sync_status);
  }
  return CURAG_OK;
}

static err_curag_t encode_reading_record(
    uint8_t record_type, const cura_lora_v2_reading_t *reading,
    uint8_t output[NODE_PERSISTENCE_RECORD_MAX_SIZE], size_t *out_length,
    node_persistence_resource_t resource, diagn_context_t *out_diag) {
  if (reading == NULL ||
      cura_lora_v2_validate_reading(reading) != CURA_LORA_V2_CODEC_OK) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_VALIDATE, resource,
                NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR,
                0);
  }

  uint8_t payload[NODE_PERSISTENCE_READING_PAYLOAD_SIZE];
  if (cura_lora_v2_encode_reading(payload, CURA_LORA_V2_READING_BODY_SIZE,
                                  reading) != CURA_LORA_V2_CODEC_OK ||
      !node_persistence_record_encode(node_persistence_backend(), record_type,
                                      payload, sizeof(payload), output,
                                      out_length)) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_ENCODE, resource,
                NODE_PERSISTENCE_STAGE_ENCODE,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  return CURAG_OK;
}

static err_curag_t claim_counter(uint32_t *out_id,
                                 node_persistence_nvs_counter_t counter,
                                 node_persistence_resource_t resource,
                                 err_curag_t exhausted_error,
                                 diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (out_id == NULL) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_VALIDATE, resource,
                NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR,
                0);
  }

  err_curag_t result = ensure_nvs(resource, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  const node_persistence_backend_t *backend = node_persistence_backend();
  uint32_t next_id = 0U;
  bool found = false;
  int32_t status =
      backend->nvs_get_counter(s_state.nvs_handle, counter, &next_id, &found);
  if (status != 0) {
    return fail(CURAG_ENVS_ACCESS, out_diag, CURAG_OP_READ, resource,
                NODE_PERSISTENCE_STAGE_GET, NODE_PERSISTENCE_BACKEND_ESP_ERR,
                status);
  }
  if (!found) {
    next_id = 0U;
  }
  if (next_id == UINT32_MAX) {
    return fail(exhausted_error, out_diag, CURAG_OP_VALIDATE, resource,
                NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR,
                0);
  }

  status = backend->nvs_set_counter(s_state.nvs_handle, counter, next_id + 1U);
  if (status != 0) {
    return fail(CURAG_ENVS_ACCESS, out_diag, CURAG_OP_WRITE, resource,
                NODE_PERSISTENCE_STAGE_SET, NODE_PERSISTENCE_BACKEND_ESP_ERR,
                status);
  }
  status = backend->nvs_commit(s_state.nvs_handle);
  if (status != 0) {
    return fail(CURAG_ENVS_ACCESS, out_diag, CURAG_OP_SYNC, resource,
                NODE_PERSISTENCE_STAGE_COMMIT, NODE_PERSISTENCE_BACKEND_ESP_ERR,
                status);
  }
  *out_id = next_id;
  return CURAG_OK;
}

err_curag_t node_persistence_claim_sample_id(uint32_t *out_sample_id,
                                             diagn_context_t *out_diag) {
  return claim_counter(out_sample_id, NODE_PERSISTENCE_NVS_COUNTER_SAMPLE,
                       NODE_PERSISTENCE_RESOURCE_NVS_SAMPLE_COUNTER,
                       CURAG_ESAMPLE_ID_EXHAUSTED, out_diag);
}

err_curag_t node_persistence_claim_message_id(uint32_t *out_message_id,
                                              diagn_context_t *out_diag) {
  return claim_counter(out_message_id, NODE_PERSISTENCE_NVS_COUNTER_MESSAGE,
                       NODE_PERSISTENCE_RESOURCE_NVS_MESSAGE_COUNTER,
                       CURAG_EMESSAGE_ID_EXHAUSTED, out_diag);
}

err_curag_t
node_persistence_append_pending_reading(const cura_lora_v2_reading_t *reading,
                                        diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  err_curag_t result = encode_reading_record(
      NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING, reading, record,
      &record_length, NODE_PERSISTENCE_RESOURCE_PENDING_LOG, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  return append_record(NODE_PERSISTENCE_LOG_PENDING, record, record_length,
                       true, out_diag);
}

err_curag_t node_persistence_bind_newest_backlog_frame(
    uint32_t expected_sample_id, uint32_t message_id,
    const uint8_t frame[CURA_LORA_V2_READING_FRAME_SIZE],
    diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (frame == NULL ||
      frame[CURA_LORA_V2_CLEAR_HEADER_CONTROL_OFFSET] != CURA_LORA_V2_CONTROL ||
      frame[CURA_LORA_V2_CLEAR_HEADER_DOMAIN_OFFSET] !=
          CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK ||
      node_persistence_load_le32(
          frame + CURA_LORA_V2_CLEAR_HEADER_MESSAGE_ID_OFFSET) != message_id) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR,
                0);
  }

  err_curag_t result = ensure_littlefs(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  result = cleanup_stale_pending_compact(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  tail_record_t tail;
  result = recover_tail(NODE_PERSISTENCE_LOG_PENDING, &tail, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  uint8_t reading_body[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t reading;
  if (!tail.found ||
      tail.record[PERSISTENCE_RECORD_TYPE_OFFSET] !=
          NODE_PERSISTENCE_RECORD_TYPE_PENDING_READING ||
      !node_persistence_record_decode_reading(tail.record, tail.length,
                                              reading_body) ||
      cura_lora_v2_decode_reading(&reading, reading_body,
                                  sizeof(reading_body)) !=
          CURA_LORA_V2_CODEC_OK ||
      reading.sample_id != expected_sample_id) {
    return fail(CURAG_ERECORD_MISMATCH, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }

  uint8_t payload[NODE_PERSISTENCE_BACKLOG_BINDING_PAYLOAD_SIZE];
  node_persistence_store_le32(payload, expected_sample_id);
  node_persistence_store_le32(payload + 4U, message_id);
  memcpy(payload + 8U, frame, CURA_LORA_V2_READING_FRAME_SIZE);
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  if (!node_persistence_record_encode(
          node_persistence_backend(),
          NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING, payload,
          sizeof(payload), record, &record_length)) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_ENCODE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_ENCODE,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  return append_record(NODE_PERSISTENCE_LOG_PENDING, record, record_length,
                       true, out_diag);
}

err_curag_t
node_persistence_peek_most_recent_pending(node_pending_reading_t *out_pending,
                                          bool *out_found,
                                          diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (out_pending == NULL || out_found == NULL) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR,
                0);
  }
  memset(out_pending, 0, sizeof(*out_pending));
  *out_found = false;

  err_curag_t result = ensure_littlefs(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  result = cleanup_stale_pending_compact(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  tail_record_t tail;
  result = recover_tail(NODE_PERSISTENCE_LOG_PENDING, &tail, out_diag);
  if (result != CURAG_OK || !tail.found) {
    return result;
  }

  tail_record_t reading_record = tail;
  if (tail.record[PERSISTENCE_RECORD_TYPE_OFFSET] ==
      NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING) {
    uint32_t binding_sample_id = 0U;
    if (!node_persistence_record_decode_backlog_binding(
            tail.record, tail.length, &binding_sample_id,
            &out_pending->message_id, out_pending->frame)) {
      return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_DECODE,
                  NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                  NODE_PERSISTENCE_STAGE_DECODE,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    result = read_record_ending_at(NODE_PERSISTENCE_LOG_PENDING, tail.start,
                                   &reading_record, out_diag);
    if (result != CURAG_OK) {
      return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_DECODE,
                  NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                  NODE_PERSISTENCE_STAGE_DECODE,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    out_pending->backlog_bound = true;
  }

  uint8_t reading_body[CURA_LORA_V2_READING_BODY_SIZE];
  if (!node_persistence_record_decode_reading(
          reading_record.record, reading_record.length, reading_body) ||
      cura_lora_v2_decode_reading(&out_pending->reading, reading_body,
                                  sizeof(reading_body)) !=
          CURA_LORA_V2_CODEC_OK) {
    return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_DECODE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_DECODE,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  if (out_pending->backlog_bound) {
    uint32_t binding_sample_id = 0U;
    uint32_t ignored_message_id = 0U;
    uint8_t ignored_frame[CURA_LORA_V2_READING_FRAME_SIZE];
    (void)node_persistence_record_decode_backlog_binding(
        tail.record, tail.length, &binding_sample_id, &ignored_message_id,
        ignored_frame);
    if (binding_sample_id != out_pending->reading.sample_id) {
      return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_DECODE,
                  NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                  NODE_PERSISTENCE_STAGE_DECODE,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
  }
  *out_found = true;
  return CURAG_OK;
}

err_curag_t node_persistence_remove_newest_reading(uint32_t expected_sample_id,
                                                   diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  err_curag_t result = ensure_littlefs(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  result = cleanup_stale_pending_compact(out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  tail_record_t tail;
  result = recover_tail(NODE_PERSISTENCE_LOG_PENDING, &tail, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  if (!tail.found) {
    return fail(CURAG_ERECORD_MISMATCH, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }

  tail_record_t reading_record = tail;
  uint64_t truncate_at = tail.start;
  if (tail.record[PERSISTENCE_RECORD_TYPE_OFFSET] ==
      NODE_PERSISTENCE_RECORD_TYPE_PENDING_BACKLOG_BINDING) {
    result = read_record_ending_at(NODE_PERSISTENCE_LOG_PENDING, tail.start,
                                   &reading_record, out_diag);
    if (result != CURAG_OK) {
      return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_DECODE,
                  NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                  NODE_PERSISTENCE_STAGE_DECODE,
                  NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
    }
    truncate_at = reading_record.start;
  }

  uint8_t reading_body[CURA_LORA_V2_READING_BODY_SIZE];
  cura_lora_v2_reading_t reading;
  if (!node_persistence_record_decode_reading(
          reading_record.record, reading_record.length, reading_body) ||
      cura_lora_v2_decode_reading(&reading, reading_body,
                                  sizeof(reading_body)) !=
          CURA_LORA_V2_CODEC_OK) {
    return fail(CURAG_ECORRUPT_RECORD, out_diag, CURAG_OP_DECODE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_DECODE,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  if (reading.sample_id != expected_sample_id) {
    return fail(CURAG_ERECORD_MISMATCH, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_TAIL_SCAN,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }

  const node_persistence_backend_t *backend = node_persistence_backend();
  int32_t status = backend->file_truncate(
      s_state.logs[NODE_PERSISTENCE_LOG_PENDING].handle, truncate_at);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_REMOVE,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_TRUNCATE, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  status =
      backend->file_sync(s_state.logs[NODE_PERSISTENCE_LOG_PENDING].handle);
  if (status != 0) {
    return fail(CURAG_EIO, out_diag, CURAG_OP_SYNC,
                NODE_PERSISTENCE_RESOURCE_PENDING_LOG,
                NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO,
                status);
  }
  return CURAG_OK;
}

err_curag_t
node_persistence_quarantine_reading(const cura_lora_v2_reading_t *reading,
                                    diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  err_curag_t result = encode_reading_record(
      NODE_PERSISTENCE_RECORD_TYPE_QUARANTINED_READING, reading, record,
      &record_length, NODE_PERSISTENCE_RESOURCE_QUARANTINE_LOG, out_diag);
  if (result != CURAG_OK) {
    return result;
  }
  return append_record(NODE_PERSISTENCE_LOG_QUARANTINE, record, record_length,
                       true, out_diag);
}

static bool validate_diagnostic_event(const node_diagnostic_event_t *event) {
  if (event == NULL || event->error == CURAG_OK ||
      curag_error_domain(event->error) == CURAG_EDOM_NONE ||
      curag_error_code(event->error) == CURAG_ECODE_NONE ||
      (event->flags & NODE_DIAGNOSTIC_RESERVED_FLAGS_MASK) != 0U ||
      ((event->flags & NODE_DIAGNOSTIC_APPLICATION_OFFSET_VALID) == 0U &&
       event->application_offset_ms != 0U) ||
      ((event->flags & NODE_DIAGNOSTIC_CYCLE_SAMPLE_ID_VALID) == 0U &&
       event->cycle_sample_id != 0U) ||
      ((event->flags & NODE_DIAGNOSTIC_MESSAGE_ID_VALID) == 0U &&
       event->message_id != 0U)) {
    return false;
  }
  if (event->context == NULL) {
    return true;
  }
  const diagn_context_t *context = event->context;
  if (context->operation > CURAG_OP_CLEANUP ||
      ((context->context_schema == CURAG_CONTEXT_SCHEMA_NONE) !=
       (context->context_length == 0U)) ||
      (context->context_length != 0U && context->operation == CURAG_OP_NONE)) {
    return false;
  }
  if (context->context_schema != UINT8_C(1)) {
    return true;
  }
  switch (curag_error_domain(event->error)) {
  case CURAG_EDOM_PERSISTENCE:
    return context->context_length == CURAG_PERSISTENCE_CONTEXT_V1_SIZE;
  case CURAG_EDOM_RADIO:
    return context->context_length == 14U;
  case CURAG_EDOM_SENSORS:
    return context->context_length == 48U;
  default:
    return true;
  }
}

err_curag_t
node_persistence_append_diagnostic_event(const node_diagnostic_event_t *event,
                                         diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (!validate_diagnostic_event(event)) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_DIAGNOSTIC_LOG,
                NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR,
                0);
  }

  const diagn_context_t *context = event->context;
  const uint8_t context_length = context == NULL ? 0U : context->context_length;
  const size_t payload_length =
      NODE_PERSISTENCE_DIAGNOSTIC_PREFIX_SIZE + context_length;
  uint8_t payload[NODE_PERSISTENCE_DIAGNOSTIC_MAX_PAYLOAD_SIZE] = {0};
  node_persistence_store_le16(payload, curag_error_domain(event->error));
  node_persistence_store_le16(payload + 2U, curag_error_code(event->error));
  node_persistence_store_le16(payload + 4U, event->flags);
  node_persistence_store_le32(payload + 6U, event->application_offset_ms);
  node_persistence_store_le32(payload + 10U, event->cycle_sample_id);
  node_persistence_store_le32(payload + 14U, event->message_id);
  if (context != NULL) {
    node_persistence_store_le16(payload + 18U, context->operation);
    payload[20U] = context->context_length;
    payload[21U] = context->context_schema;
    if (context_length != 0U) {
      memcpy(payload + NODE_PERSISTENCE_DIAGNOSTIC_PREFIX_SIZE,
             context->context, context_length);
    }
  }

  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  if (!node_persistence_record_encode(
          node_persistence_backend(),
          NODE_PERSISTENCE_RECORD_TYPE_DIAGNOSTIC_EVENT, payload,
          payload_length, record, &record_length) ||
      node_persistence_record_validate(
          node_persistence_backend(), NODE_PERSISTENCE_LOG_DIAGNOSTIC, record,
          record_length) != NODE_PERSISTENCE_RECORD_VALID) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_ENCODE,
                NODE_PERSISTENCE_RESOURCE_DIAGNOSTIC_LOG,
                NODE_PERSISTENCE_STAGE_ENCODE,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  return append_record(NODE_PERSISTENCE_LOG_DIAGNOSTIC, record, record_length,
                       false, out_diag);
}

static bool validate_delivery_event(const node_delivery_event_t *event) {
  if (event == NULL ||
      (event->domain != CURA_LORA_V2_DOMAIN_CURRENT_READING_UPLINK &&
       event->domain != CURA_LORA_V2_DOMAIN_BACKLOG_READING_UPLINK)) {
    return false;
  }
  if (event->type == NODE_DELIVERY_EVENT_STARTED) {
    return true;
  }
  if (event->type != NODE_DELIVERY_EVENT_FINISHED) {
    return false;
  }
  return event->detail.finished.final_result >= NODE_DELIVERY_RESULT_ACCEPTED &&
         event->detail.finished.final_result <=
             NODE_DELIVERY_RESULT_LOCAL_RADIO_ERROR;
}

err_curag_t
node_persistence_append_delivery_event(const node_delivery_event_t *event,
                                       diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  if (!validate_delivery_event(event)) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_VALIDATE,
                NODE_PERSISTENCE_RESOURCE_DELIVERY_LOG,
                NODE_PERSISTENCE_STAGE_NONE, NODE_PERSISTENCE_BACKEND_NO_ERROR,
                0);
  }

  uint8_t payload[NODE_PERSISTENCE_DELIVERY_STARTED_PAYLOAD_SIZE] = {0};
  node_persistence_store_le32(payload, event->cycle_sample_id);
  node_persistence_store_le32(payload + 4U, event->sample_id);
  node_persistence_store_le32(payload + 8U, event->message_id);
  payload[12U] = event->domain;
  uint8_t record_type = NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_STARTED;
  size_t payload_length = NODE_PERSISTENCE_DELIVERY_STARTED_PAYLOAD_SIZE;
  if (event->type == NODE_DELIVERY_EVENT_STARTED) {
    node_persistence_store_le32(payload + 13U,
                                event->detail.started.start_offset_ms);
  } else {
    record_type = NODE_PERSISTENCE_RECORD_TYPE_DELIVERY_FINISHED;
    payload_length = NODE_PERSISTENCE_DELIVERY_FINISHED_PAYLOAD_SIZE;
    payload[13U] = event->detail.finished.attempt_count;
    payload[14U] = event->detail.finished.final_result;
  }

  uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
  size_t record_length = 0U;
  if (!node_persistence_record_encode(node_persistence_backend(), record_type,
                                      payload, payload_length, record,
                                      &record_length) ||
      node_persistence_record_validate(
          node_persistence_backend(), NODE_PERSISTENCE_LOG_DELIVERY, record,
          record_length) != NODE_PERSISTENCE_RECORD_VALID) {
    return fail(CURAG_EINVALID_ARGUMENT, out_diag, CURAG_OP_ENCODE,
                NODE_PERSISTENCE_RESOURCE_DELIVERY_LOG,
                NODE_PERSISTENCE_STAGE_ENCODE,
                NODE_PERSISTENCE_BACKEND_NO_ERROR, 0);
  }
  return append_record(NODE_PERSISTENCE_LOG_DELIVERY, record, record_length,
                       true, out_diag);
}

err_curag_t node_persistence_sync_all(diagn_context_t *out_diag) {
  curag_diagnostic_context_clear(out_diag);
  const node_persistence_backend_t *backend = node_persistence_backend();
  err_curag_t first_error = CURAG_OK;
  diagn_context_t first_diag;
  curag_diagnostic_context_clear(&first_diag);

  for (size_t index = 0U; index < NODE_PERSISTENCE_LOG_COUNT; ++index) {
    log_file_state_t *file = &s_state.logs[index];
    if (!file->open) {
      continue;
    }
    int32_t status = backend->file_sync(file->handle);
    if (status != 0 && first_error == CURAG_OK) {
      first_error = CURAG_EIO;
      const node_persistence_resource_t resource =
          index == NODE_PERSISTENCE_LOG_DIAGNOSTIC
              ? NODE_PERSISTENCE_RESOURCE_DIAGNOSTIC_LOG
              : NODE_PERSISTENCE_RESOURCE_LITTLEFS;
      set_diag(&first_diag, CURAG_OP_SYNC, resource,
               NODE_PERSISTENCE_STAGE_SYNC, NODE_PERSISTENCE_BACKEND_ERRNO,
               status);
    }
    status = backend->file_close(file->handle);
    file->open = false;
    file->handle = NODE_PERSISTENCE_INVALID_FILE_HANDLE;
    if (status != 0 && first_error == CURAG_OK) {
      first_error = CURAG_EIO;
      set_diag(&first_diag, CURAG_OP_SYNC, LOG_RESOURCES[index],
               NODE_PERSISTENCE_STAGE_CLOSE, NODE_PERSISTENCE_BACKEND_ERRNO,
               status);
    }
  }

  if (s_state.nvs_handle_open) {
    backend->nvs_close(s_state.nvs_handle);
    s_state.nvs_handle_open = false;
  }
  if (first_error != CURAG_OK) {
    copy_diag(out_diag, &first_diag);
  }
  return first_error;
}

#ifdef NODE_PERSISTENCE_TESTING
void node_persistence_test_reset(void) {
  const node_persistence_backend_t *backend = node_persistence_backend();
  for (size_t index = 0U; index < NODE_PERSISTENCE_LOG_COUNT; ++index) {
    if (s_state.logs[index].open) {
      (void)backend->file_close(s_state.logs[index].handle);
    }
  }
  if (s_state.nvs_handle_open) {
    backend->nvs_close(s_state.nvs_handle);
  }
  memset(&s_state, 0, sizeof(s_state));
}
#endif
