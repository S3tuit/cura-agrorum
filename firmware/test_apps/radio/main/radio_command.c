#include "radio_command.h"
#include <string.h>

rf_line_status_t rf_line_feed(rf_line_t *line, int byte, uint64_t now_us) {
  if (line->status != RF_LINE_PENDING) return line->status;
  if (line->started && now_us - line->first_byte_us > 2000000) {
    return line->status = RF_LINE_EXPIRED;
  }
  if (byte < 0) return line->status;
  if (!line->started) {
    line->started = true;
    line->first_byte_us = now_us;
  }
  if (byte == '\n') {
    if (line->length && line->text[line->length - 1] == '\r') --line->length;
    line->text[line->length] = '\0';
    return line->status = RF_LINE_COMPLETE;
  }
  if ((byte < 0x20 && byte != '\r') || byte > 0x7e ||
      (line->length && line->text[line->length - 1] == '\r')) {
    return line->status = RF_LINE_INVALID;
  }
  if (line->length == sizeof(line->text) - 1) return line->status = RF_LINE_OVERSIZED;
  line->text[line->length++] = (char)byte;
  return line->status;
}

bool rf_command_parse(const char *text, rf_command_t *command) {
  rf_command_t parsed = {0};
  if (strncmp(text, "RUN ", 4) != 0) return false;
  const char *p = text + 4;
  for (unsigned i = 0; i < 32; ++i) {
    if (!((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f'))) return false;
    parsed.run[i] = *p++;
  }
  if (*p++ != ' ') return false;
  size_t length = 0;
  while (*p && *p != ' ') {
    if (length == sizeof(parsed.selection) - 1 ||
        !((*p >= 'A' && *p <= 'Z') || (*p >= 'a' && *p <= 'z') ||
          (*p >= '0' && *p <= '9') || *p == '-' || *p == '.')) return false;
    parsed.selection[length++] = *p++;
  }
  if (!length || *p++ != ' ' || *p < '0' || *p > '9') return false;
  if (*p == '0' && p[1] != ' ') return false;
  while (*p >= '0' && *p <= '9') {
    unsigned digit = (unsigned)(*p++ - '0');
    if (parsed.boot > (UINT32_MAX - digit) / 10) return false;
    parsed.boot = parsed.boot * 10 + digit;
  }
  if (*p++ != ' ' || (*p != '0' && *p != '1') || p[1] != '\0') return false;
  parsed.phase = (unsigned)(*p - '0');
  *command = parsed;
  return true;
}
