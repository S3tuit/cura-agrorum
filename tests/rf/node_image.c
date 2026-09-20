/* Offline evidence reader. Built with LFS_READONLY; never opens a device. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lfs.h"
#include "node_persistence_record.h"

static int image_read(const struct lfs_config *cfg, lfs_block_t block,
                      lfs_off_t offset, void *buffer, lfs_size_t size) {
  FILE *image = cfg->context;
  if (block >= cfg->block_count || offset > cfg->block_size ||
      size > cfg->block_size - offset ||
      fseek(image, (long)block * cfg->block_size + offset, SEEK_SET) != 0 ||
      fread(buffer, 1, size, image) != size) {
    return LFS_ERR_IO;
  }
  return 0;
}

static uint32_t crc32(const uint8_t *bytes, size_t length) {
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ ((crc & 1) ? UINT32_C(0xedb88320) : 0);
    }
  }
  return ~crc;
}

static int read_log(lfs_t *fs, const char *name,
                    node_persistence_log_kind_t kind) {
  const node_persistence_backend_t backend = {.crc32_iso_hdlc = crc32};
  lfs_file_t file;
  int error = lfs_file_open(fs, &file, name, LFS_O_RDONLY);
  printf("\"%s\":", name);
  if (error == LFS_ERR_NOENT) {
    printf("null");
    return 0;
  }
  if (error != 0) {
    fprintf(stderr, "%s: open failed (%d)\n", name, error);
    return 1;
  }
  printf("[");
  size_t offset = 0;
  while (true) {
    uint8_t record[NODE_PERSISTENCE_RECORD_MAX_SIZE];
    lfs_ssize_t count = lfs_file_read(fs, &file, record,
                                     NODE_PERSISTENCE_RECORD_HEADER_SIZE);
    if (count == 0) {
      break;
    }
    if (count != NODE_PERSISTENCE_RECORD_HEADER_SIZE) {
      error = 1;
      break;
    }
    size_t payload = node_persistence_load_le16(record + 6);
    size_t total = payload + NODE_PERSISTENCE_RECORD_OVERHEAD;
    if (total > sizeof(record) ||
        lfs_file_read(fs, &file, record + NODE_PERSISTENCE_RECORD_HEADER_SIZE,
                      total - NODE_PERSISTENCE_RECORD_HEADER_SIZE) !=
            (lfs_ssize_t)(total - NODE_PERSISTENCE_RECORD_HEADER_SIZE) ||
        node_persistence_record_validate(&backend, kind, record, total) !=
            NODE_PERSISTENCE_RECORD_VALID) {
      error = 1;
      break;
    }
    printf("%s{\"offset\":%zu,\"type\":%u,\"payload\":\"",
           offset ? "," : "", offset, record[5]);
    for (size_t i = 0; i < payload; ++i) {
      printf("%02x", record[NODE_PERSISTENCE_RECORD_HEADER_SIZE + i]);
    }
    printf("\"}");
    offset += total;
  }
  if (lfs_file_close(fs, &file) != 0) {
    error = 1;
  }
  printf("]");
  if (error) {
    fprintf(stderr, "%s: invalid/unsupported/unreadable record at %zu\n",
            name, offset);
  }
  return error != 0;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: node-image storage.bin\n");
    return 2;
  }
  FILE *image = fopen(argv[1], "rb");
  if (image == NULL) {
    perror("open image");
    return 1;
  }
  /* Reviewed pilot partition: 2944 KiB, 4 KiB erase blocks. Read/cache
   * geometry matches firmware/sdkconfig. Reject other layouts explicitly. */
  const long image_size = 2944L * 1024;
  if (fseek(image, 0, SEEK_END) || ftell(image) != image_size) {
    fprintf(stderr, "wrong pilot storage image size\n");
    fclose(image);
    return 1;
  }
  struct lfs_config cfg = {
      .context = image, .read = image_read,
      .read_size = 128, .prog_size = 128, .block_size = 4096,
      .block_count = (lfs_size_t)image_size / 4096,
      .cache_size = 512, .lookahead_size = 128, .block_cycles = 512,
      /* Match the ESP adapter: zero selects the LittleFS library default. */
      .name_max = 0,
  };
  lfs_t fs;
  int error = lfs_mount(&fs, &cfg);
  if (error != 0) {
    fprintf(stderr, "LittleFS mount failed (%d); image preserved\n", error);
    fclose(image);
    return 1;
  }
  const char *names[] = {"pending.log", "quarantine.log", "diagnostic.log",
                         "delivery.log"};
  printf("{");
  for (unsigned i = 0; i < 4; ++i) {
    if (i) printf(",");
    if (read_log(&fs, names[i], (node_persistence_log_kind_t)i)) {
      error = 1;
      break;
    }
  }
  printf("}\n");
  if (lfs_unmount(&fs) != 0) error = 1;
  if (fclose(image) != 0) error = 1;
  return error != 0;
}
