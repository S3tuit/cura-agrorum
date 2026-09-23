/* Host-only image builder. Never linked into the read-only evidence reader. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lfs.h"

static unsigned char image[2944 * 1024];
static int read_block(const struct lfs_config *c, lfs_block_t b, lfs_off_t o,
                      void *p, lfs_size_t n) {
  memcpy(p, image + b * c->block_size + o, n);
  return 0;
}
static int write_block(const struct lfs_config *c, lfs_block_t b, lfs_off_t o,
                       const void *p, lfs_size_t n) {
  memcpy(image + b * c->block_size + o, p, n);
  return 0;
}
static int erase_block(const struct lfs_config *c, lfs_block_t b) {
  memset(image + b * c->block_size, 255, c->block_size);
  return 0;
}
static int sync_blocks(const struct lfs_config *c) { (void)c; return 0; }
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  memset(image, 255, sizeof(image));
  struct lfs_config c = {
      .read = read_block, .prog = write_block, .erase = erase_block,
      .sync = sync_blocks, .read_size = 128, .prog_size = 128,
      .block_size = 4096, .block_count = sizeof(image) / 4096,
      .cache_size = 512, .lookahead_size = 128, .block_cycles = 512,
      /* Match the ESP adapter: zero selects the LittleFS library default. */
      .name_max = 0,
  };
  lfs_t fs;
  if (lfs_format(&fs, &c) || lfs_mount(&fs, &c)) return 1;
  const char *names[] = {"pending.log", "quarantine.log", "diagnostic.log", "delivery.log"};
  for (unsigned i = 0; i < 4; ++i) {
    FILE *input = fopen(names[i], "rb");
    if (!input) continue;
    lfs_file_t file;
    if (lfs_file_open(&fs, &file, names[i], LFS_O_WRONLY | LFS_O_CREAT)) return 1;
    unsigned char buffer[512];
    size_t count;
    while ((count = fread(buffer, 1, sizeof(buffer), input))) {
      if (lfs_file_write(&fs, &file, buffer, count) != (lfs_ssize_t)count) return 1;
    }
    if (ferror(input) || fclose(input) || lfs_file_close(&fs, &file)) return 1;
  }
  if (lfs_unmount(&fs)) return 1;
  FILE *output = fopen(argv[1], "wb");
  if (!output) return 1;
  if (fwrite(image, 1, sizeof(image), output) != sizeof(image)) return 1;
  return fclose(output) != 0;
}
