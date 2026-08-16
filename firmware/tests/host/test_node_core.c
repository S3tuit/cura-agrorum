#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "node_core_test.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s SCENARIO\n", argv[0]);
    return 2;
  }
  const char *name = argv[1];
  const bool passed =
      node_core_test_initialization(name) || node_core_test_delivery(name) ||
      node_core_test_transitions(name) || node_core_test_finalization(name);
  if (!passed) {
    fprintf(stderr, "FAIL %s\n", name);
    return 1;
  }
  printf("PASS %s\n", name);
  return 0;
}
