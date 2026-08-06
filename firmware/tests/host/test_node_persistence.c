#include <stddef.h>
#include <stdio.h>

#include "node_persistence_test.h"

static size_t run_group(const node_persistence_test_group_t *group) {
  size_t failures = 0U;
  for (size_t index = 0U; index < group->count; ++index) {
    if (group->cases[index].function()) {
      printf("PASS %s/%s\n", group->name, group->cases[index].name);
    } else {
      printf("FAIL %s/%s\n", group->name, group->cases[index].name);
      ++failures;
    }
  }
  return failures;
}

int main(void) {
  static const node_persistence_test_group_t *const groups[] = {
      &NODE_PERSISTENCE_NVS_TEST_GROUP,
      &NODE_PERSISTENCE_RECORD_TEST_GROUP,
      &NODE_PERSISTENCE_RECOVERY_TEST_GROUP,
      &NODE_PERSISTENCE_FAULT_TEST_GROUP,
      &NODE_PERSISTENCE_RETENTION_TEST_GROUP,
  };

  size_t failures = 0U;
  for (size_t index = 0U; index < sizeof(groups) / sizeof(groups[0]); ++index) {
    failures += run_group(groups[index]);
  }
  node_persistence_test_reset();
  if (failures != 0U) {
    fprintf(stderr, "%zu node_persistence test(s) failed\n", failures);
    return 1;
  }
  return 0;
}
