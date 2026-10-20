#include "usb_console_test_support.h"
#include "unity_test_runner.h"

void app_main(void) {
  if (hwtest_usb_console_ready()) unity_run_menu();
}
