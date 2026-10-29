# DS3231 hardware checks

Use the Adafruit DS3231 and CR1220 with the nominal
[receiver schematic](../../HARDWARE.md). The component families keep their
physical preparation, code, restoration, and reusable findings together:

- [Boot, battery retention, and OSF](tests/boot_cycle/README.md): four cold boots
  and the optional production-adapter assertion.
- [I2C controller faults](tests/i2c_faults/README.md): removable fault branches,
  four automated cases, recovery limits, and the adopted operation bound.

[Deployment](../../deploy/README.md) owns OS setup;
[Chrony setup](../../deploy/CHRONY.md) owns normal time configuration and RTC
initialization. Runtime behavior is explained in
[RuntimeTime](../../cura_receiver/runtime_time.py), the
[RTC port](../../cura_receiver/ports/ds3231.py), and the
[Linux adapter](../../cura_receiver/platform/linux_ds3231.py).
Detailed experiment and deployment histories belong in the sibling logbook.
