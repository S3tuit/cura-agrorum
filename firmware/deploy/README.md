# Fresh XIAO ESP32S3 deployment

This sequence provisions a fresh receiver group and node identity, builds for
the nominal [carrier wiring](../HARDWARE.md), and resets
the S3 before installation. Ordinary firmware updates retain the existing
identity, counters and storage; do not use this reset sequence for them.

Run the commands in one Bash session. You need Git, Python 3, ripgrep (`rg`),
an installed ESP-IDF, and exclusive access to the XIAO native USB Serial/JTAG port. Replace the
port and ESP-IDF path below with yours.

## 1. Create private staging directories

From the repository root:

```bash
umask 077
cura_repo=$(git rev-parse --show-toplevel)
cura_deploy=$(mktemp -d /tmp/cura-node-deploy.XXXXXX)
cura_stage="$cura_deploy/source"
cura_port=/dev/ttyACM0
mkdir "$cura_deploy/receiver" "$cura_deploy/node"
git -C "$cura_repo" worktree add --detach "$cura_stage" HEAD
```

The worktree contains committed source at `HEAD`, including these deployment
files, without local secrets, resolved `sdkconfig` or build output. Uncommitted
changes are not included. The private parent directory also protects generated
credentials and firmware images containing them.

## 2. Provision and install the node identity

```bash
python3 "$cura_stage/protocol/protocol-v2-lora/tools/init_receiver_group.py" \
  --recv-dir "$cura_deploy/receiver"
python3 "$cura_stage/protocol/protocol-v2-lora/tools/provision_node.py" \
  --recv-dir "$cura_deploy/receiver" --node-dir "$cura_deploy/node"
install -m 600 "$cura_deploy/node/protocol_v2_lora_identity.h" \
  "$cura_stage/firmware/main/protocol_v2_lora_identity.h"
```

The tools generate the node ID and key and register that ID in the group.
Keep `$cura_deploy/receiver/receiver-group.json` for the matching
[Pi deployment](../../receiver/deploy/README.md); securely retain both credential
files outside temporary storage before cleanup. Never commit or publish keys.
For an existing group or identity replacement, follow the
[provisioning rules](../../protocol/protocol-v2-lora/README.md#revocation-and-identity-replacement).

## 3. Assign probes and build production firmware

Edit `$cura_stage/firmware/deploy/sdkconfig.probes`: fill both empty strings
with the attached probes' distinct, nonzero ROM IDs, each exactly 16 hexadecimal
digits. The variable names assign physical probes to logical channels 0 and 1.
An empty field leaves that channel unprovisioned.

```bash
source ~/esp/esp-idf/export.sh
cd "$cura_stage/firmware"
test ! -e sdkconfig && \
  idf.py -D 'SDKCONFIG_DEFAULTS=sdkconfig.defaults;deploy/sdkconfig.pilot;deploy/sdkconfig.probes' build
rg '^CONFIG_CURA_DS18B20_[01]_ROM=' sdkconfig
```

Production selects ESP32S3 in CMake, 8 MB flash, one core and primary native
USB Serial/JTAG. GPIO44 belongs to the sensor gate; UART0 console is rejected.
The defaults supply the production layout,
`sdkconfig.pilot` selects carrier wiring, and `sdkconfig.probes` supplies your
channel identities. Check that the printed resolved values match your probes;
an existing `sdkconfig` overrides defaults, so stop if the freshness check fails.

## 4. Reset the S3 and initialize LittleFS

Stop production firmware before this destructive step. It discards all previous
node-local state. The complete rationale and storage requirements belong to the
[maintenance procedure](../maintenance/erase_storage/README.md).

```bash
cd "$cura_stage/firmware/maintenance/erase_storage"
idf.py set-target esp32s3
idf.py build
idf.py -p "$cura_port" erase-flash
```

Disconnect **all** node power, then reconnect. USB may enumerate with a
different ttyACM number; identify the same physical board and update `cura_port`
before continuing. This clears retained RTC state as
well as the flash state already erased. Now initialize the empty filesystem:

```bash
idf.py -p "$cura_port" flash monitor
```

Require `LittleFS format completed successfully; NVS was preserved`, then exit
the monitor with Ctrl+]. Production does not format an empty filesystem itself.

## 5. Flash production firmware

Flashing starts production.

```bash
cd "$cura_stage/firmware"
idf.py -p "$cura_port" flash monitor
```

This final flash preserves the initialized LittleFS partition. Keep
credential-bearing images private.
