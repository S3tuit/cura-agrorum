# ESP32-C6 firmware

The node acquires soil/environment readings, persists them, exchanges LoRa v2
frames with the receiver, and enters deep sleep.

- [Architecture](ARCHITECTURE.md): wake cycle, ownership, design reasons and limits.
- [Hardware](HARDWARE.md): nominal sensor/radio schematic and pin assignments.
- [Deployment](deploy/README.md): identity, configuration, build and flash workflow.
- [Tests](tests/README.md): host, bare-C6, sensor and joint RF entry points.
- [Protocol](../protocol/protocol-v2-lora/README.md): wire format and provisioning.

`main/` composes production components. `components/` owns implementation and
concise API documentation; generated protocol sources come from the protocol
schema/tools. `maintenance/` contains separate utility images with local usage
instructions. `tests/` holds host/target software checks and shared support;
`hardware/` holds complete sensor/radio test projects and their physical procedures.

For a local build, activate ESP-IDF and use the existing configured identity:

```sh
source ~/esp/esp-idf/export.sh
idf.py -C firmware build
```

A fresh checkout needs the identity/configuration setup in the deployment guide.
Generated credentials, sdkconfig, build output and managed dependencies are ignored.
Driver versions and licenses are owned by their component manifests/vendor guides;
do not patch fetched managed sources in place.
