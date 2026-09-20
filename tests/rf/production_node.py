"""Production artifact and read-only node capture guards. No flash/erase API."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import threading

from evidence import REPO, digest, write_json
from inputs import check_flash, firmware_sources
from node_capture import decode_image

class NodeUART:
    """One explicitly started autonomous image, passive UART thereafter."""
    def __init__(self, port, root):
        import serial
        self.port = serial.Serial(port=None, baudrate=115200, timeout=0.2, exclusive=True)
        self.port.dtr = False
        # On the reviewed UART reset circuit, RTS asserted with DTR released
        # holds EN low. Opening with both released can start the application
        # before the peer is armed; start() must be the first release.
        self.port.rts = True
        self.port.port = port
        self.port.open()
        self.raw = (Path(root) / "c6-uart.bin").open("xb")
        self.stop = threading.Event()
        self.failure = None
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        try:
            while not self.stop.is_set():
                data = self.port.read(4096)
                if data:
                    self.raw.write(data)
                    self.raw.flush()
        except BaseException as error:
            self.failure = error

    def start(self):
        from esptool.targets.esp32c6 import ESP32C6ROM
        ESP32C6ROM(port=self.port).hard_reset()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        self.port.close()
        self.raw.close()
        if self.thread.is_alive() or self.failure is not None:
            raise RuntimeError("UART capture incomplete: " + str(self.failure))


APP = REPO / "firmware"
STORAGE_OFFSET = 0x110000
STORAGE_SIZE = 0x2e0000


def build_identity(build):
    build = Path(build).resolve()
    project = json.loads((build / "project_description.json").read_text())
    if Path(project["project_path"]).resolve() != APP or project["project_name"] != "cura_agrorum_firmware":
        raise ValueError("not the production firmware build")
    files = check_flash(build, "cura_agrorum_firmware")
    config = json.loads((build / "config/sdkconfig.json").read_text())
    expected = {"LITTLEFS_READ_SIZE": 128, "LITTLEFS_WRITE_SIZE": 128,
                "LITTLEFS_CACHE_SIZE": 512, "LITTLEFS_LOOKAHEAD_SIZE": 128,
                "LITTLEFS_BLOCK_CYCLES": 512}
    if any(config.get(k) != v for k, v in expected.items()):
        raise ValueError("unreviewed LittleFS geometry")
    # Only a public ID is emitted. The key/header is never copied into capture.
    identity = (APP / "main/protocol_v2_lora_identity.h").read_text()
    sys.path.insert(0, str(REPO / "protocol/protocol-v2-lora/tools"))
    from provisioning_common import parse_node_identity_id
    node_id = parse_node_identity_id(identity)
    return dict(schema=1, node_id=node_id.hex(), files=files,
                sleep_seconds=config.get("NODE_DEEP_SLEEP_SECONDS", 900),
                sleep_observation=config.get("NODE_RF_SLEEP_OBSERVATION", False),
                dependencies=firmware_sources(build, APP))


def verify_build(build):
    sealed = json.loads((Path(build) / "production-rf-build.json").read_text())
    if sealed != build_identity(build):
        raise ValueError("production build/source changed; rebuild and reseal")
    return sealed


def esptool(fixture, root, label, arguments, *, timeout=60):
    command = [sys.executable, "-m", "esptool", "--chip", "esp32c6", "--port", fixture["c6_uart"],
               "--baud", "460800", "--after", "no-reset", *arguments]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as error:
        write_json(Path(root) / (label + ".json"), dict(command=command, exit=None,
                   error="timeout; node state must be re-established"))
        raise
    write_json(Path(root) / (label + ".json"), dict(command=command, exit=result.returncode,
               stdout=result.stdout, stderr=result.stderr))
    if result.returncode:
        raise RuntimeError(label + " failed; do not restart node")
    return result


def identify(fixture, root, label):
    result = esptool(fixture, root, label, ["read-mac"])
    expected = ":".join(fixture["c6_dut"][i:i + 2] for i in range(0, 12, 2))
    if not re.search(r"^(?:BASE )?MAC:\s+" + re.escape(expected) + r"\s*$", result.stdout, re.M):
        raise ValueError("wrong C6 UART identity; node left stopped")


def verify_installed(fixture, build, root):
    seal = verify_build(build)
    identify(fixture, root, "installed-mac")
    flash = json.loads((Path(build) / "flasher_args.json").read_text())["flash_files"]
    pairs = [part for offset, name in flash.items() for part in (offset, str(Path(build) / name))]
    esptool(fixture, root, "installed-image", ["verify-flash", *pairs], timeout=180)
    return seal


def capture_storage(fixture, root, name, reader, seal, run):
    image = Path(root) / (name + ".bin")
    if image.exists():
        raise ValueError("capture destination already exists")
    identify(fixture, root, name + "-mac")
    esptool(fixture, root, name + "-read", ["read-flash", hex(STORAGE_OFFSET), hex(STORAGE_SIZE), str(image)], timeout=180)
    if image.stat().st_size != STORAGE_SIZE:
        raise ValueError("incomplete storage capture")
    binding = dict(run=run, c6_dut=fixture["c6_dut"], node_id=seal["node_id"],
                   image_sha256=digest(image), offset=STORAGE_OFFSET, size=STORAGE_SIZE,
                   application_sha256=seal["files"]["cura_agrorum_firmware.bin"],
                   partition_sha256=seal["files"]["partition_table/partition-table.bin"])
    write_json(Path(root) / (name + "-binding.json"), binding)
    result = decode_image(image, reader)
    write_json(Path(root) / (name + "-records.json"), result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seal-build", type=Path, required=True)
    args = parser.parse_args()
    write_json(args.seal_build / "production-rf-build.json", build_identity(args.seal_build))


if __name__ == "__main__":
    main()
