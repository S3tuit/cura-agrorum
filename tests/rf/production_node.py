"""Production artifact and read-only node capture guards. No flash/erase API."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

from capture import REPO, digest, write_json
from inputs import check_flash, firmware_sources
from node_capture import decode_image
from finalization import Finalization
from sleep_observation import sleep_count
sys.path.insert(0, str(REPO / "firmware/tests/support"))
from usb_console import USBSerial, hard_reset_native_usb, usb_identity
from usb_console import fingerprint, list_ports

class NodeUART:
    """Admit the already stopped ROM without reset; start only after peer arm."""
    def __init__(self, port, root, *, reconnect_timeout=930, on_data=None, usb_binding=None):
        import serial
        from esptool.targets.esp32s3 import ESP32S3ROM
        if usb_binding is not None and usb_identity(port) != usb_binding:
            raise ValueError("node USB identity changed before opening capture")
        raw = serial.Serial(port=None, baudrate=115200, timeout=0.2, exclusive=True)
        raw.dtr = raw.rts = False
        raw.port = port
        try:
            raw.open()
            if usb_binding is not None and usb_identity(port) != usb_binding:
                raise ValueError("node USB identity changed before ROM admission")
            # verify_installed/capture_storage leave ROM active (--after no-reset).
            # Native USB control lines do not implement the old UART EN hold.
            rom = ESP32S3ROM(port=raw)
            rom.connect(mode="no-reset", attempts=1)
            # Clear forced download while the ROM owns the stream, before the
            # passive reader starts. The later reset then boots the application.
            rom.write_reg(rom.RTC_CNTL_OPTION1_REG, 0,
                          rom.RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK)
            self.port = USBSerial(raw, reconnect_timeout=reconnect_timeout)
            self.raw = (Path(root) / "node-uart.bin").open("xb")
        except BaseException:
            raw.close()
            raise
        self.stop = threading.Event()
        self.on_data = on_data
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
                    if self.on_data is not None:
                        self.on_data(data)
                else:
                    time.sleep(0.05)
        except BaseException as error:
            self.failure = error

    def start(self):
        hard_reset_native_usb(self.port)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError("UART reader did not stop; preserve capture handle")
        self.port.close()
        self.raw.close()
        self.port.check_health()
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
                phy_observation=config.get("NODE_RF_PHY_OBSERVATION", False),
                dependencies=firmware_sources(build, APP))


def verify_build(build):
    sealed = json.loads((Path(build) / "production-rf-build.json").read_text())
    if sealed != build_identity(build):
        raise ValueError("production build/source changed; rebuild and reseal")
    return sealed


def esptool(fixture, root, label, arguments, *, timeout=60, usb_binding=None):
    port = fixture["node_uart"]
    if usb_binding is not None:
        matches = [p.device for p in list_ports.comports() if fingerprint(p) == usb_binding]
        if len(matches) != 1:
            raise ValueError("bound node USB device absent or ambiguous; do not restart")
        port = matches[0]
    admitted = usb_identity(port)
    if usb_binding is not None and admitted != usb_binding:
        raise ValueError("bound node USB device changed before ROM access")
    command = [sys.executable, "-m", "esptool", "--chip", "esp32s3", "--port", port,
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


def identify(fixture, root, label, *, usb_binding=None):
    options = {} if usb_binding is None else dict(usb_binding=usb_binding)
    result = esptool(fixture, root, label, ["read-mac"], **options)
    expected = ":".join(fixture["node_dut"][i:i + 2] for i in range(0, 12, 2))
    if not re.search(r"^(?:BASE )?MAC:\s+" + re.escape(expected) + r"\s*$", result.stdout, re.M):
        raise ValueError("wrong S3 UART identity; node left stopped")


def verify_installed(fixture, build, root, *, usb_binding=None):
    seal = verify_build(build)
    options = {} if usb_binding is None else dict(usb_binding=usb_binding)
    identify(fixture, root, "installed-mac", **options)
    flash = json.loads((Path(build) / "flasher_args.json").read_text())["flash_files"]
    pairs = [part for offset, name in flash.items() for part in (offset, str(Path(build) / name))]
    esptool(fixture, root, "installed-image", ["verify-flash", *pairs], timeout=180, **options)
    return seal


def capture_storage(fixture, root, name, reader, seal, run, *, usb_binding=None):
    image = Path(root) / (name + ".bin")
    if image.exists():
        raise ValueError("capture destination already exists")
    options = {} if usb_binding is None else dict(usb_binding=usb_binding)
    identify(fixture, root, name + "-mac", **options)
    esptool(fixture, root, name + "-read", ["read-flash", hex(STORAGE_OFFSET), hex(STORAGE_SIZE), str(image)], timeout=180, **options)
    if image.stat().st_size != STORAGE_SIZE:
        raise ValueError("incomplete storage capture")
    binding = dict(run=run, node_dut=fixture["node_dut"], node_id=seal["node_id"],
                   image_sha256=digest(image), offset=STORAGE_OFFSET, size=STORAGE_SIZE,
                   application_sha256=seal["files"]["cura_agrorum_firmware.bin"],
                   partition_sha256=seal["files"]["partition_table/partition-table.bin"])
    write_json(Path(root) / (name + "-binding.json"), binding)
    result = decode_image(image, reader)
    write_json(Path(root) / (name + "-records.json"), result)
    return result


class ProductionSession:
    """Own observation, receipt/park, ROM handoff and snapshots for one episode."""
    def __init__(self, fixture, root, reader, seal, run, wakes, *, stop_timeout=60):
        if seal.get("sleep_seconds") != 10 or not seal.get("sleep_observation"):
            raise ValueError("RF session requires the observed accelerated image")
        self.fixture, self.root, self.reader, self.seal, self.run = fixture, Path(root), reader, seal, run
        self.wakes, self.stop_timeout = wakes, stop_timeout
        self.binding = usb_identity(fixture["node_uart"])
        self.node = None
        self.started = self.stopped = self.abort_requested = False
        self.stop_error = None
        self.receipt = Finalization(self._select_action, self._send)

    def verify_installed(self, build):
        return verify_installed(self.fixture, build, self.root, usb_binding=self.binding)

    def capture_before(self):
        if self.node is not None:
            raise RuntimeError("baseline capture must precede UART observation")
        return self._capture("before")

    def _capture(self, name):
        return capture_storage(self.fixture, self.root, name, self.reader, self.seal,
                               self.run, usb_binding=self.binding)

    def prepare(self):
        if self.node is not None or self.stopped:
            raise RuntimeError("RF session already prepared or stopped")
        matches = [p.device for p in list_ports.comports() if fingerprint(p) == self.binding]
        if len(matches) != 1:
            raise ValueError("bound node USB device absent or ambiguous")
        self.node = NodeUART(matches[0], self.root, on_data=self.receipt.feed, usb_binding=self.binding)
        if self.node.port.identity != self.binding:
            self.node.close()
            raise ValueError("node USB identity changed before observation")

    def start(self):
        if self.node is None or self.started or self.stopped:
            raise RuntimeError("RF session cannot start")
        # A reset failure may still have started the autonomous app.
        self.started = True
        self.node.start()

    def _send(self, command):
        self.node.port.write(command)

    def _select_action(self):
        count = self.sleep_count()
        if not 1 <= count <= self.wakes:
            raise ValueError("RF receipt lacks a bound completed wake")
        if count == self.receipt.scope:
            return count, self.receipt.action  # Never change an uncertain receipt.
        return count, "STOP" if count == self.wakes or self.abort_requested else "SLEEP"

    def check_health(self):
        if self.node is not None and self.node.failure is not None:
            raise RuntimeError("UART observation failed: " + str(self.node.failure))
        if self.receipt.failure is not None:
            raise RuntimeError("RF receipt failed: " + str(self.receipt.failure))

    def sleep_count(self):
        self.check_health()
        return sleep_count((self.root / "node-uart.bin").read_bytes(), 10, self.wakes)

    @property
    def finalized(self):
        self.check_health()
        return (self.receipt.parked.is_set() and self.receipt.scope == self.wakes
                and self.sleep_count() == self.wakes)

    def stop(self):
        if self.stopped:
            return
        if self.stop_error is not None:
            raise self.stop_error
        try:
            if not self.started:
                if self.node is not None:
                    self.node.close()
                self.stopped = True
                return
            self.abort_requested = True
            deadline = time.monotonic() + self.stop_timeout
            while not self.receipt.parked.is_set():
                self.check_health()
                if time.monotonic() >= deadline:
                    raise TimeoutError("missing acknowledged RF stop handoff")
                self.receipt.parked.wait(min(0.05, max(0, deadline - time.monotonic())))
            self.check_health()
            if self.sleep_count() != self.receipt.scope:
                raise ValueError("RF stop handoff does not match captured wakes")
            # The final receipt keeps USB awake and radio work terminal. Finish
            # capture before esptool owns the stream; resolve USB identity anew
            # for every ROM command, including the subsequent flash read.
            self.node.close()
            identify(self.fixture, self.root, "final-stop-mac", usb_binding=self.binding)
            self.stopped = True
            write_json(self.root / "node-stop.json", dict(wake=self.receipt.scope,
                nonce=self.receipt.nonce.decode(), boundary="acknowledged park, then ROM stop"))
        except BaseException as error:
            self.stop_error = error
            write_json(self.root / "node-restoration-required.json", dict(error=str(error),
                action="operator must remove all node power; autonomous TX state unconfirmed"))
            if self.node is not None:
                try:
                    self.node.close()
                except BaseException:
                    pass
            raise

    def capture_after(self):
        if not self.stopped or not self.finalized:
            raise RuntimeError("final capture requires all wakes, acknowledged park and ROM stop")
        return self._capture("after")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seal-build", type=Path, required=True)
    args = parser.parse_args()
    write_json(args.seal_build / "production-rf-build.json", build_identity(args.seal_build))


if __name__ == "__main__":
    main()
