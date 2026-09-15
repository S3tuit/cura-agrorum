"""Laptop-only SSH bridge for the Pi maximum-slew test, with bounded UTC evidence.

This is an invasive test runner, not a deployed receiver service. Requires an
explicit opt-in and the Pi's marked test root. It never changes laptop clocks.
"""

import argparse
from decimal import Decimal, ROUND_CEILING
import ipaddress
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

SLEW_MIN_US = 1_200_000_000
SLEW_MAX_US = 2_400_000_000
SLEW_COMPONENT_TIMEOUT_S = 2700


def reference_value(before_us, after_us, csv, pi_addresses):
    fields = csv.strip().split(",")
    if (
        len(fields) != 14
        or fields[13] != "Normal"
        or fields[1] in pi_addresses
        or fields[0] in ("00000000", "7F7F0101")
        or not 1 <= int(fields[2]) <= 15
        or not 0 <= after_us - before_us <= 100_000
    ):
        raise ValueError(
            "laptop reference is unsynchronized, dependent, stepped or too slow"
        )
    correction, delay, dispersion = map(Decimal, (fields[4], fields[10], fields[11]))
    if (
        not all(v.is_finite() for v in (correction, delay, dispersion))
        or min(delay, dispersion) < 0
    ):
        raise ValueError("invalid reference tracking uncertainty")
    base = int(
        ((abs(correction) + delay / 2 + dispersion) * 1_000_000).to_integral_value(
            rounding=ROUND_CEILING
        )
    )
    # The tracking sample occurs inside this laptop UTC interval. Expand its
    # midpoint to include the whole interval plus Chrony's reported clock error.
    midpoint = (before_us + after_us) // 2
    error = base + (after_us - before_us + 1) // 2 + 1
    if error > 100_000:
        raise ValueError("laptop uncertainty exceeds fixture ceiling")
    return {
        "version": 1,
        "utc_us": midpoint,
        "error_us": error,
        "source_independent": True,
        "tracking_csv": csv.strip(),
        "laptop_before_us": before_us,
        "laptop_after_us": after_us,
    }


def rate_interval(first, last):
    elapsed = last["utc_us"] - first["utc_us"]
    error = first["error_us"] + last["error_us"]
    if elapsed <= error:
        raise ValueError("no positive independent elapsed interval")
    low = (
        Decimal(last["pi_before_us"] - first["pi_after_us"])
        / (elapsed + error)
        * 1_000_000
        - 1_000_000
    )
    high = (
        Decimal(last["pi_after_us"] - first["pi_before_us"])
        / (elapsed - error)
        * 1_000_000
        - 1_000_000
    )
    if low > high:
        raise ValueError("invalid Pi measurement brackets")
    return low, high


def slew_decision(elapsed_us, low, high, sign):
    """Stop on the first resolved result after 20 minutes, with a 40-minute cap."""
    if (
        sign not in (-1, 1)
        or elapsed_us < 0
        or not low.is_finite()
        or not high.is_finite()
        or low > high
    ):
        raise ValueError("invalid slew measurement")
    if elapsed_us < SLEW_MIN_US:
        return "CONTINUE"
    if high - low >= 200:
        return "CONTINUE" if elapsed_us < SLEW_MAX_US else "FAIL_RESOLUTION"
    if not -3700 <= low <= high <= 3700:
        return "FAIL_RATE_BOUND"
    if (sign > 0 and low <= 3300) or (sign < 0 and high >= -3300):
        return "FAIL_MAXIMUM_SLEW"
    return "PASS"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--confirm-receiver-destructive", action="store_true", required=True
    )
    parser.add_argument("--pi-stage", required=True)
    parser.add_argument("--pi-python", required=True)
    parser.add_argument("--pi-test-root", required=True)
    parser.add_argument(
        "--pi-address",
        type=ipaddress.ip_address,
        help="optional numeric address for cura-receiver; retains its SSH host-key identity",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--slew-only", action="store_true", help="run only the two maximum-slew cases"
    )
    args = parser.parse_args()
    for value in (args.pi_stage, args.pi_python, args.pi_test_root):
        if not value.startswith("/") or "\n" in value or "\x00" in value:
            parser.error("absolute Pi paths required")
    # sshpass reads SSH_PASSWORD; sudo consumes exactly its own first line.
    password = os.environ.get("SSH_PASSWORD")
    if not password:
        parser.error("set SSH_PASSWORD for the authorized cura bench account")
    ssh = ["sshpass", "-e", "ssh", "-F", "/dev/null"]
    if args.pi_address is not None:
        ssh += ["-o", f"HostName={args.pi_address}", "-o", "HostKeyAlias=cura-receiver"]
    ssh.append("cura@cura-receiver")
    env = {**os.environ, "SSHPASS": password}
    addresses = subprocess.check_output(
        ssh + ["hostname -I"], text=True, env=env, timeout=10
    ).split()
    before = time.time_ns() // 1000
    tracking = subprocess.check_output(
        ["/usr/bin/chronyc", "-n", "-c", "tracking"], text=True, timeout=2
    )
    reference_value(
        before, time.time_ns() // 1000, tracking, addresses
    )  # Reject missing reference before Pi mutation.
    argv = [
        "sudo",
        "-k",
        "-S",
        "-p",
        "",
        "env",
        "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1",
        "CURA_LAPTOP_REFERENCE=SSH_STDIO_V1",
        args.pi_python,
        "-m",
        "pytest",
        "-c",
        args.pi_stage + "/receiver/pytest.ini",
        args.pi_stage + "/receiver/tests/hardware/test_time_mutations.py",
        "--receiver-hardware",
        "--confirm-receiver-destructive",
        "--receiver-test-root=" + args.pi_test_root,
        "-s",
        "-q",
        "-x",
    ]
    if args.slew_only:
        argv += ["-k", "maximum_slew_against_laptop"]
    with args.output.open("x") as log:
        child = subprocess.Popen(
            ssh + [shlex.join(argv)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )
        child.stdin.write(password + "\n")
        child.stdin.flush()
        try:
            # A reference failure sends invalid evidence, allowing pytest's fixture
            # finalizer to restore the Pi. Never kill the remote restoration owner.
            for line in child.stdout:
                print(line, end="", flush=True)
                log.write(line)
                log.flush()
                if line.strip() == "CURA_REFERENCE_REQUEST":
                    try:
                        before = time.time_ns() // 1000
                        csv = subprocess.check_output(
                            ["/usr/bin/chronyc", "-n", "-c", "tracking"],
                            text=True,
                            timeout=2,
                        )
                        value = reference_value(
                            before, time.time_ns() // 1000, csv, addresses
                        )
                    except Exception as error:
                        value = {"version": 0, "error": str(error)}
                    # Preserve the laptop evidence even if the Pi rejects the reply
                    # or its component is terminated before writing a final result.
                    log.write("CURA_REFERENCE_RESPONSE " + json.dumps(value) + "\n")
                    log.flush()
                    child.stdin.write(json.dumps(value) + "\n")
                    child.stdin.flush()
            code = child.wait(timeout=10)
            if code:
                raise SystemExit(code)
        finally:
            child.stdin.close()
            child.stdout.close()
            if child.poll() is None:
                print(
                    "SSH is still running; leave the Pi restoration owner alive and inspect its journal.",
                    file=sys.stderr,
                )


if __name__ == "__main__":
    main()
