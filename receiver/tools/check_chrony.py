#!/usr/bin/python3
"""Read-only ExecStartPre policy check for the trusted pilot deployment."""

import subprocess
import sys


def validate_configuration(effective):
    directives = {}
    for line in effective.splitlines():
        fields = line.split("#", 1)[0].split()
        if fields:
            directives.setdefault(fields[0].lower(), []).append(fields[1:])
    for name, value in (
        ("leapsecmode", "slew"),
        ("maxslewrate", "3500"),
        ("cmdport", "0"),
        ("bindcmdaddress", "/run/chrony/chronyd.sock"),
    ):
        if directives.get(name) != [[value]]:
            raise ValueError(f"require exactly one '{name} {value}' directive")
    forbidden = {"makestep", "initstepslew", "rtcsync", "rtcfile"} & directives.keys()
    if forbidden:
        raise ValueError("forbidden directives: " + ", ".join(sorted(forbidden)))


def check_configuration(configuration, *, chronyd="/usr/sbin/chronyd"):
    effective = subprocess.run(
        [chronyd, "-p", "-f", str(configuration)],
        check=True, capture_output=True, text=True, timeout=10,
    ).stdout
    validate_configuration(effective)
    return effective


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: check-chrony.py CONFIGURATION", file=sys.stderr)
        return 2
    try:
        check_configuration(argv[0])
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Chrony policy check failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
