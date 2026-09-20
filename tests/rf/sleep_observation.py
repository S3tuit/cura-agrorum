"""Bounded UART evidence for the agreed accelerated RF sleep boundary."""
import re


def sleep_count(raw, duration_seconds, wakes):
    # Parse only complete lines. Boot ROM reset records independently delimit
    # marker episodes; repeated markers cannot stand in for actual wakes.
    boots = 0
    markers = 0
    for line in raw.split(b'\n')[:-1]:
        if b'panic' in line.lower() or b'Guru Meditation' in line:
            raise ValueError('node panic during accelerated run')
        if b'rst:0x' in line:
            boots += 1
            if boots > 1 and b'(SLEEP_WAKEUP)' not in line:
                raise ValueError('unexpected reset during accelerated run')
            if boots != markers + 1 or boots > wakes:
                raise ValueError('missing sleep marker or extra wake')
        if b'RF_NODE_SLEEP' in line:
            match = re.fullmatch(rb'RF_NODE_SLEEP duration_us=(\d+)\r?', line)
            if not match or int(match[1]) != duration_seconds * 1_000_000:
                raise ValueError('invalid sleep marker duration/framing')
            markers += 1
            if markers != boots or markers > wakes:
                raise ValueError('duplicate or unbound sleep marker')
    return markers
