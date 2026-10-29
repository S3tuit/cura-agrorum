"""Validate captured core output against the wire contract and its own acquisition."""
import re
import struct


def record_fields(serial, marker):
    matches = re.findall(rb'(?:^|\n)' + marker.encode() + rb' ([^\r\n]+)', serial)
    if len(matches) != 1:
        raise ValueError(f'expected exactly one {marker} record')
    pairs = [part.split('=') for part in matches[0].decode().split()]
    if any(len(pair) != 2 for pair in pairs) or len({pair[0] for pair in pairs}) != len(pairs):
        raise ValueError(f'malformed or duplicate {marker} fields')
    return dict(pairs)


def parse_sample(serial):
    fields = record_fields(serial, 'CARRIER_SAMPLE')
    needed = {'result', 'duration_us', 'validity', 'soil0_mv', 'soil1_mv',
              'temp0_centi_c', 'temp1_centi_c', 'enclosure_centi_c',
              'pressure_pa', 'humidity_centi_pct'}
    if set(fields) != needed:
        raise ValueError('incomplete sample record')
    return {k: int(v, 16 if k in {'result', 'validity'} else 10) for k, v in fields.items()}


# INTERFACE's five component groups and protocol README's bits 1..7.
FIXTURES = {'nominal': (0x1f, 0x00fe), 'missing_ds0': (0x1b, 0x00f6),
            'missing_ds1': (0x17, 0x00ee), 'missing_bme280': (0x0f, 0x001e),
            'adc_reference': (0x1f, 0x00fe)}
WIRE_FIELDS = ('sample_id', 'run_ms', 'soil0_mv', 'soil1_mv', 'temp0_centi_c',
               'temp1_centi_c', 'enclosure_centi_c', 'pressure_pa', 'humidity_centi_pct',
               'reset_reason', 'previous_current_tx_attempts', 'previous_awake_ms',
               'previous_current_delivery_ms', 'previous_cycle_tx_attempts',
               'previous_cycle_accepted_readings', 'flags')
SENSOR_FIELDS = WIRE_FIELDS[2:9]


def validate_reading(serial, fixture):
    if fixture not in FIXTURES:
        raise ValueError('unknown reading fixture')
    validity, flags = FIXTURES[fixture]
    sample = parse_sample(serial)
    if (sample['validity'] != validity or
            sample['result'] != (0 if validity == 0x1f else 0x00030002) or
            not 0 <= sample['duration_us'] <= 30_000_000):
        raise AssertionError('sample outcome does not match the declared reading fixture')
    fields = record_fields(serial, 'CARRIER_READING')
    counts = {'samples': 1, 'tx_calls': 1, 'rx_calls': 0,
              'terminal_calls': 1, 'sleep_us': 900_000_000}
    if set(fields) != set(counts) | {'body', 'pending_body'}:
        raise ValueError('incomplete core reading record')
    if any(int(fields[key]) != value for key, value in counts.items()):
        raise AssertionError('core wake did not execute exactly one acquisition/capture/terminal call')
    for key in ('body', 'pending_body'):
        if not re.fullmatch('[0-9a-f]{64}', fields[key]):
            raise ValueError('expected a canonical 32-byte body')
    if fields['body'] != fields['pending_body']:
        raise AssertionError('captured frame and persisted canonical bodies differ')
    body = bytes.fromhex(fields['body'])
    # Independent decoding of the README's offsets/types, not a test builder.
    reading = dict(zip(WIRE_FIELDS, struct.unpack('<IHHHhhhIHBBHHBBH', body)))
    if (reading['sample_id'] != 0 or reading['flags'] & 0xfffe != flags or
            bool(reading['flags'] & 1) != (reading['reset_reason'] == 8) or
            any(reading[name] for name in WIRE_FIELDS[10:15])):
        raise AssertionError('reading flags/metadata violate the fresh-cycle contract')
    for bit, name in enumerate(SENSOR_FIELDS, 1):
        if reading[name] != sample[name]:
            raise AssertionError(f'{name} differs from the same core acquisition')
        if not flags & (1 << bit) and reading[name] != 0:
            raise AssertionError(f'invalid {name} must be zero')
    if fixture != 'adc_reference' and any(
            not 2000 <= sample[name] <= 2700 for name in SENSOR_FIELDS[:2]):
        raise AssertionError('connected air-probe reading outside 2000..2700 mV')

    diagnostic = record_fields(serial, 'CARRIER_DIAGNOSTIC')
    if set(diagnostic) != {'operation', 'schema', 'length', 'context'}:
        raise ValueError('incomplete sensor diagnostic record')
    partial = validity != 0x1f
    if ([int(diagnostic[k]) for k in ('operation', 'schema', 'length')] !=
            ([1, 1, 48] if partial else [0, 0, 0]) or
            not re.fullmatch('[0-9a-f]{504}', diagnostic['context'])):
        raise AssertionError('sensor diagnostic header does not match fixture')
    context = bytes.fromhex(diagnostic['context'])
    expected = bytearray(252)
    missing = {'missing_ds0': (24, 2, 261), 'missing_ds1': (32, 2, 261),
               'missing_bme280': (40, 1, 264)}
    if fixture in missing:
        offset, kind, status = missing[fixture]
        struct.pack_into('<Ii', expected, offset, kind, status)
    if context != expected:
        raise AssertionError('sensor diagnostic slots do not match fixture')
    return dict(sample=sample, reading=reading, body=fields['body'],
                pending_body=fields['pending_body'], diagnostic=diagnostic, **counts)
