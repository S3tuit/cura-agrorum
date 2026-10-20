"""Reviewed protocol bytes for host orchestration tests; no DUT measurements."""
import struct


# Reviewed wire-layout example: ID=0, run=1345 ms, soil=2301/2402 mV,
# temperatures=-1234/2345/2500 centi-C, pressure=100123 Pa, humidity=4567,
# POWERON=1, no previous metrics, seven sensor flags. These bytes are local
# parser test input; target expectations never come from this fixture builder.
BODY = bytes.fromhex('00000000 4105 fd08 6209 2efb 2909 c409 1b870100 d711 '
                     '01 00 0000 0000 00 00 fe00')


def serial_reading(fixture='nominal', *, position='A'):
    body = bytearray(BODY)
    validity = {'nominal': 0x1f, 'adc_reference': 0x1f, 'missing_ds0': 0x1b,
                'missing_ds1': 0x17, 'missing_bme280': 0x0f}[fixture]
    flags = {'nominal': 0xfe, 'adc_reference': 0xfe, 'missing_ds0': 0xf6,
             'missing_ds1': 0xee, 'missing_bme280': 0x1e}[fixture]
    sample = dict(result=0 if validity == 0x1f else 0x30002, duration_us=1_340_000,
                  validity=validity, soil0_mv=2301, soil1_mv=2402,
                  temp0_centi_c=-1234, temp1_centi_c=2345,
                  enclosure_centi_c=2500, pressure_pa=100123, humidity_centi_pct=4567)
    if fixture == 'adc_reference':
        sample.update(soil0_mv=1183, soil1_mv=1645)
        body[6:10] = bytes.fromhex('9f04 6d06')
        if position == 'B':
            sample.update(soil0_mv=1645, soil1_mv=1183)
            body[6:10] = bytes.fromhex('6d06 9f04')
    if fixture == 'missing_ds0':
        sample['temp0_centi_c'] = 0
        body[10:12] = b'\0\0'
    if fixture == 'missing_ds1':
        sample['temp1_centi_c'] = 0
        body[12:14] = b'\0\0'
    if fixture == 'missing_bme280':
        sample.update(enclosure_centi_c=0, pressure_pa=0, humidity_centi_pct=0)
        body[14:22] = b'\0' * 8
    body[30:32] = bytes((flags, 0))
    context = bytearray(252)
    if fixture.startswith('missing'):
        offset = {'missing_ds0': 24, 'missing_ds1': 32, 'missing_bme280': 40}[fixture]
        struct.pack_into('<Ii', context, offset, 1 if fixture == 'missing_bme280' else 2,
                         264 if fixture == 'missing_bme280' else 261)
    sample_line = ' '.join(f'{key}={value:08x}' if key in {'validity', 'result'} else f'{key}={value}'
                           for key, value in sample.items())
    partial = fixture.startswith('missing')
    return (f'CARRIER_READING samples=1 tx_calls=1 rx_calls=0 terminal_calls=1 '
            f'sleep_us=900000000 body={body.hex()} pending_body={body.hex()}\n'
            f'CARRIER_SAMPLE {sample_line}\n'
            f'CARRIER_DIAGNOSTIC operation={int(partial)} schema={int(partial)} '
            f'length={48 if partial else 0} context={context.hex()}\n').encode()
