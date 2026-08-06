# Vendored Semtech SX126x driver

The files in this directory are the unmodified command/register subset of
Semtech `sx126x_driver` v2.5.0 from `Lora-net/SWL2001` commit
`a8ddc544043e72807cf7db532478e1dda734ae7c`:

- `sx126x.c`
- `sx126x.h`
- `sx126x_hal.h`
- `sx126x_regs.h`
- `sx126x_status.h`

LR-FHSS, BPSK, version and build-system files are not vendored because this
component does not compile or expose those features. The upstream Clear BSD
license is retained in `LICENSE.txt` and in each source file.
