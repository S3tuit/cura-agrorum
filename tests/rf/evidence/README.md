# Manual RF faults — 18 September 2026

Both retained cases require physical rewiring and powered-off restoration.
C6 `cc8da2fc0224`, Pi `00000000e0027211`, firmware ELF
`6e8acd2deb2a2eefc53ddc7aa2d08a05cd44b478d3037df33f1873567c61cd3a`.

| Case | Observed result | Restoration |
|---|---|---|
| RF-012, DIO1 disconnected with MCU-side RN5 retained | PASS: one C6 TX attempt, missing TxDone, TRANSMIT/WAIT_IRQ/HARDWARE_TOUCHED; returned 6,060 µs after deadline within 50,000 µs allowance. Pi copied exact pattern A and sent nothing. | Operator reconnected DIO1 unpowered; fresh RF-001 PASS |
| RF-013, radio absent, BUSY high/MISO low, RN5 retained | PASS: INITIALIZE/WAIT_BUSY `0x20004` in 21,631 µs, no SetTx. Pi observed no packets or TX. | Operator removed both ties unpowered and reconnected radio; fresh RF-001 PASS |

[results.json](results.json) consolidates the source/build binding, run IDs,
essential C6 observations, endpoint outcomes and restoration. Both C6 runs
entered timer deep sleep and woke to nontransmitting idle. Pi cleanup/exit
passed. RF-013 permits a C6 cleanup BUSY error with the module absent; this is
not physical radio-sleep proof. Original archives were verified before their
earlier curation; this extract does not retain replayable raw traces.

These are historical component results, not qualification of a changed image,
full service, physical RF timing/power or durable airtime enforcement. Nominal
checks and wiring mistakes need no permanent archive. Follow the
[retention policy](../../../EVIDENCE.md) and [RF procedure](../README.md).
