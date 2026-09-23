# Pi component RF peer

`peer.py` runs as the configured non-root receiver user in a fresh, source-sealed
directory staged by the laptop. It verifies the Pi serial, nominal fixture and
sources before GPIO/SPI access. GPIO line requests and a process lock enforce
exclusive ownership. The production service must be stopped.

Normal cases compose `Radio -> Sx1262 -> LinuxRadioIo`. RF-006 is explicitly
labelled `Sx1262/LinuxRadioIo`: its finite burst does not claim the Radio owner's
one-response-per-occurrence behavior. The backend retains its fixed production
PHY and watchdog. This peer has no protocol acceptance, SQLite or durable
allowance owner; its authorization is the operator's component-test reservation.

The process reports ready, waits for a matching GO command, and owns a finite
45-second episode. UART/SSH never schedule a packet response: Pi-local RX_DONE
and its own monotonic clock establish the response targets. The last possible
SetTx is bounded before the lease end, including watchdog margin. EOF, STOP or
signals inhibit later operations. Completion retains actual SPI/IRQ samples,
TX certainty, profiles and shutdown/handle-release results. A lost connection
or failed cleanup requires explicit restoration; killing SSH proves no safe
radio state by itself.

Use [tests/rf/README.md](../../../tests/rf/README.md) and the fixture-specific
launcher; ordinary receiver hardware targets remain Pi-local.
