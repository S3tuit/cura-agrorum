"""Physical SPI/GPIO fake shared by command and owner tests, after both exist.

Only peripheral storage and explicit boundary hooks; no receiver policy.
"""
from collections import deque

class PhysicalPort:
    """A local physical-port fake, with explicit command-boundary injection.

    Register/buffer/IRQ storage models only the SPI peripheral. It knows no
    receiver state, profile, recovery policy, packet format or airtime policy.
    Tests supply all expected command bytes and decisions independently.
    """

    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.commands = []
        self.hooks = {}
        self.status = 0x24
        self.statuses = deque()
        self.registers = {}
        self.irq = 0
        self.errors = 0
        self.edges = deque()
        self.buffer = bytearray(b"packet")
        self.packet_status = b"\xc8\xf4\xc4"
        self.offset = 7
        self.busy_until = 0
        self.busy_forever = False
        self.after_transfer = None

    def open(self, configuration, *, deadline_monotonic_us):
        self.calls.append(("open", configuration, deadline_monotonic_us))

    def close(self):
        self.calls.append(("close",))

    def busy(self):
        return self.busy_forever or self.clock.now_monotonic_us() < self.busy_until

    def dio1(self):
        return bool(self.irq)

    def set_reset(self, *, asserted):
        self.calls.append(("reset", asserted, self.clock.now_monotonic_us()))
        if not asserted:
            self.status = 0x24
            self.irq = self.errors = 0
            self.registers.clear()

    def wait_edge(self, *, deadline_monotonic_us):
        self.calls.append(("edge", deadline_monotonic_us))
        if self.edges:
            return self.edges.popleft()
        return None

    def transfer(self, data, *, deadline_monotonic_us):
        self.commands.append(data)
        self.calls.append(("spi", data))
        if data[0] in self.hooks:
            self.hooks[data[0]](data)
        response = bytearray(len(data))
        opcode = data[0]
        if opcode == 0xC0:
            response[1] = self.statuses.popleft() if self.statuses else self.status
        elif opcode == 0x80:
            self.status = 0x24
        elif opcode == 0x82:
            self.status = 0x54
        elif opcode == 0x83:
            self.status = 0x64
        elif opcode == 0x12:
            response[2:] = self.irq.to_bytes(2, "big")
        elif opcode == 0x02:
            self.irq &= ~int.from_bytes(data[1:], "big")
        elif opcode == 0x17:
            response[2:] = self.errors.to_bytes(2, "big")
        elif opcode == 0x07:
            self.errors = 0
        elif opcode == 0x1D:
            address = int.from_bytes(data[1:3], "big")
            response[4:] = bytes(self.registers.get(address + i, 0) for i in range(len(data) - 4))
        elif opcode == 0x0D:
            address = int.from_bytes(data[1:3], "big")
            self.registers.update({address + i: value for i, value in enumerate(data[3:])})
        elif opcode == 0x13:
            response[2:] = bytes((len(self.buffer), self.offset))
        elif opcode == 0x1E:
            assert data[1] == self.offset
            response[3:] = self.buffer[:len(data) - 3]
        elif opcode == 0x14:
            response[2:] = self.packet_status
        elif opcode == 0x0E:
            self.buffer[:] = data[2:]
        result = bytes(response)
        if self.after_transfer is not None:
            self.after_transfer(data)
        return result

    def resynchronize_events(self, *, deadline_monotonic_us):
        self.calls.append(("resynchronize_events", deadline_monotonic_us))
        self.edges.clear()


class Wait:
    """Explicit wait boundary advances the manual clock; reads never do."""

    def __init__(self, clock):
        self.clock = clock
        self.deadlines = []

    def wait_until_monotonic_us(self, deadline_monotonic_us):
        self.deadlines.append(deadline_monotonic_us)
        self.clock.advance_elapsed_us(max(0, deadline_monotonic_us - self.clock.now_monotonic_us()))

