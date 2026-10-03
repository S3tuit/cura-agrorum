"""Production policy + real SQLite, checked by a separate physical-history oracle."""
from dataclasses import replace
from fractions import Fraction as F

from cura_receiver.airtime_ledger import AirtimeCorrelation
from cura_receiver.communicator_state_owner import CommunicatorStateOwner
from cura_receiver.generated.receiver_enums_generated import (
    DiagnosticOperation as Op, RtcHealth as RH, SystemTimeQuality as Q,
    PersistenceControlPurpose as Purpose,
)
from cura_receiver.persistence_control_values import (
    CommunicatorStateCommitDisposition as CD, CommunicatorStateCommitFailureKind as CF,
)
from cura_receiver.time_observations import TrustedTimeSample
from cura_receiver.tx_airtime import TxAirtimePolicy, TxCertainty
from tests.support.builders.persistence_control import state
from tests.support.models.tx_airtime import Oracle, P, B, C


class CommitFaults:
    def __init__(self, control):
        self.control, self.mode, self.fail_load = control, 'committed', False

    def commit_communicator_state(self, value, **kwargs):
        mode, self.mode = self.mode, 'committed'
        if mode in ('not_installed', 'unknown_absent'):
            kwargs['deadline_monotonic_us'] = 0
        result = self.control.commit_communicator_state(value, **kwargs)
        assert result.disposition in (CD.COMMITTED, CD.ALREADY_COMMITTED, CD.NOT_INSTALLED)
        if mode.startswith('unknown'):
            return replace(result, disposition=CD.OUTCOME_UNKNOWN,
                           failure_kind=CF.DEADLINE_EXCEEDED, operation=Op.WRITE)
        return result

    def load_communicator_state(self, **kwargs):
        if self.fail_load:
            kwargs['deadline_monotonic_us'] = 0
        return self.control.load_communicator_state(**kwargs)


class PhysicalRun:
    def __init__(self, component, *, longest=67_866, empty=True):
        self.policy, self.worker, self.database, self.clock, _ = component(initial_state=state() if empty else None)
        self.channel = CommitFaults(self.worker.control)
        self.policy.owner._control = self.channel
        self.policy.longest_packet_charge_us = longest
        self.longest = longest
        self.physical, self.oracle, self.sent, self.denied, self.saves = F(0), Oracle(), 0, 0, 0
        self.trust()
        self.policy.recover(deadline_monotonic_us=self.deadline())
        self.check()

    def deadline(self):
        return self.clock.now_monotonic_us()+5_000_000

    def trust(self, trusted=True, offset=0, error=1):
        now = self.clock.now_monotonic_us()
        # Includes the fractional physical-time rounding error explicitly.
        sample = TrustedTimeSample(now, int(self.physical)+offset,
                                  abs(offset)+error, Q.NETWORK_SYNCED, 1)
        self.policy.update_time(AirtimeCorrelation(sample, 1, now+10_000_000_000) if trusted else None,
                                rtc_health=RH.PRESENT)

    def live(self):
        if self.policy._ledger is not None:
            return self.policy._ledger
        prepared = self.policy._prepared
        return None if prepared is None else prepared.recovery_ledger

    def check(self):
        used = self.oracle.at(self.physical)
        assert used <= B
        ledger = self.live()
        if ledger is not None:
            assert C*sum(d > self.clock.now_monotonic_us() for d in ledger.deadlines) >= used

    def advance(self, mono_delta, rate=P):
        start = self.clock.now_monotonic_us()
        ledger = self.live()
        if ledger is not None:
            for edge in sorted(set(d for d in ledger.deadlines if start < d <= start+mono_delta)):
                actual = self.oracle.at(self.physical+F((edge-start)*P, rate))
                assert C*sum(d > edge for d in ledger.deadlines) >= actual
        self.clock.advance_elapsed_us(mono_delta)
        self.physical += F(mono_delta*P, rate)
        self.check()

    def transmit(self, charge=67_866, certainty=TxCertainty.STARTED):
        result = self.policy.try_spend(charge)
        if result.token is None:
            self.denied += 1
            self.check()
            return False
        self.policy.report_tx(result.token, certainty)
        if certainty is not TxCertainty.NOT_STARTED:
            self.oracle.transmit(self.physical, charge)
            self.sent += 1
        self.check()
        return True

    def save(self, *, rtc=False):
        self.trust()
        if rtc and self.policy.owner.pending is None:
            now = self.clock.now_monotonic_us()
            requested = self.policy.snapshot(provenance=None, snapshot_monotonic_us=now,
                snapshot_utc_us=int(self.physical), previous_state=self.policy.state)
            if requested is not None:
                receipt = self.policy.owner.commit(requested, deadline_monotonic_us=self.deadline(),
                                                   purpose=Purpose.RTC_PROVENANCE)
                self.policy.snapshot_receipt(receipt)
                self.saves += 1
        else:
            self.policy.save(deadline_monotonic_us=self.deadline())
            self.saves += 1
        self.check()

    def restart(self, *, trusted=True, fresh_clock=False, offset=0):
        # A serialized load follows any possibly committed request on the same
        # worker channel, as the next receiver process's startup does.
        self.channel.fail_load = False
        if fresh_clock:
            with self.clock._lock:
                self.clock._monotonic_us = 100
        loaded = self.channel.load_communicator_state(deadline_monotonic_us=self.deadline())
        owner = CommunicatorStateOwner.from_load(control=self.channel, loaded=loaded)
        self.policy = TxAirtimePolicy(state_owner=owner, clock=self.clock,
                                     longest_packet_charge_us=self.longest)
        self.trust(trusted, offset=offset)
        self.policy.recover(deadline_monotonic_us=self.deadline())
        self.check()


