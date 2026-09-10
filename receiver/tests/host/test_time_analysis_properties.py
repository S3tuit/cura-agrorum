"""Reviewed primitive timeline oracle and generated recorded-history schedules."""

from hypothesis import settings, strategies as st
from hypothesis.stateful import RuleBasedStateMachine, invariant, rule

from cura_receiver.clock_correlation import AnalysisInstance, ClockCorrelation
from cura_receiver.generated.receiver_entities_generated import ClockObservationV1
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth,
    SystemTimeQuality,
)


class ReferenceTimeline:
    """Replay observations and events as a merged stream, with pending backfill lists.

    The oracle uses primitive tuples and unbounded arithmetic only. It does not
    call the production index, arithmetic, builders or generated codecs.
    """

    def assign(self, starts, observations, events):
        results = [None] * len(events)
        for identity, start in starts.items():
            stream = []
            for instance, sequence, monotonic, utc, step in observations:
                if instance == identity:
                    stream.append((monotonic, 0, sequence, utc, step))
            for index, (instance, monotonic) in enumerate(events):
                if instance == identity and monotonic >= start:
                    stream.append((monotonic, 1, index, None, False))
            anchor = None
            blocked = False
            pending = []
            for monotonic, kind, index, utc, step in sorted(stream):
                if kind == 0:
                    if utc is not None:
                        anchor = (monotonic, utc, index)
                        blocked = False
                        for event_index, event_time in pending:
                            value = utc + event_time - monotonic
                            if -(1 << 63) <= value < 1 << 63:
                                results[event_index] = (value, identity, index)
                        pending = []
                    else:
                        anchor = None
                        if step:
                            blocked = True
                            pending = []
                elif anchor is not None:
                    value = anchor[1] + monotonic - anchor[0]
                    if -(1 << 63) <= value < 1 << 63:
                        results[index] = (value, identity, anchor[2])
                elif not blocked:
                    pending.append((index, monotonic))
        return results


# Reviewed ordinary-gap backfill retains UTC zero and never borrows another receiver instance's anchor.
def test_reference_ordinary_backfill():
    assert ReferenceTimeline().assign(
        {1: 10, 2: 10},
        [(1, 0, 20, None, False), (1, 1, 30, 0, False)],
        [(1, 10), (1, 20), (1, 30), (1, 40), (2, 30)],
    ) == [(-20, 1, 1), (-10, 1, 1), (0, 1, 1), (10, 1, 1), None]


# Reviewed step replay discards older pending events and remains blocked across ordinary quality changes.
def test_reference_step_gap():
    assert ReferenceTimeline().assign(
        {1: 0},
        [(1, 0, 10, None, True), (1, 1, 20, None, False), (1, 2, 30, 100, False)],
        [(1, 0), (1, 10), (1, 20), (1, 29), (1, 30)],
    ) == [None, None, None, None, (100, 1, 2)]


# A trusted observation after a same-microsecond step precedes events and leaves an empty gap.
def test_reference_same_time_restart():
    assert ReferenceTimeline().assign(
        {1: 0},
        [(1, 0, 10, None, True), (1, 1, 10, 50, False)],
        [(1, 9), (1, 10)],
    ) == [None, (50, 1, 1)]


class RecordedTimeHistory(RuleBasedStateMachine):
    def __init__(self):
        super().__init__()
        self.ordinal = self.boot = 1
        self.monotonic = 0
        self.sequence = 0
        self.instances = [
            AnalysisInstance(1, self.identity, self.boot.to_bytes(16, "little"), 0)
        ]
        self.starts = {self.identity: 0}
        self.observations = []
        self.primitives = []
        self.events = []

    @property
    def identity(self):
        return self.ordinal.to_bytes(16, "little")

    def append(self, utc, step, quality, health):
        self.observations.append(
            ClockObservationV1(
                self.identity,
                self.sequence,
                self.sequence,
                self.monotonic,
                utc,
                step,
                quality,
                health,
            )
        )
        self.primitives.append(
            (self.identity, self.sequence, self.monotonic, utc, step)
        )
        self.sequence += 1

    # Completed network observations vary UTC, RTC health and repeated monotonic samples independently.
    @rule(
        delta=st.integers(0, 10),
        utc=st.one_of(
            st.integers(-1000, 1000), st.sampled_from([-(1 << 63), (1 << 63) - 1])
        ),
        health=st.sampled_from(list(RtcHealth)),
    )
    def network_observation(self, delta, utc, health):
        self.monotonic += delta
        self.append(utc, False, SystemTimeQuality.NETWORK_SYNCED, health)

    # An unusable RTC result records ordinary quality loss; usable direct evidence records holdover.
    @rule(
        delta=st.integers(0, 10), usable=st.booleans(), utc_second=st.integers(-10, 10)
    )
    def rtc_read(self, delta, usable, utc_second):
        self.monotonic += delta
        self.append(
            utc_second * 1_000_000 + 500_000 if usable else None,
            False,
            SystemTimeQuality.RTC_HOLDOVER if usable else SystemTimeQuality.UNTRUSTED,
            RtcHealth.PRESENT if usable else RtcHealth.INVALID,
        )

    # Expiry is an ordinary observation, including while an earlier step gap remains open.
    @rule(delta=st.integers(0, 10))
    def expire_quality(self, delta):
        self.monotonic += delta
        self.append(None, False, SystemTimeQuality.UNTRUSTED, RtcHealth.PRESENT)

    # Analysis treats a recorded step boundary identically for rejected, submitted and unknown commands.
    @rule(
        delta=st.integers(0, 10),
        outcome=st.sampled_from(["not_submitted", "submitted", "unknown"]),
    )
    def recorded_step(self, delta, outcome):
        self.monotonic += delta
        self.append(None, True, SystemTimeQuality.UNTRUSTED, RtcHealth.PRESENT)

    # Restarts introduce real instance fences; a reboot additionally resets the monotonic domain.
    @rule(reboot=st.booleans())
    def process_start(self, reboot):
        self.ordinal += 1
        if reboot:
            self.boot += 1
            self.monotonic = 0
        self.sequence = 0
        self.instances.append(
            AnalysisInstance(
                self.ordinal,
                self.identity,
                self.boot.to_bytes(16, "little"),
                self.monotonic,
            )
        )
        self.starts[self.identity] = self.monotonic

    # Newly recorded events include boundary equality and later reevaluation of existing instance history.
    @rule(delta=st.integers(0, 10))
    def event(self, delta):
        self.monotonic += delta
        self.events.append((self.identity, self.monotonic))

    # Every schedule prefix must agree with the independently replayed assignments, including absence and provenance.
    @invariant()
    def assignments_match(self):
        expected = ReferenceTimeline().assign(self.starts, self.primitives, self.events)
        production = ClockCorrelation(
            reversed(self.instances), reversed(self.observations)
        )
        actual = []
        for identity, monotonic in self.events:
            result = production.correlate(identity, monotonic)
            actual.append(
                None
                if result is None
                else (
                    result.utc_us,
                    result.clock_observation_receiver_instance_id,
                    result.clock_observation_sequence,
                )
            )
        assert actual == expected


# Hypothesis shrinks unsafe assignments through changes, step gaps and process/boot transitions.
TestRecordedTimeHistory = RecordedTimeHistory.TestCase
TestRecordedTimeHistory.settings = settings(
    max_examples=100, stateful_step_count=60, derandomize=True, deadline=None
)
