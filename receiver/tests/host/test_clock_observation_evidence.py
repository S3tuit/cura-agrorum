"""Published clock observations carry the evidence that reproduces their bounds.

Every expected bound here is recomputed only from stored row columns plus the
deployed policy constants, as the offline analysis must do.
"""

import sqlite3
from pathlib import Path
from types import SimpleNamespace

import pytest

from cura_receiver.clock_correlation import clock_observation_from_row
from cura_receiver.elapsed_duration import (
    exclusive_trust_distance_us,
    maximum_physical_half_bracket_us,
    rate_growth_us,
)
from cura_receiver.generated import receiver_entities_generated as row
from cura_receiver.generated import receiver_enums_generated as E
from cura_receiver.generated.receiver_entities_generated import RtcProvenanceV1
from cura_receiver.ports.chrony import ChronyTrackingResult, ChronyQueryStatus as Q
from cura_receiver.ports.ds3231 import Ds3231ReadResult, Ds3231ReadStatus as R
from cura_receiver.ports.kernel_clock import KernelClockResult, KernelSampleStatus as K
from cura_receiver.time_observations import (
    SampleEvidence,
    TrustedTimeSample,
    observation_schedule,
    trusted_clock_observation,
)
from cura_receiver.time_policy import TimePolicy
from tests.host.test_runtime_time import UTC, runtime
from tests.support.fakes.chrony import FakeChronyControl
from tests.support.fakes.ds3231 import FakeDs3231Control

POLICY = TimePolicy()
SCHEMA = Path(__file__).resolve().parents[2] / "db" / "schema.sql"
OTHER_INSTANCE = b"\x01" * 16


def tracking_at(rt, *, started, correction=-2_640_071):
    """A Pi-shaped successful tracking result whose query ended at the current time."""
    return ChronyTrackingResult(
        Q.OK,
        started,
        rt.clock.now_monotonic_us(),
        True,
        True,
        correction,
        20_286,
        166,
        reference_id=0xA29FC87B,
        reference_time_utc_us=UTC - 30_000_000,
        stratum=4,
        root_delay_us=38_471,
        root_dispersion_us=1_051,
        estimated_frequency_ppb=6_208,
    )


def kernel_read(rt, kernel, width=300):
    def result():
        start = rt.clock.now_monotonic_us()
        rt.clock.advance_elapsed_us(width)
        return KernelClockResult(K.OK, start, rt.clock.now_monotonic_us(), UTC, 5, 0x2040)

    kernel.results.append(result)


def rtc_read(rt, rtc, utc_s, width=2_000):
    def result():
        start = rt.clock.now_monotonic_us()
        rt.clock.advance_elapsed_us(width)
        return Ds3231ReadResult(R.OK, start, rt.clock.now_monotonic_us(), utc_s)

    rtc.read_results.append(result)


def with_provenance(rt, provenance):
    rt.state_owner = SimpleNamespace(
        state=SimpleNamespace(rtc_provenance=provenance), pending=None
    )


def round_trip(*observations):
    """Store through the generated binder and rebuild from the exact stored columns."""
    connection = sqlite3.connect(":memory:")
    connection.executescript(SCHEMA.read_text(encoding="utf-8"))
    connection.execute("PRAGMA foreign_keys = OFF")
    columns = row.CLOCK_OBSERVATION_V1_COLUMNS
    insert = (
        f"INSERT INTO clock_observations ({', '.join(columns)}) "
        f"VALUES ({', '.join('?' for _ in columns)})"
    )
    for observation in observations:
        connection.execute(insert, row.clock_observation_v1_parameters(observation))
    stored = connection.execute(
        f"SELECT {', '.join(columns)} FROM clock_observations "
        "ORDER BY observation_sequence"
    ).fetchall()
    return [clock_observation_from_row(values) for values in stored]


def horizon(observation):
    distance = exclusive_trust_distance_us(
        observation.error_bound_us,
        POLICY.receiver_utc_error_budget_us,
        rate_bound_ppm=POLICY.monotonic_elapsed_rate_bound_ppm,
    )
    return observation.sampled_at_monotonic_us + distance


def recomputed_network_error(observation):
    network = observation.network_evidence
    projection = rate_growth_us(
        POLICY.monotonic_elapsed_rate_bound_ppm,
        observation.sampled_at_monotonic_us - network.tracking_started_at_monotonic_us,
    )
    return (
        abs(network.remaining_correction_us)
        + network.root_distance_us
        + POLICY.time_sampling_margin_us
        + projection
    )


def recomputed_rtc_error(observation):
    rtc = observation.rtc_evidence
    drift = rate_growth_us(
        rtc.reference_drift_bound_ppm,
        observation.sampled_at_utc_us - rtc.reference_readback_utc_us,
    )
    assert drift == rtc.drift_contribution_us
    read = (
        500_000
        + maximum_physical_half_bracket_us(
            observation.sample_finished_at_monotonic_us
            - observation.sample_started_at_monotonic_us,
            rate_bound_ppm=POLICY.monotonic_elapsed_rate_bound_ppm,
        )
        + POLICY.time_sampling_margin_us
    )
    return rtc.reference_uncertainty_us + drift + read


# A network observation stores the kernel bracket, the complete supporting tracking
# result and a bound/horizon that analysis reproduces from the stored row alone.
def test_network_observation_reproduces_bound_and_horizon():
    rt, clock, kernel, _ = runtime()
    started = clock.now_monotonic_us()
    clock.advance_elapsed_us(40_000)
    query = tracking_at(rt, started=started)
    kernel_read(rt, kernel)
    published = rt.sample_network(query).observation
    (stored,) = round_trip(published)
    assert stored == published
    assert stored.system_time_quality is E.SystemTimeQuality.CHRONY_SYNCED
    assert stored.rtc_evidence is None
    assert (stored.sample_started_at_monotonic_us, stored.sample_finished_at_monotonic_us) == (
        41_000,
        41_300,
    )
    assert stored.sampled_at_monotonic_us == 41_150
    assert stored.network_evidence == row.NetworkClockEvidenceV1(
        started, 41_000, -2_640_071, 38_471, 1_051, 20_286, 6_208, 166,
        0xA29FC87B, UTC - 30_000_000, 4,
    )
    assert stored.error_bound_us == recomputed_network_error(stored)
    assert stored.error_budget_expires_at_monotonic_us == horizon(stored)
    assert stored.error_budget_expires_at_monotonic_us == (
        rt.schedule.trust_expires_at_monotonic_us
    )


# Holdover evidence copies the verification baseline, so stored rows stay reproducible
# after a later verification replaces the communicator's latest provenance.
def test_rtc_observations_keep_their_own_verification_baseline():
    rt, clock, _, _ = runtime()
    first = RtcProvenanceV1(rt.instance, UTC - 3_600_000_000, UTC - 3_600_000_000, 4_000_000, 10)
    second = RtcProvenanceV1(rt.instance, UTC - 60_000_000, UTC - 59_500_000, 2_500_000, 10)
    rtc = FakeDs3231Control()
    with_provenance(rt, first)
    rtc_read(rt, rtc, UTC // 1_000_000)
    old = rt.observe_rtc(rtc).observation
    with_provenance(rt, second)
    clock.advance_elapsed_us(5_000_000)
    rtc_read(rt, rtc, UTC // 1_000_000 + 5)
    new = rt.observe_rtc(rtc).observation
    stored = round_trip(old, new)
    assert stored == [old, new]
    for observation, provenance in zip(stored, (first, second), strict=True):
        assert observation.system_time_quality is E.SystemTimeQuality.RTC_HOLDOVER
        assert observation.network_evidence is None
        evidence = observation.rtc_evidence
        assert (
            evidence.reference_receiver_instance_id,
            evidence.reference_verified_at_utc_us,
            evidence.reference_readback_utc_us,
            evidence.reference_uncertainty_us,
            evidence.reference_drift_bound_ppm,
        ) == (
            provenance.verified_by_receiver_instance_id,
            provenance.network_utc_at_verification_us,
            provenance.rtc_readback_utc_us,
            provenance.verification_uncertainty_us,
            provenance.drift_bound_ppm,
        )
        assert observation.error_bound_us == recomputed_rtc_error(observation)
        assert observation.error_budget_expires_at_monotonic_us == horizon(observation)
    assert stored[0].rtc_evidence.drift_contribution_us == 36_001  # ceil(3600 s x 10 ppm)
    assert stored[1].rtc_evidence.drift_contribution_us == 646  # ceil(64.5 s x 10 ppm)


# After a reboot the startup read charges drift over the whole powered-off age of a
# baseline verified by an earlier receiver instance; nothing resets that age.
def test_aged_baseline_after_reboot():
    rt, _, _, _ = runtime()
    age_us = 30 * 86_400_000_000
    readback = UTC - age_us
    with_provenance(rt, RtcProvenanceV1(OTHER_INSTANCE, readback, readback, 4_000_000, 10))
    (stored,) = round_trip(rt.observe_rtc(None, startup=True).observation)
    assert stored.system_time_quality is E.SystemTimeQuality.RTC_HOLDOVER
    assert stored.rtc_evidence.reference_receiver_instance_id == OTHER_INSTANCE
    assert stored.receiver_instance_id != OTHER_INSTANCE
    assert stored.rtc_evidence.drift_contribution_us == 25_920_260  # ceil(30 d x 10 ppm)
    assert (stored.sample_started_at_monotonic_us, stored.sample_finished_at_monotonic_us) == (0, 100)
    assert stored.error_bound_us == recomputed_rtc_error(stored)


# Boundaries carry no sample: no bound, bracket, horizon or source evidence, and an
# explicit step remains distinguishable only through its existing flag.
def test_untrusted_boundaries_have_no_evidence():
    def synced():
        rt, clock, kernel, _ = runtime()
        chrony = FakeChronyControl()
        kernel_read(rt, kernel)
        chrony.tracking_results.append(
            lambda: tracking_at(rt, started=rt.clock.now_monotonic_us())
        )
        assert rt.poll_chrony(chrony).observation.network_evidence is not None
        return rt, clock, chrony

    rt, clock, chrony = synced()
    clock.advance_elapsed_us(1_000_000)
    chrony.tracking_results.append(
        lambda: tracking_at(rt, started=rt.clock.now_monotonic_us(), correction=50_000_000)
    )
    step = rt.poll_chrony(chrony).observation
    rt, clock, _ = synced()
    clock.advance_elapsed_us(POLICY.chrony_tracking_poll_period_cap_us)
    expiry = rt.expire_due().observation
    assert step.step_discontinuity_boundary and not expiry.step_discontinuity_boundary
    for observation in (*round_trip(step), *round_trip(expiry)):
        assert observation.system_time_quality is E.SystemTimeQuality.UNTRUSTED
        assert (
            observation.sampled_at_utc_us,
            observation.error_bound_us,
            observation.sample_started_at_monotonic_us,
            observation.sample_finished_at_monotonic_us,
            observation.error_budget_expires_at_monotonic_us,
            observation.network_evidence,
            observation.rtc_evidence,
        ) == (None,) * 7


# Evidence capture performs no extra source operations: one poll is one tracking
# query, one kernel read and one observation; one holdover is one RTC read.
def test_capture_adds_no_operations_or_emissions():
    rt, _, kernel, queue = runtime()
    chrony = FakeChronyControl()
    kernel_read(rt, kernel)
    chrony.tracking_results.append(lambda: tracking_at(rt, started=rt.clock.now_monotonic_us()))
    rt.poll_chrony(chrony)
    assert [call[0] for call in chrony.calls] == ["tracking"]
    assert len(kernel.deadlines) == 1 and queue.snapshot().published_entities == 1

    rt, _, _, queue = runtime()
    with_provenance(rt, RtcProvenanceV1(rt.instance, UTC, UTC, 4_000_000, 10))
    rtc = FakeDs3231Control()
    rtc_read(rt, rtc, UTC // 1_000_000)
    rt.observe_rtc(rtc)
    assert [call[0] for call in rtc.calls] == ["read"]
    assert queue.snapshot().published_entities == 1


NETWORK_FACTS = row.NetworkClockEvidenceV1(0, 10, 0, 0, 0, 0, 0, 0, 1, 0, 2)
RTC_FACTS = row.RtcClockEvidenceV1(b"v" * 16, 0, 0, 1, 10, 0)


# Evidence must name exactly the source that produced the sample's quality.
@pytest.mark.parametrize(
    "network,rtc,quality",
    [
        (None, None, E.SystemTimeQuality.CHRONY_SYNCED),
        (NETWORK_FACTS, RTC_FACTS, E.SystemTimeQuality.CHRONY_SYNCED),
        (NETWORK_FACTS, None, E.SystemTimeQuality.RTC_HOLDOVER),
        (None, RTC_FACTS, E.SystemTimeQuality.CHRONY_SYNCED),
    ],
)
def test_sample_evidence_matches_its_source(network, rtc, quality):
    with pytest.raises(ValueError):
        TrustedTimeSample(
            20, UTC, 1_000, quality, 1, SampleEvidence(10, 30, network=network, rtc=rtc)
        )


# A trusted row is never published without the evidence of its bound.
def test_trusted_publication_requires_evidence():
    sample = TrustedTimeSample(20, UTC, 1_000, E.SystemTimeQuality.CHRONY_SYNCED, 1)
    schedule = observation_schedule(sample, POLICY, tracking_started_at_monotonic_us=0)
    with pytest.raises(ValueError):
        trusted_clock_observation(
            b"i" * 16, 1, sample, schedule, generation=1, rtc_health=E.RtcHealth.PRESENT
        )
