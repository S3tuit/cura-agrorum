from dataclasses import replace

import pytest

from cura_receiver.elapsed_duration import (
    rate_growth_us,
    minimum_wait_monotonic_us,
    maximum_lifetime_monotonic_us,
    checked_monotonic_deadline,
)
from cura_receiver.generated.receiver_entities_generated import RtcProvenanceV1
from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth as Health,
    SystemTimeQuality as Quality,
)
from cura_receiver.time_observations import (
    TrustedTimeSample,
    advanced_error_us,
    network_observation,
    observation_schedule,
    rtc_observation,
    rtc_provenance_candidate,
    rtc_read_uncertainty_us,
    rtc_refresh_due_us,
    rtc_refresh_source_error_us,
)
from cura_receiver.time_policy import (
    ClockState,
    NetworkEvidence,
    TimePolicy,
    advance_clock_state,
    expire_clock_trust,
)
from tests.support.fakes.os_clock import FakeOsClock

POLICY = TimePolicy(maximum_network_skew_ppb=1000)
STATE = ClockState(Quality.NETWORK_SYNCED, Health.PRESENT, 7)
EVIDENCE = NetworkEvidence(0, 100, True, True, 0, 0, 1000)
PROVENANCE = RtcProvenanceV1(b"v" * 16, 500_000, 500_000, 4_000_000, 10)


def network(**overrides):
    args = dict(
        before=STATE,
        after=STATE,
        evidence=EVIDENCE,
        operation_started_at_monotonic_us=100,
        operation_finished_at_monotonic_us=200,
        sampled_utc_us=0,
        kernel_sample_usable=True,
        required_poll_deadline_us=60_000_000,
        policy=POLICY,
    )
    args.update(overrides)
    return network_observation(**args)


def rtc(**overrides):
    args = dict(
        before=STATE,
        after=STATE,
        provenance=PROVENANCE,
        rtc_health=Health.PRESENT,
        rtc_utc_seconds=0,
        operation_started_at_monotonic_us=100,
        operation_finished_at_monotonic_us=100,
        policy=POLICY,
    )
    args.update(overrides)
    return rtc_observation(**args)


def verification(**overrides):
    args = dict(
        sample=TrustedTimeSample(0, 500_000, 1_000_000, Quality.NETWORK_SYNCED, 7),
        current=STATE,
        verified_by_receiver_instance_id=b"v" * 16,
        episode_started_at_monotonic_us=0,
        read_started_at_monotonic_us=0,
        read_finished_at_monotonic_us=0,
        commit_at_monotonic_us=0,
        rtc_utc_seconds=0,
        required_poll_deadline_us=60_000_000,
        policy=POLICY,
    )
    args.update(overrides)
    return rtc_provenance_candidate(**args)


# A bounded network bracket produces its midpoint and preserves a valid UTC zero.
def test_network_midpoint_and_zero_utc():
    sample = network()
    assert sample == TrustedTimeSample(150, 0, 1_000_000, Quality.NETWORK_SYNCED, 7)
    assert network(operation_finished_at_monotonic_us=201).monotonic_us == 150


# The complete tracking-to-sampling span, every ordering edge, kernel verdict and generation must agree.
@pytest.mark.parametrize(
    "overrides",
    [
        {"after": replace(STATE, generation=9)},
        {"kernel_sample_usable": False},
        {"kernel_sample_usable": 1},
        {"operation_started_at_monotonic_us": 99},
        {"operation_started_at_monotonic_us": 201},
        {"operation_finished_at_monotonic_us": 1_000_001},
        {"required_poll_deadline_us": 200},
        {"sampled_utc_us": 1 << 63},
        {"evidence": None},
        {"evidence": replace(EVIDENCE, synchronized=False)},
        {"evidence": replace(EVIDENCE, remaining_correction_us=39_000_000)},
    ],
)
def test_network_sample_rejections(overrides):
    assert network(**overrides) is None


# A generation-changing away-and-back transition invalidates both kinds of sampled clock evidence.
def test_sampling_rejects_quality_aba():
    lost = expire_clock_trust(STATE)
    restored = advance_clock_state(
        lost, quality=STATE.quality, rtc_health=STATE.rtc_health
    )
    assert network(after=restored) is None
    assert rtc(after=restored) is None


# Network entry is permitted with a missing RTC; a hysteresis-band candidate cannot promote untrusted time.
def test_network_axes_and_hysteresis():
    missing = replace(STATE, rtc_health=Health.MISSING)
    assert network(before=missing, after=missing) is not None
    untrusted = replace(STATE, quality=Quality.UNTRUSTED)
    evidence = replace(EVIDENCE, remaining_correction_us=34_000_001)
    assert network(before=untrusted, after=untrusted, evidence=evidence) is None
    assert network(evidence=evidence).error_bound_us == 35_000_001
    assert network(operation_finished_at_monotonic_us=1_000_000) is not None


# Direct RTC reads combine provenance, pre-read drift and measured whole-second uncertainty exactly.
def test_rtc_error_terms_and_persisted_drift():
    sample = rtc(rtc_utc_seconds=100_000, operation_finished_at_monotonic_us=1_000_100)
    assert sample.monotonic_us == 500_100
    assert sample.utc_us == 100_000_500_000
    assert sample.error_bound_us == 7_001_868
    changed_policy = replace(POLICY, rtc_drift_bound_ppm=999)
    assert (
        rtc(policy=changed_policy, rtc_utc_seconds=100_000).error_bound_us == 6_500_011
    )
    assert rtc_read_uncertainty_us(100, 100, POLICY) == 1_500_000
    assert rtc_read_uncertainty_us(100, 101, POLICY) == 1_500_001


# Holdover requires usable durable provenance and a fresh present RTC without a backward age or wide bracket.
@pytest.mark.parametrize(
    "overrides",
    [
        {"provenance": None},
        {"rtc_health": Health.MISSING},
        {"rtc_health": Health.INVALID},
        {"rtc_utc_seconds": -1},
        {"rtc_utc_seconds": 1 << 63},
        {"operation_finished_at_monotonic_us": 99},
        {"operation_finished_at_monotonic_us": 1_000_101},
        {"provenance": replace(PROVENANCE, drift_bound_ppm=0)},
        {"provenance": replace(PROVENANCE, drift_bound_ppm=1_000_000)},
        {"provenance": replace(PROVENANCE, verification_uncertainty_us=40_000_000)},
        {"provenance": replace(PROVENANCE, verification_uncertainty_us=(1 << 64) - 1)},
        {"provenance": replace(PROVENANCE, verified_by_receiver_instance_id=b"bad")},
        {"provenance": replace(PROVENANCE, network_utc_at_verification_us=1_500_001)},
    ],
)
def test_rtc_rejections(overrides):
    assert rtc(**overrides) is None


# Exact abstract age boundaries precede checks at the RTC's actually representable whole-second ages.
@pytest.mark.parametrize(
    "post_error,last_age", [(0, 3_449_965_400_001), (13_369_468, 2_113_031_969_469)]
)
def test_documented_holdover_age_boundaries(post_error, last_age):
    assert 5_500_000 + rate_growth_us(10, last_age) + post_error == 39_999_999
    assert 5_500_000 + rate_growth_us(10, last_age + 1) + post_error == 40_000_000
    seconds = last_age // 1_000_000
    sample = rtc(rtc_utc_seconds=seconds)
    assert sample is not None
    if post_error:
        assert (
            advanced_error_us(sample, sample.monotonic_us + 3_600_000_000, POLICY)
            < 40_000_000
        )
        next_sample = rtc(rtc_utc_seconds=seconds + 1)
        assert (
            advanced_error_us(
                next_sample, next_sample.monotonic_us + 3_600_000_000, POLICY
            )
            >= 40_000_000
        )
    else:
        assert rtc(rtc_utc_seconds=seconds + 1) is None


# A nonzero read bracket shortens both age limits and repeated reads never reset provenance age.
def test_holdover_bracket_and_reread_age():
    assert rtc(rtc_utc_seconds=3_449_965) is not None
    assert (
        rtc(rtc_utc_seconds=3_449_965, operation_finished_at_monotonic_us=1_000_100)
        is None
    )
    first = rtc(rtc_utc_seconds=100_000)
    later = rtc(rtc_utc_seconds=200_000)
    assert later.error_bound_us > first.error_bound_us
    assert PROVENANCE.rtc_readback_utc_us == 500_000


# Only monotonic-rate error grows after a direct read, including backward event correlation.
def test_post_read_growth_has_no_rtc_component():
    sample = rtc()
    for policy in (POLICY, replace(POLICY, rtc_drift_bound_ppm=999)):
        assert (
            advanced_error_us(sample, sample.monotonic_us + 3_600_000_000, policy)
            == 18_869_468
        )
    shifted = replace(sample, monotonic_us=3_600_000_100)
    assert advanced_error_us(shifted, 100, POLICY) == 18_869_468


# Observation and polling caps remain at zero rate and shorten when the strict error horizon wins.
def test_observation_and_poll_deadlines():
    zero_rate = replace(
        POLICY, chrony_max_slew_rate_ppm=0, monotonic_elapsed_rate_bound_ppm=0
    )
    sample = TrustedTimeSample(100, 0, 1_000_000, Quality.NETWORK_SYNCED, 7)
    schedule = observation_schedule(
        sample, zero_rate, tracking_started_at_monotonic_us=100
    )
    assert schedule.trust_expires_at_monotonic_us is None
    assert schedule.observation_due_at_monotonic_us == 10_800_000_100
    assert schedule.tracking_poll_due_at_monotonic_us == 60_000_100
    holdover = observation_schedule(
        replace(sample, quality=Quality.RTC_HOLDOVER), zero_rate
    )
    assert holdover.observation_due_at_monotonic_us == 3_600_000_100
    assert holdover.tracking_poll_due_at_monotonic_us is None
    expiring = observation_schedule(
        replace(sample, error_bound_us=39_999_999),
        POLICY,
        tracking_started_at_monotonic_us=100,
    )
    assert expiring.trust_expires_at_monotonic_us == 101
    assert expiring.observation_due_at_monotonic_us == 101
    assert expiring.tracking_poll_due_at_monotonic_us == 101


# Unusable error horizons and absolute deadline overflow never produce a trusted schedule.
@pytest.mark.parametrize(
    "sample",
    [
        TrustedTimeSample(0, 0, 40_000_000, Quality.NETWORK_SYNCED, 0),
        TrustedTimeSample((1 << 64) - 1, 0, 1_000_000, Quality.NETWORK_SYNCED, 0),
    ],
)
def test_schedule_rejections(sample):
    with pytest.raises((ValueError, OverflowError)):
        observation_schedule(sample, POLICY, tracking_started_at_monotonic_us=0)


# Fresh stable samples trigger the initial refresh; later refresh caps are bounded by trust expiry.
def test_rtc_refresh_due_times():
    sample = TrustedTimeSample(100, 0, 1_000_000, Quality.NETWORK_SYNCED, 7)
    zero_rate = replace(
        POLICY, chrony_max_slew_rate_ppm=0, monotonic_elapsed_rate_bound_ppm=0
    )
    assert (
        rtc_refresh_due_us(sample, last_refresh_monotonic_us=None, policy=POLICY) == 100
    )
    assert (
        rtc_refresh_due_us(sample, last_refresh_monotonic_us=0, policy=zero_rate)
        == 10_800_000_000
    )
    assert (
        rtc_refresh_due_us(sample, last_refresh_monotonic_us=0, policy=POLICY)
        == observation_schedule(
            sample, POLICY, tracking_started_at_monotonic_us=0
        ).trust_expires_at_monotonic_us
    )
    assert (
        rtc_refresh_due_us(
            replace(sample, error_bound_us=5_000_001),
            last_refresh_monotonic_us=None,
            policy=POLICY,
        )
        is None
    )


# F-001: refresh completion can follow its supporting sample without reversing a clock bracket.
@pytest.mark.parametrize("completion", [99, 100, 101])
def test_refresh_completion_relative_to_supporting_sample(completion):
    sample = TrustedTimeSample(100, 0, 1_000_000, Quality.NETWORK_SYNCED, 7)
    zero_rate = replace(
        POLICY, chrony_max_slew_rate_ppm=0, monotonic_elapsed_rate_bound_ppm=0
    )
    assert rtc_refresh_due_us(
        sample, last_refresh_monotonic_us=completion, policy=zero_rate
    ) == completion + 10_800_000_000


# The stricter five-second threshold is inclusive and expires on the first added error unit.
def test_rtc_source_threshold():
    sample = TrustedTimeSample(0, 500_000, 5_000_000, Quality.NETWORK_SYNCED, 7)
    assert (
        rtc_refresh_source_error_us(
            sample,
            STATE,
            now_monotonic_us=0,
            required_poll_deadline_us=100,
            policy=POLICY,
        )
        == 5_000_000
    )
    assert (
        rtc_refresh_source_error_us(
            sample,
            STATE,
            now_monotonic_us=1,
            required_poll_deadline_us=100,
            policy=POLICY,
        )
        is None
    )
    assert verification(sample=sample) is not None
    assert verification(sample=sample, commit_at_monotonic_us=1) is None
    assert verification(sample=replace(sample, error_bound_us=5_000_001)) is None


# A provenance proposal charges actual read-back difference, advanced source error and read uncertainty.
def test_verification_exact_terms():
    candidate = verification(
        read_started_at_monotonic_us=100_000,
        read_finished_at_monotonic_us=300_000,
        commit_at_monotonic_us=300_000,
    )
    assert candidate == RtcProvenanceV1(b"v" * 16, 700_000, 500_000, 2_801_115, 10)
    assert verification().verification_uncertainty_us == 2_500_000


# Accepted read-back equality is inclusive, while later commit/generation failures establish no proposal.
@pytest.mark.parametrize(
    "overrides",
    [
        {
            "sample": TrustedTimeSample(
                0, 1_500_001, 1_000_000, Quality.NETWORK_SYNCED, 7
            )
        },
        {"current": replace(STATE, generation=9)},
        {"current": replace(STATE, quality=Quality.UNTRUSTED)},
        {"required_poll_deadline_us": 0},
        {"read_started_at_monotonic_us": 1},
        {
            "read_finished_at_monotonic_us": 1_000_001,
            "commit_at_monotonic_us": 1_000_001,
        },
        {"verified_by_receiver_instance_id": b"bad"},
        {"policy": replace(POLICY, receiver_utc_error_budget_us=2_500_000)},
    ],
)
def test_verification_rejections(overrides):
    assert verification(**overrides) is None


# Read-back tolerance charges exactly the observed discrepancy at its inclusive one-second boundary.
def test_verification_difference_equality():
    sample = TrustedTimeSample(0, 1_500_000, 1_000_000, Quality.NETWORK_SYNCED, 7)
    assert verification(sample=sample).verification_uncertainty_us == 3_500_000


# Manual realtime jumps leave monotonic retry waits, lifetimes and event intervals unchanged.
def test_realtime_step_immunity():
    clock = FakeOsClock(monotonic_us=100, realtime_us=1_000_000)
    retry = checked_monotonic_deadline(
        clock.now_monotonic_us(), minimum_wait_monotonic_us(250_000)
    )
    deadline = checked_monotonic_deadline(
        clock.now_monotonic_us(), maximum_lifetime_monotonic_us(30_000_000)
    )
    for delta in (86_400_000_000, -172_800_000_000):
        clock.step_realtime_us(delta)
        assert clock.now_monotonic_us() == 100
        assert retry == 251_025
        assert deadline == 29_889_100
    clock.advance_elapsed_us(251_000)
    assert clock.now_monotonic_us() - 100 == 251_000
    assert clock.now_monotonic_us() > retry


# A slow tracking/sampling episode cannot shift the next one-minute poll later than its query-start cap.
def test_poll_cap_uses_tracking_query_start():
    sample = TrustedTimeSample(750_000, 0, 1_000_000, Quality.NETWORK_SYNCED, 7)
    schedule = observation_schedule(sample, POLICY, tracking_started_at_monotonic_us=0)
    assert schedule.tracking_poll_due_at_monotonic_us == 60_000_000
    with pytest.raises(ValueError):
        observation_schedule(sample, POLICY)
    with pytest.raises(ValueError):
        observation_schedule(sample, POLICY, tracking_started_at_monotonic_us=750_001)


# An otherwise bounded candidate is unusable when its interval expires before sampling finishes.
def test_sampling_must_finish_before_trust_expiry():
    evidence = replace(EVIDENCE, remaining_correction_us=38_999_999)
    assert (
        network(evidence=evidence, operation_finished_at_monotonic_us=100) is not None
    )
    assert network(evidence=evidence, operation_finished_at_monotonic_us=101) is None
    provenance = replace(PROVENANCE, verification_uncertainty_us=4_000_003)
    assert rtc(provenance=provenance, rtc_utc_seconds=3_449_965) is not None
    assert (
        rtc(
            provenance=provenance,
            rtc_utc_seconds=3_449_965,
            operation_finished_at_monotonic_us=101,
        )
        is None
    )


# A refresh cannot be due from a sample already outside a tighter configured UTC budget.
def test_refresh_rejects_exhausted_budget():
    sample = TrustedTimeSample(0, 0, 2_000_000, Quality.NETWORK_SYNCED, 7)
    assert (
        rtc_refresh_due_us(
            sample,
            last_refresh_monotonic_us=None,
            policy=replace(POLICY, receiver_utc_error_budget_us=2_000_000),
        )
        is None
    )
