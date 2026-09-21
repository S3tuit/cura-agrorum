from dataclasses import replace

import pytest

from cura_receiver.generated.receiver_enums_generated import (
    RtcHealth as Health,
    SystemTimeQuality as Quality,
)
from cura_receiver.time_policy import (
    ClockState,
    NetworkEvidence,
    TimePolicy,
    advance_clock_state,
    expire_clock_trust,
    network_error_bound_us,
    network_estimate,
    network_root_distance_us,
    network_tracking_decision,
    startup_clock_state,
    usable_network_error_us,
)

POLICY = TimePolicy()
EVIDENCE = NetworkEvidence(100, 200, True, True, 0, 0, 1000)


# Both non-holdover qualities work with every RTC health, including synchronized time without an RTC.
@pytest.mark.parametrize("quality,health", [(q, h) for q in Quality for h in Health])
def test_independent_quality_axes(quality, health):
    if quality is Quality.RTC_HOLDOVER and health is not Health.PRESENT:
        with pytest.raises(ValueError):
            ClockState(quality, health, 0)
    else:
        assert ClockState(quality, health, 0).rtc_health is health


# Startup uses the current probe and cannot promote any previously observed quality/health snapshot.
@pytest.mark.parametrize(
    "old_quality,old_health,probed",
    [(q, h, p) for q in Quality for h in Health for p in Health],
)
def test_startup_ignores_last_observed_snapshots(old_quality, old_health, probed):
    persisted_snapshot = (old_quality, old_health)
    assert startup_clock_state(probed) == ClockState(Quality.UNTRUSTED, probed, 0)
    with pytest.raises(TypeError):
        startup_clock_state(persisted_snapshot)


# The skew ceiling is removed and the pilot sampling margin remains fixed.
def test_error_policy_and_fixed_margin():
    assert TimePolicy() == POLICY
    with pytest.raises(TypeError):
        TimePolicy(maximum_network_skew_ppb=1000)
    with pytest.raises(TypeError):
        TimePolicy(time_sampling_margin_us=2)
    assert POLICY.time_sampling_margin_us == 1_000_000


# Configuration rejects incompatible thresholds, invalid rates, empty periods and noninteger values.
@pytest.mark.parametrize(
    "overrides",
    [
        {"monotonic_elapsed_rate_bound_ppm": -1},
        {"monotonic_elapsed_rate_bound_ppm": True},
        {"chrony_max_slew_rate_ppm": 3701},
        {"monotonic_elapsed_rate_bound_ppm": 1_000_000},
        {"rtc_drift_bound_ppm": 0},
        {"rtc_drift_bound_ppm": 1_000_000},
        {"network_rtc_write_error_threshold_us": 1_000_000},
        {"network_trust_error_threshold_us": 40_000_000},
        {"receiver_utc_error_budget_us": 0},
        {"receiver_utc_error_budget_us": 40_000_001},
        {"chrony_tracking_poll_period_cap_us": 0},
        {"clock_observation_period_cap_us": 0},
        {"rtc_holdover_observation_period_cap_us": 0},
        {"network_rtc_refresh_period_us": 0},
    ],
)
def test_invalid_configuration(overrides):
    with pytest.raises((TypeError, ValueError, OverflowError)):
        replace(POLICY, **overrides)


# Root distance rounds odd half-delays upward and correction signs never cancel source uncertainty.
@pytest.mark.parametrize("correction", [-3_000_000, 3_000_000])
def test_network_error_terms(correction):
    assert network_root_distance_us(3, 7) == 9
    assert network_error_bound_us(correction, 2_000_000) == 6_000_000


# Invalid unsigned terms and signed absolute overflow cannot produce a trustworthy network bound.
@pytest.mark.parametrize(
    "function,args",
    [
        (network_root_distance_us, (-1, 0)),
        (network_root_distance_us, (0, -1)),
        (network_root_distance_us, ((1 << 64) - 1, (1 << 64) - 1)),
        (network_error_bound_us, (-(1 << 63), 0)),
        (network_error_bound_us, (0, (1 << 64) - 1)),
    ],
)
def test_network_arithmetic_rejections(function, args):
    with pytest.raises(OverflowError):
        function(*args)


# Entry and hysteresis thresholds apply to complete error, independently from the observation budget.
@pytest.mark.parametrize("quality", list(Quality))
@pytest.mark.parametrize(
    "error", [34_999_999, 35_000_000, 35_000_001, 39_999_999, 40_000_000, 40_000_001]
)
def test_network_hysteresis(quality, error):
    evidence = replace(EVIDENCE, remaining_correction_us=error - 1_000_001)
    decision = network_tracking_decision(
        ClockState(quality, Health.PRESENT, 5),
        network_estimate(evidence),
        now_monotonic_us=200,
        required_poll_deadline_us=60_000_100,
        policy=POLICY,
    )
    expected = (
        Quality.NETWORK_SYNCED
        if error <= 35_000_000
        else Quality.UNTRUSTED if error > 40_000_000 else quality
    )
    assert decision.candidate_quality is expected
    assert decision.error_bound_us == error
    assert decision.step_required is (error > 40_000_000)


# Unavailable, unselected, unsynchronized, unreliable, malformed and overflowing evidence is unusable.
@pytest.mark.parametrize(
    "evidence",
    [
        None,
        object(),
        replace(EVIDENCE, source_selected=False),
        replace(EVIDENCE, synchronized=False),
        replace(EVIDENCE, synchronized=1),
        replace(EVIDENCE, estimated_skew_ppb=True),
        replace(EVIDENCE, estimated_skew_ppb=-1),
        replace(EVIDENCE, estimated_skew_ppb=1 << 64),
        replace(EVIDENCE, root_distance_us=-1),
        replace(EVIDENCE, root_distance_us=1 << 64),
        replace(EVIDENCE, remaining_correction_us=-(1 << 63)),
        replace(EVIDENCE, sample_started_at_monotonic_us=201),
        replace(EVIDENCE, sample_finished_at_monotonic_us=301),
    ],
)
def test_unusable_network_evidence(evidence):
    decision = network_tracking_decision(
        ClockState(Quality.NETWORK_SYNCED, Health.MISSING, 5),
        network_estimate(evidence),
        now_monotonic_us=300,
        required_poll_deadline_us=60_000_100,
        policy=POLICY,
    )
    assert decision.candidate_quality is Quality.UNTRUSTED
    assert decision.error_bound_us is None
    assert not decision.step_required


# Sampling freshness includes the whole tracking-start span and the next poll deadline is exclusive.
@pytest.mark.parametrize(
    "now,deadline,expected",
    [
        (1_000_100, 2_000_000, 1_003_714),
        (1_000_101, 2_000_000, None),
        (300, 300, None),
        (300, 301, 1_000_001),
    ],
)
def test_network_freshness_boundaries(now, deadline, expected):
    assert (
        usable_network_error_us(
            network_estimate(EVIDENCE),
            now_monotonic_us=now,
            required_poll_deadline_us=deadline,
            policy=POLICY,
        )
        == expected
    )


# A completed tracking input advances once even without enum changes; one combined update is atomic.
def test_atomic_generation_updates():
    before = ClockState(Quality.NETWORK_SYNCED, Health.PRESENT, 7)
    periodic = advance_clock_state(
        before, quality=before.quality, rtc_health=before.rtc_health
    )
    assert periodic == before
    tracking = advance_clock_state(
        before,
        quality=before.quality,
        rtc_health=before.rtc_health,
        tracking_processed=True,
    )
    assert tracking.generation == 8
    changed = advance_clock_state(
        before,
        quality=Quality.UNTRUSTED,
        rtc_health=Health.MISSING,
        tracking_processed=True,
        step_boundary=True,
    )
    assert changed.generation == 8


# An away-and-back quality transition invalidates a captured generation despite equal final quality.
def test_quality_aba_generation():
    before = ClockState(Quality.NETWORK_SYNCED, Health.PRESENT, 7)
    lost = expire_clock_trust(before)
    restored = advance_clock_state(
        lost, quality=before.quality, rtc_health=before.rtc_health
    )
    assert restored.quality is before.quality
    assert restored.generation == 9
    assert restored.generation != before.generation


# Generation exhaustion and a trusted step-boundary request cannot silently change state.
def test_generation_failures():
    state = ClockState(Quality.NETWORK_SYNCED, Health.PRESENT, (1 << 63) - 1)
    with pytest.raises(OverflowError):
        expire_clock_trust(state)
    with pytest.raises(ValueError):
        advance_clock_state(
            state,
            quality=state.quality,
            rtc_health=state.rtc_health,
            step_boundary=True,
        )


# Skew above the former 10 ppm ceiling no longer delays an otherwise bounded source.
@pytest.mark.parametrize("skew", [10_001, 250_000, 3_500_000])
def test_skew_is_diagnostic_while_total_error_controls_admission(skew):
    estimate = network_estimate(replace(EVIDENCE, estimated_skew_ppb=skew))
    decision = network_tracking_decision(
        ClockState(Quality.UNTRUSTED, Health.PRESENT, 0), estimate,
        now_monotonic_us=300, required_poll_deadline_us=60_000_100, policy=POLICY,
    )
    assert decision.candidate_quality is Quality.NETWORK_SYNCED
    assert decision.error_bound_us == 1_000_001


# Query duration consumes the same error budget as delay after the query completes.
def test_query_age_crosses_entry_threshold_without_a_new_initial_sum():
    estimate = network_estimate(replace(EVIDENCE, remaining_correction_us=33_999_000))
    state = ClockState(Quality.UNTRUSTED, Health.PRESENT, 0)
    first = network_tracking_decision(state, estimate, now_monotonic_us=200,
        required_poll_deadline_us=60_000_100, policy=POLICY)
    aged = network_tracking_decision(state, estimate, now_monotonic_us=1_000_100,
        required_poll_deadline_us=60_000_100, policy=POLICY)
    assert first.error_bound_us == 34_999_001
    assert first.candidate_quality is Quality.NETWORK_SYNCED
    assert aged.error_bound_us == 35_002_714
    assert aged.candidate_quality is Quality.UNTRUSTED
