"""Automated installed-Pi time mutations using shared isolated fixtures."""

import os

import pytest

from tests.support.time_components import component, fixture

pytestmark = [pytest.mark.hardware, pytest.mark.destructive]


# The installed unprivileged component verifies RTC refresh and helper privileges.
@pytest.mark.parametrize("fixture", [0], indirect=True)
def test_network_rtc_refresh_and_helper_privilege(fixture):
    component(fixture, "rtc")


# Explicit clock steps verify forward/backward correlation handling.
@pytest.mark.parametrize("fixture", [60, -60], indirect=True)
def test_forward_backward_step_component(fixture):
    component(fixture, "step")


# Maximum slew uses the independent laptop reference bridge.
@pytest.mark.slow
@pytest.mark.parametrize("fixture", [120, -120], indirect=True)
def test_maximum_slew_against_laptop(fixture):
    assert (
        os.environ.get("CURA_LAPTOP_REFERENCE") == "SSH_STDIO_V1"
    ), "run through the laptop reference bridge"
    component(fixture, "slew", stream=True)
