from __future__ import annotations

import pytest

from deployment.jaka_mini2.runtime.hand_presets import HandPreset
from deployment.jaka_mini2.runtime.hand_presets import HandPresetState


def test_uncalibrated_preset_is_rejected() -> None:
    state = HandPresetState({"1": HandPreset("1", "open", None)})
    with pytest.raises(ValueError, match="not calibrated"):
        state.select("1")


def test_space_latches_current_o6_state() -> None:
    state = HandPresetState({})
    selected = state.select("space", current_state=(1, 2, 3, 4, 5, 6))
    assert selected.target == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert state.active_key == "space"


def test_calibrated_target_remains_active_until_next_event() -> None:
    grasp = HandPreset("2", "grasp_cup", (10, 20, 30, 40, 50, 60))
    state = HandPresetState({"2": grasp})
    selected = state.select("2")
    assert selected.target == (10, 20, 30, 40, 50, 60)
    assert state.active is selected
    assert state.event_index == 1
