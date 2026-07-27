"""
Tests for the settings.py user-overrides overlay (M12).

Reloads settings with a monkeypatched settings_store.load_user_overrides
and checks that whitelisted keys pick up the override while everything
else keeps its committed value. Teardown restores by re-reloading with
the real loader.

NOTE: on a machine where user_settings.json exists (e.g. this rig stores
SERIAL_PORT "COM8"), the post-teardown value is the real overlay value,
not the committed default — so the restore assertions compare against
settings_store.load_user_overrides() content, never hardcoded defaults.
"""
import importlib

import pytest

import settings_store


def test_overlay_applies_and_reload_restores(monkeypatch: pytest.MonkeyPatch) -> None:
    import settings

    monkeypatch.setattr(
        settings_store,
        "load_user_overrides",
        lambda path=None: {"SERIAL_PORT": "COM99", "MANUAL_ROLL_TRIM_DEG": 2.5},
    )
    try:
        importlib.reload(settings)
        assert settings.SERIAL_PORT == "COM99"
        assert settings.MANUAL_ROLL_TRIM_DEG == 2.5
        # Non-overridable keys are untouched by the overlay.
        assert settings.SERIAL_BAUD == 250000  # firmware v2 default
    finally:
        monkeypatch.undo()
        importlib.reload(settings)

    # Restored values reflect whatever the real overlay file holds.
    real = settings_store.load_user_overrides()
    assert settings.SERIAL_PORT == str(real.get("SERIAL_PORT", "COM4"))
    assert settings.MANUAL_ROLL_TRIM_DEG == float(
        real.get("MANUAL_ROLL_TRIM_DEG", 0.0)
    )
    assert settings.SERIAL_BAUD == 250000  # firmware v2 default


def test_autotune_calibration_keys_overlay(monkeypatch: pytest.MonkeyPatch) -> None:
    """SysID rework: an autotune Apply persists the plant calibration
    (prediction horizon, integral deadband, effective gravity) via the
    overlay — settings.py must pick all three up on reload."""
    import settings

    monkeypatch.setattr(
        settings_store,
        "load_user_overrides",
        lambda path=None: {
            "CONTROL_PREDICT_S": 0.11,
            "PD_I_ERR_DEADBAND_MM": 3.5,
            "PD_AUTOTUNE_G_EFF": 155.0,
        },
    )
    try:
        importlib.reload(settings)
        assert settings.CONTROL_PREDICT_S == 0.11
        assert settings.PD_I_ERR_DEADBAND_MM == 3.5
        assert settings.PD_AUTOTUNE_G_EFF == 155.0
    finally:
        monkeypatch.undo()
        importlib.reload(settings)

    # Restored values reflect whatever the real overlay file holds.
    real = settings_store.load_user_overrides()
    assert settings.CONTROL_PREDICT_S == float(
        real.get("CONTROL_PREDICT_S", 0.08)
    )
    assert settings.PD_I_ERR_DEADBAND_MM == float(
        real.get("PD_I_ERR_DEADBAND_MM", 2.0)
    )
    assert settings.PD_AUTOTUNE_G_EFF == float(
        real.get("PD_AUTOTUNE_G_EFF", 171.0)
    )
