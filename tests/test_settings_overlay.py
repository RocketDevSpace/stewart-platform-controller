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
