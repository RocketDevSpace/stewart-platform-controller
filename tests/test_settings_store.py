"""
Tests for settings_store.py — the user-settings overlay (M12).

Covers: missing/corrupt/non-dict loads, unknown-key filtering on load and
save, merge behavior (saving trims must not clobber SERIAL_PORT), atomic
write (no .tmp left behind), exact float round-trip.
"""
import json
import logging
from pathlib import Path

import pytest

from settings_store import load_user_overrides, save_user_overrides


def _write(p: Path, obj: object) -> None:
    p.write_text(json.dumps(obj), encoding="utf-8")


class TestLoad:
    def test_missing_file_returns_empty(self, tmp_path: Path) -> None:
        assert load_user_overrides(tmp_path / "does_not_exist.json") == {}

    def test_corrupt_json_returns_empty_with_one_warning(
        self, tmp_path: Path, caplog: pytest.LogCaptureFixture
    ) -> None:
        p = tmp_path / "user_settings.json"
        p.write_text("{this is not json", encoding="utf-8")
        with caplog.at_level(logging.WARNING, logger="settings_store"):
            assert load_user_overrides(p) == {}
        warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
        assert len(warnings) == 1

    def test_non_dict_json_returns_empty_with_one_warning(
        self, tmp_path: Path, caplog: pytest.LogCaptureFixture
    ) -> None:
        p = tmp_path / "user_settings.json"
        _write(p, ["SERIAL_PORT", "COM9"])
        with caplog.at_level(logging.WARNING, logger="settings_store"):
            assert load_user_overrides(p) == {}
        warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
        assert len(warnings) == 1

    def test_unknown_keys_dropped_with_warning(
        self, tmp_path: Path, caplog: pytest.LogCaptureFixture
    ) -> None:
        p = tmp_path / "user_settings.json"
        _write(p, {"SERIAL_PORT": "COM9", "TOTALLY_BOGUS_KEY": 42})
        with caplog.at_level(logging.WARNING, logger="settings_store"):
            overrides = load_user_overrides(p)
        assert overrides == {"SERIAL_PORT": "COM9"}
        assert any("TOTALLY_BOGUS_KEY" in r.getMessage() for r in caplog.records)


class TestSave:
    def test_merge_preserves_other_keys(self, tmp_path: Path) -> None:
        p = tmp_path / "user_settings.json"
        _write(p, {"SERIAL_PORT": "COM8", "CAMERA_INDEX": 1})
        save_user_overrides({"MANUAL_ROLL_TRIM_DEG": 1.25}, path=p)
        overrides = load_user_overrides(p)
        assert overrides == {
            "SERIAL_PORT": "COM8",
            "CAMERA_INDEX": 1,
            "MANUAL_ROLL_TRIM_DEG": 1.25,
        }

    def test_save_updates_existing_key(self, tmp_path: Path) -> None:
        p = tmp_path / "user_settings.json"
        _write(p, {"SERIAL_PORT": "COM8"})
        save_user_overrides({"SERIAL_PORT": "COM9"}, path=p)
        assert load_user_overrides(p) == {"SERIAL_PORT": "COM9"}

    def test_unknown_keys_filtered_on_save(
        self, tmp_path: Path, caplog: pytest.LogCaptureFixture
    ) -> None:
        p = tmp_path / "user_settings.json"
        with caplog.at_level(logging.WARNING, logger="settings_store"):
            save_user_overrides({"SERIAL_PORT": "COM9", "NOT_A_KEY": 1}, path=p)
        stored = json.loads(p.read_text(encoding="utf-8"))
        assert stored == {"SERIAL_PORT": "COM9"}
        assert any("NOT_A_KEY" in r.getMessage() for r in caplog.records)

    def test_save_creates_file_when_missing(self, tmp_path: Path) -> None:
        p = tmp_path / "user_settings.json"
        save_user_overrides({"CAMERA_INDEX": 0}, path=p)
        assert load_user_overrides(p) == {"CAMERA_INDEX": 0}

    def test_atomic_write_leaves_no_tmp_file(self, tmp_path: Path) -> None:
        p = tmp_path / "user_settings.json"
        save_user_overrides({"SERIAL_PORT": "COM9"}, path=p)
        leftovers = [f for f in tmp_path.iterdir() if f.name != "user_settings.json"]
        assert leftovers == []

    def test_float_round_trips_exactly(self, tmp_path: Path) -> None:
        p = tmp_path / "user_settings.json"
        value = -0.8131485683893847
        save_user_overrides({"MANUAL_ROLL_TRIM_DEG": value}, path=p)
        loaded = load_user_overrides(p)
        assert loaded["MANUAL_ROLL_TRIM_DEG"] == value
