"""
settings_store.py

User-settings overlay (M12): a small JSON file next to the repo root
(user_settings.json, untracked) holding per-machine overrides for a
whitelisted subset of settings.py keys. settings.py loads the overrides
at import time; the GUI persists values here instead of rewriting
settings.py with regexes.

This module must NOT import settings (settings imports it).
"""
from __future__ import annotations

import json
import logging
import os
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

OVERRIDABLE_KEYS = frozenset({
    "SERIAL_PORT",
    "CAMERA_INDEX",
    "MANUAL_ROLL_TRIM_DEG",
    "MANUAL_PITCH_TRIM_DEG",
    "PD_DEFAULT_KP",
    "PD_DEFAULT_KD",
    "TRACKER_HSV_H_MIN",
    "TRACKER_HSV_H_MAX",
    "TRACKER_HSV_S_MIN",
    "TRACKER_HSV_S_MAX",
    "TRACKER_HSV_V_MIN",
    "TRACKER_HSV_V_MAX",
})


def user_settings_path() -> Path:
    """The overlay file lives next to this module (repo root), not cwd."""
    return Path(__file__).resolve().parent / "user_settings.json"


def load_user_overrides(path: Path | None = None) -> dict[str, Any]:
    """Load the overlay. Missing file -> {}. Corrupt or non-dict content ->
    {} with a warning. Unknown keys are dropped with a warning."""
    p = path if path is not None else user_settings_path()
    try:
        text = p.read_text(encoding="utf-8")
    except FileNotFoundError:
        return {}
    except OSError as exc:
        logger.warning("user settings %s unreadable (%s); ignoring overrides", p, exc)
        return {}
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        logger.warning("user settings %s is not valid JSON; ignoring overrides", p)
        return {}
    if not isinstance(data, dict):
        logger.warning("user settings %s is not a JSON object; ignoring overrides", p)
        return {}
    overrides: dict[str, Any] = {}
    for key, value in data.items():
        if key in OVERRIDABLE_KEYS:
            overrides[key] = value
        else:
            logger.warning("user settings %s: dropping unknown key %r", p, key)
    return overrides


def save_user_overrides(updates: dict[str, Any], path: Path | None = None) -> None:
    """Merge ``updates`` over the existing overlay and write atomically
    (tmp file in the same directory + os.replace), so e.g. saving trims
    never clobbers a stored SERIAL_PORT."""
    p = path if path is not None else user_settings_path()
    merged = load_user_overrides(p)
    for key, value in updates.items():
        if key in OVERRIDABLE_KEYS:
            merged[key] = value
        else:
            logger.warning("save_user_overrides: dropping unknown key %r", key)
    tmp = p.parent / (p.name + ".tmp")
    tmp.write_text(json.dumps(merged, indent=2, sort_keys=True), encoding="utf-8")
    os.replace(tmp, p)
