# Stewart Platform Controller

6-DOF Stewart platform controller with closed-loop ball balancing.
**Windows 10/11 only.** Communicates with an Arduino over serial, uses OpenCV for vision, and presents a PyQt5 desktop GUI.

## Prerequisites

- Python 3.11 or newer
- Arduino flashed with the companion firmware (see [`firmware/`](firmware/README.md) — pin map, wiring order, protocol, and a flashable image)
- USB camera (index 1 by default — see Configure below)
- Windows 10 or 11

## Install

```
git clone https://github.com/RocketDevSpace/stewart-platform-controller.git
cd stewart-platform-controller
pip install -r requirements.txt
```

## Configure

`settings.py` holds the committed, neutral defaults — **don't edit it for your
machine.** Per-machine values live in `user_settings.json` in the repo root: a
plain JSON file, untracked by git, holding overrides for a whitelisted set of
13 keys (serial port, camera index, roll/pitch trims, PID gains, and the six
HSV thresholds — see `OVERRIDABLE_KEYS` in `settings_store.py`). Nothing is
created automatically on first run; without the file the defaults apply.

Two ways to set your values:

1. **Edit `user_settings.json` by hand.** Create it next to `main.py`:

   ```json
   {
     "SERIAL_PORT": "COM8",
     "CAMERA_INDEX": 1
   }
   ```

   `SERIAL_PORT` is the COM port the Arduino appears on — check Device
   Manager under "Ports (COM & LPT)". `CAMERA_INDEX` 0 = integrated webcam,
   1 = first USB camera.

2. **Use the GUI.** The "Save Trim as Default" button writes the current
   roll/pitch trim values into `user_settings.json` (atomically, preserving
   any keys you set by hand). Values take effect at the next launch.

All other defaults work out of the box for the reference hardware.

## Run

```
python main.py
```

The GUI opens. Connect the Arduino first, then click **Connect** in the GUI.
The camera starts automatically when the vision panel is activated.

## Development / testing

```
pip install -r requirements-dev.txt
pytest          # unit tests
flake8          # lint
mypy .          # type check (runs locally; same config as CI)
```

See `CLAUDE.md` for architecture, contribution rules, and milestone history.
See `CHANGELOG.md` for what changed in each milestone.
