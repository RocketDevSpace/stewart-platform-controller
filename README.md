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

Open `settings.py` and set these two values to match your machine before running:

```python
SERIAL_PORT = "COM4"   # COM port the Arduino appears on (check Device Manager)
CAMERA_INDEX = 1       # 0 = integrated webcam, 1 = first USB camera
```

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
mypy            # type check
```

See `CLAUDE.md` for architecture, contribution rules, and milestone history.
See `CHANGELOG.md` for what changed in each milestone.
