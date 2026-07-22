"""Entry point.

Installs a custom sys.excepthook BEFORE the Qt app starts: PyQt5 (>= 5.5)
hard-aborts the process via qFatal() on any unhandled exception in a slot
if the default hook is installed — which would kill the app mid-hardware-
control with no serial disconnect. The custom hook logs the traceback,
attempts an orderly shutdown (ramped neutral pose + serial disconnect via
MainWindow.emergency_shutdown), and exits cleanly.
"""

import logging
import sys
import traceback
from types import TracebackType

from PyQt5 import QtWidgets

from gui.main_window import MainWindow

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
logger = logging.getLogger("main")

_window: MainWindow | None = None
_crashing = False


def _excepthook(
    exc_type: type[BaseException],
    exc: BaseException,
    tb: TracebackType | None,
) -> None:
    global _crashing
    logger.critical(
        "Unhandled exception:\n%s",
        "".join(traceback.format_exception(exc_type, exc, tb)),
    )
    if _crashing:  # a crash inside crash handling: just die
        sys.exit(2)
    _crashing = True
    try:
        if _window is not None:
            _window.emergency_shutdown()
    except Exception:
        logger.exception("emergency shutdown failed")
    app = QtWidgets.QApplication.instance()
    if app is not None:
        app.exit(2)


if __name__ == "__main__":
    sys.excepthook = _excepthook
    app = QtWidgets.QApplication(sys.argv)
    _window = MainWindow()
    _window.show()
    sys.exit(app.exec_())
