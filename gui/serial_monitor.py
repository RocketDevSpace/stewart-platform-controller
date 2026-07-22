"""
gui/serial_monitor.py

Read-only display widget for the Arduino serial monitor stream.
Pure view: owns no serial logic, no callbacks. The MainWindow forwards
incoming lines via append_line() and outgoing commands via append_command().
"""

from __future__ import annotations

from PyQt5.QtWidgets import QLabel, QTextEdit, QVBoxLayout, QWidget

from settings import GUI_LOG_MAX_LINES


class SerialMonitor(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setPlaceholderText(
            "Arduino Serial Monitor output will appear here..."
        )
        doc = self._text.document()
        if doc is not None:
            # Qt drops the oldest blocks automatically — the widget used
            # to grow without bound over a long session.
            doc.setMaximumBlockCount(GUI_LOG_MAX_LINES)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("Serial Monitor:"))
        layout.addWidget(self._text)
        self.setLayout(layout)

    def append_line(self, line: str) -> None:
        # Firmware v2 acks every streamed T command with a bare "k" — at up
        # to 30/s that floods the monitor with noise. The acks are consumed
        # by the RTT telemetry; the display drops them.
        if line == "k":
            return
        self._text.append(line)

    def append_command(self, cmd: str) -> None:
        self._text.append(f">>> {cmd}")
