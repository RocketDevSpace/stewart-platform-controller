"""
gui/serial_monitor.py

Read-only display widget for the Arduino serial monitor stream.
Pure view: owns no serial logic, no callbacks. The MainWindow forwards
incoming lines via append_line() and outgoing commands via append_command().
"""

from __future__ import annotations

from PyQt5.QtWidgets import QLabel, QTextEdit, QVBoxLayout, QWidget


class SerialMonitor(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._text = QTextEdit()
        self._text.setReadOnly(True)
        self._text.setPlaceholderText(
            "Arduino Serial Monitor output will appear here..."
        )

        layout = QVBoxLayout()
        layout.addWidget(QLabel("Serial Monitor:"))
        layout.addWidget(self._text)
        self.setLayout(layout)

    def append_line(self, line: str) -> None:
        self._text.append(line)

    def append_command(self, cmd: str) -> None:
        self._text.append(f">>> {cmd}")
