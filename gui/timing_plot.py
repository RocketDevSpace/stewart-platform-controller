"""
gui/timing_plot.py

TimingPlotWidget: the vision-loop timing diagnostics strip (summary label
over a matplotlib canvas). Extracted from MainWindow so the top-level
window only wires modules together.

Rendering: the 10 Line2D artists and the legend are created ONCE in
__init__; each redraw only calls set_data / set_ylim and draw_idle().
No per-update cla()/replot/legend rebuild (the old approach rebuilt the
whole axes on every refresh, which dominated GUI-thread time).

The caller passes a fully-assembled timings dict to ingest() — derived
metrics measured at the call site (visualizer_gui, worker_to_gui_ms,
frame_to_cmd) are injected there, since the caller owns those
measurements. Missing keys plot as 0.0.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Any

from PyQt5.QtWidgets import QLabel, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from settings import GUI_SNAPSHOT_HZ

_TIMING_KEYS = [
    "ball_update",
    "pd_compute",
    "ik_solve",
    "serial_enqueue",
    "visualizer_gui",
    "frame_to_worker_ms",
    "worker_to_gui_ms",
    "frame_to_cmd",
    "serial_rtt_ms",
    "total",
]
_TIMING_COLORS = {
    "ball_update":       "#38bdf8",
    "pd_compute":        "#34d399",
    "ik_solve":          "#f43f5e",
    "serial_enqueue":    "#a78bfa",
    "visualizer_gui":    "#f59e0b",
    "frame_to_worker_ms": "#0ea5e9",
    "worker_to_gui_ms":  "#64748b",
    "frame_to_cmd":      "#22d3ee",
    "serial_rtt_ms":     "#fb7185",
    "total":             "#e2e8f0",
}
_TIMING_WINDOW_S = 30.0
_PLOT_UPDATE_EVERY = 5
# Deque headroom beyond the visible window. Samples arrive at snapshot
# rate (GUI_SNAPSHOT_HZ) — NOT at VISION_LOOP_HZ; the old capacity was
# sized off the loop rate and over-allocated ~4x.
_CAPACITY_SLACK_S = 5.0

_WAITING_TEXT = "Vision Timing Avg (ms): waiting for data..."


class TimingPlotWidget(QWidget):
    """Summary label + rolling timing plot for the vision loop."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.summary_label = QLabel(_WAITING_TEXT)
        self.summary_label.setMaximumHeight(22)

        figure = Figure(figsize=(5, 2.2))
        figure.patch.set_facecolor("#0f1726")
        self._ax = figure.add_subplot(111)
        self.canvas = FigureCanvas(figure)
        self.canvas.setFixedHeight(210)
        self.canvas.mpl_connect("scroll_event", self._on_scroll)

        self._init_axes_style()

        # Persistent artists — created once, updated via set_data().
        self._lines: dict[str, Any] = {}
        for key in _TIMING_KEYS:
            (line,) = self._ax.plot(
                [], [], label=key, linewidth=1.2, color=_TIMING_COLORS[key]
            )
            self._lines[key] = line
        self._ax.set_xlim(-_TIMING_WINDOW_S, 0.0)
        self._ax.set_ylim(0.0, 1.0)
        self._legend = self._ax.legend(loc="upper right", fontsize=6)

        capacity = int(GUI_SNAPSHOT_HZ * (_TIMING_WINDOW_S + _CAPACITY_SLACK_S))
        self._history: dict[str, deque] = {
            k: deque(maxlen=capacity) for k in _TIMING_KEYS
        }
        self._timestamps: deque = deque(maxlen=capacity)
        self._y_zoom = 1.0
        self._counter = 0  # own cadence counter, decoupled from log cadence

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)
        layout.addWidget(self.summary_label)
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def ingest(self, timings: dict) -> None:
        """Append one fully-assembled timings sample and refresh."""
        now = time.time()
        self._timestamps.append(now)
        for key in _TIMING_KEYS:
            self._history[key].append(float(timings.get(key, 0.0)))
        self._trim_window(now)
        self._counter += 1
        self._refresh(force_redraw=False)

    def reset(self) -> None:
        """Clear all history and return to the waiting state."""
        for hist in self._history.values():
            hist.clear()
        self._timestamps.clear()
        self._counter = 0
        self.summary_label.setText(_WAITING_TEXT)
        for line in self._lines.values():
            line.set_data([], [])
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _init_axes_style(self) -> None:
        self._ax.set_facecolor("#0f1726")
        self._ax.set_title(
            "Vision Loop Timings (30s)", color="#d6e2ff", fontsize=8
        )
        self._ax.tick_params(colors="#9fb4d9", labelsize=7)
        self._ax.grid(True, alpha=0.2, color="#2a3b59")
        self._ax.set_xlabel("Time (s)", color="#9fb4d9", fontsize=7)
        self._ax.set_ylabel("ms", color="#9fb4d9", fontsize=7)

    def _on_scroll(self, event: object) -> None:
        btn = getattr(event, "button", None)
        if btn == "up":
            self._y_zoom = max(0.2, self._y_zoom * 0.9)
        elif btn == "down":
            self._y_zoom = min(10.0, self._y_zoom * 1.1)
        self._refresh(force_redraw=True)

    def _trim_window(self, now: float) -> None:
        while (
            self._timestamps
            and (now - self._timestamps[0]) > _TIMING_WINDOW_S
        ):
            self._timestamps.popleft()
            for key in _TIMING_KEYS:
                if self._history[key]:
                    self._history[key].popleft()

    def _refresh(self, force_redraw: bool) -> None:
        if not self._history["total"]:
            return
        avgs = {}
        for key in _TIMING_KEYS:
            vals = self._history[key]
            avgs[key] = (sum(vals) / len(vals)) if vals else 0.0
        self.summary_label.setText(
            "Vision Timing Avg (ms):  "
            f"ball={avgs['ball_update']:.2f}  "
            f"pd={avgs['pd_compute']:.2f}  "
            f"ik={avgs['ik_solve']:.2f}  "
            f"serQ={avgs['serial_enqueue']:.2f}  "
            f"vis={avgs['visualizer_gui']:.2f}  "
            f"f2w={avgs['frame_to_worker_ms']:.2f}  "
            f"w2g={avgs['worker_to_gui_ms']:.2f}  "
            f"f2c={avgs['frame_to_cmd']:.2f}  "
            f"total={avgs['total']:.2f}  "
            f"zoom={self._y_zoom:.2f}x"
        )

        if not force_redraw and (self._counter % _PLOT_UPDATE_EVERY != 0):
            return
        if not self._timestamps:
            return

        now_ref = self._timestamps[-1]
        x = [t - now_ref for t in self._timestamps]
        y_max = 1.0
        for key in _TIMING_KEYS:
            y = list(self._history[key])
            self._lines[key].set_data(x[:len(y)], y)
            if y:
                y_max = max(y_max, max(y))
        self._ax.set_ylim(0.0, y_max * self._y_zoom)
        self.canvas.draw_idle()
