"""
state_bus.py

Thread-safe hand-off between the high-rate acquisition worker (up to
1125 Hz) and the fixed-rate GUI render loop (~60 Hz).

Two pieces of state are exposed:
  - a single "latest snapshot" slot (drop-oldest semantics: every publish()
    overwrites it, the GUI just reads whatever is current at its own pace)
  - a bounded rolling history of the per-component HW/SW delta vector,
    pruned by a time window so memory never grows unbounded even at 1125 Hz.

A plain threading.Lock is used rather than a queue: the GUI only ever wants
the *latest* value, never a backlog, so a queue with drop-oldest behavior
would just reduce to this anyway. Locking overhead here (~100 ns) is
negligible next to the ~0.9 ms per-packet budget at 1125 Hz.
"""

from __future__ import annotations

import threading
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class StateSnapshot:
    seq: int
    t: float                # seconds since acquisition start
    hw: np.ndarray          # [phi, theta, psi, bx, by, bz], rad / rad/s
    sw: np.ndarray          # same layout
    delta: np.ndarray       # hw - sw, per component, wraparound-safe on the 3 angles
    used_mag: bool          # whether the software EKF's update used the magnetometer


class StateBus:
    def __init__(self, history_window_s: float = 30.0, max_history_points: int = 200_000):
        self._lock = threading.Lock()
        self._latest: StateSnapshot | None = None
        self._t_hist: deque[float] = deque(maxlen=max_history_points)
        self._delta_hist: deque[np.ndarray] = deque(maxlen=max_history_points)
        self.history_window_s = history_window_s

    def publish(self, snapshot: StateSnapshot) -> None:
        with self._lock:
            self._latest = snapshot
            self._t_hist.append(snapshot.t)
            self._delta_hist.append(snapshot.delta)
            cutoff = snapshot.t - self.history_window_s
            while self._t_hist and self._t_hist[0] < cutoff:
                self._t_hist.popleft()
                self._delta_hist.popleft()

    def snapshot(self) -> tuple[StateSnapshot | None, np.ndarray, np.ndarray]:
        """Called from the GUI thread at ~60 Hz. Returns the latest state
        plus a copy of the current rolling per-component delta history
        (shape (N, 6)) alongside its timestamps (shape (N,))."""
        with self._lock:
            latest = self._latest
            t_arr = np.fromiter(self._t_hist, dtype=float) if self._t_hist else np.empty(0)
            delta_arr = np.array(self._delta_hist) if self._delta_hist else np.empty((0, 6))
        return latest, t_arr, delta_arr

    def clear(self) -> None:
        with self._lock:
            self._latest = None
            self._t_hist.clear()
            self._delta_hist.clear()
