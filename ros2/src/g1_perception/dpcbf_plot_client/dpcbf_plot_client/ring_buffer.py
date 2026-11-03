"""Time-windowed ring buffer for plot series.

Pure python, no ROS, no GUI — the unit-testable core of the client's data
path. Append is O(1); eviction trims everything older than `window_s`
behind the newest sample. `arrays()` returns plain lists (pyqtgraph and
matplotlib both accept them) copied under the caller's locking discipline —
the buffer itself is NOT thread-safe by design; DataHub owns the lock.
"""
from collections import deque


class TimeSeriesBuffer:
    def __init__(self, window_s: float = 30.0):
        if window_s <= 0.0:
            raise ValueError('window_s must be positive')
        self.window_s = float(window_s)
        self._t = deque()
        self._v = deque()

    def append(self, t: float, value: float) -> None:
        self._t.append(float(t))
        self._v.append(float(value))
        horizon = self._t[-1] - self.window_s
        while self._t and self._t[0] < horizon:
            self._t.popleft()
            self._v.popleft()

    def arrays(self):
        return list(self._t), list(self._v)

    def latest(self):
        if not self._t:
            return None
        return self._t[-1], self._v[-1]

    def __len__(self):
        return len(self._t)

    def clear(self) -> None:
        self._t.clear()
        self._v.clear()


class TrailBuffer:
    """XY trail (robot trajectory) with the same time-window eviction."""

    def __init__(self, window_s: float = 30.0):
        if window_s <= 0.0:
            raise ValueError('window_s must be positive')
        self.window_s = float(window_s)
        self._t = deque()
        self._x = deque()
        self._y = deque()

    def append(self, t: float, x: float, y: float) -> None:
        self._t.append(float(t))
        self._x.append(float(x))
        self._y.append(float(y))
        horizon = self._t[-1] - self.window_s
        while self._t and self._t[0] < horizon:
            self._t.popleft()
            self._x.popleft()
            self._y.popleft()

    def arrays(self):
        return list(self._x), list(self._y)

    def __len__(self):
        return len(self._t)
