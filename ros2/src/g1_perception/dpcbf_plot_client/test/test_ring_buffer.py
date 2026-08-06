"""TimeSeriesBuffer / TrailBuffer: the pure-python plot data core."""
import pytest

from dpcbf_plot_client.ring_buffer import TimeSeriesBuffer, TrailBuffer


def test_append_and_arrays():
    buf = TimeSeriesBuffer(window_s=10.0)
    for i in range(5):
        buf.append(float(i), float(i) * 2.0)
    ts, vs = buf.arrays()
    assert ts == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert vs == [0.0, 2.0, 4.0, 6.0, 8.0]
    assert buf.latest() == (4.0, 8.0)
    assert len(buf) == 5


def test_window_eviction():
    buf = TimeSeriesBuffer(window_s=2.0)
    for i in range(100):
        buf.append(0.1 * i, float(i))
    ts, _ = buf.arrays()
    assert ts[-1] == pytest.approx(9.9)
    assert ts[0] >= 9.9 - 2.0
    # window is 2 s at 10 Hz -> about 21 samples survive
    assert 18 <= len(buf) <= 22


def test_empty_and_clear():
    buf = TimeSeriesBuffer(window_s=1.0)
    assert buf.latest() is None
    assert buf.arrays() == ([], [])
    buf.append(0.0, 1.0)
    buf.clear()
    assert len(buf) == 0


def test_invalid_window():
    with pytest.raises(ValueError):
        TimeSeriesBuffer(window_s=0.0)
    with pytest.raises(ValueError):
        TrailBuffer(window_s=-1.0)


def test_trail_eviction():
    trail = TrailBuffer(window_s=1.0)
    for i in range(50):
        trail.append(0.1 * i, float(i), -float(i))
    xs, ys = trail.arrays()
    assert len(xs) == len(ys)
    assert 9 <= len(xs) <= 12
    assert xs[-1] == 49.0 and ys[-1] == -49.0
