#!/usr/bin/python3
"""Pure analysis core shared by the hardware probes (Phase 5C).

NO ROS IMPORTS. Everything here is arithmetic over numbers the probes have
already extracted from messages, which is what makes the probes' *judgements*
— "that stamp went backwards", "that is not the driver's point layout",
"those stamps are host-clock, not sensor-clock" — testable in CTest on a
machine with no LiDAR, no robot and no rclpy.

`hw_source_probe.py` and `hw_tf_probe.py` import this; so does
`test/test_hw_offline_gates.py`. If a rule is worth asserting, it belongs
here rather than inline in a callback.
"""
import math

# --- livox_ros_driver2 (tag 1.2.6, patch 0005) xfer_format 0 --------------
# Verified against the pinned checkout, not the README: the point struct is
# #pragma pack(1), so the FLOAT64 timestamp sits at offset 18 unaligned and
# point_step is 26. Same constants hw_source_stub.py emits.
PF_UINT8, PF_FLOAT32, PF_FLOAT64 = 2, 7, 8
LIVOX_POINT_STEP = 26
LIVOX_FIELDS = (
    ('x', 0, PF_FLOAT32),
    ('y', 4, PF_FLOAT32),
    ('z', 8, PF_FLOAT32),
    ('intensity', 12, PF_FLOAT32),
    ('tag', 16, PF_UINT8),
    ('line', 17, PF_UINT8),
    ('timestamp', 18, PF_FLOAT64),
)
# §7.2: downstream needs x,y,z and tolerates extra fields. Anything missing
# from this set is fatal; a layout that merely differs from the driver's is
# reported as a deviation, because it means the source is not the source the
# whole stack was gated against.
REQUIRED_FIELDS = ('x', 'y', 'z')

EXPECTED_FRAME = 'mid360_link'          # §7.1/§8.1, set in livox_driver.yaml


def check_cloud_layout(fields, point_step):
    """fields: [(name, offset, datatype), ...]. Returns (fatal, deviations)."""
    fatal, dev = [], []
    got = {n: (o, d) for n, o, d in fields}
    for name in REQUIRED_FIELDS:
        if name not in got:
            fatal.append(f'PointCloud2 has no {name!r} field — downstream '
                         'CropBox/projection cannot read this cloud '
                         f'(fields present: {sorted(got)})')
        elif got[name][1] != PF_FLOAT32:
            fatal.append(f'field {name!r} datatype {got[name][1]} != FLOAT32')
    if point_step != LIVOX_POINT_STEP:
        dev.append(f'point_step {point_step} != {LIVOX_POINT_STEP} (the '
                   'pinned driver xfer_format 0 layout the seam tests gate '
                   'on) — this is not livox_ros_driver2 1.2.6 output')
    for name, off, dt in LIVOX_FIELDS:
        if name not in got:
            dev.append(f'driver field {name!r} absent')
        elif got[name] != (off, dt):
            dev.append(f'field {name!r} at offset {got[name][0]} type '
                       f'{got[name][1]}, driver has offset {off} type {dt}')
    return fatal, dev


def check_frame_id(actual, expected=EXPECTED_FRAME):
    """§7.1. The driver's own ROS2 default is `frame_default`; its example
    launch files say `livox_frame`. Either means livox_driver.yaml did not
    reach the node, and every TF lookup downstream fails on a frame nobody
    publishes."""
    if actual == expected:
        return None
    hint = ''
    if actual in ('frame_default', 'livox_frame'):
        hint = (' — that is the driver default, i.e. config/livox_driver.yaml '
                'was not loaded (stale install prefix?)')
    return f'frame_id {actual!r} != {expected!r}{hint}'


class StampSeries:
    """Header-stamp / arrival-time bookkeeping for one topic.

    Feed it (stamp_s, recv_s) per message in arrival order. It answers the
    four questions a first hardware session actually needs: is the stream
    there, at what rate, do its stamps ever go backwards, and are its stamps
    in the same clock as this host.
    """

    def __init__(self, name, expect_hz=None, gap_factor=3.0):
        self.name = name
        self.expect_hz = expect_hz
        self.gap_factor = gap_factor
        self.n = 0
        self.first_recv = None
        self.last_recv = None
        self.last_stamp = None
        self.regressions = []      # (index, prev_stamp, stamp)
        self.jumps = []            # (index, dt) forward jumps beyond gap
        self.gaps = []             # (index, arrival dt) missing intervals
        self.offsets = []          # recv - stamp, the clock-domain evidence
        self.stamp_dts = []
        self.recv_dts = []
        self.zero_stamps = 0

    def add(self, stamp_s, recv_s):
        self.n += 1
        if stamp_s == 0.0:
            # DLIO's /odom stamps are 0 until its first IMU message; a driver
            # that has not seen a packet header does the same. Counted, not
            # treated as a regression.
            self.zero_stamps += 1
        if self.first_recv is None:
            self.first_recv = recv_s
        else:
            d = recv_s - self.last_recv
            self.recv_dts.append(d)
            if self._gap_threshold() and d > self._gap_threshold():
                self.gaps.append((self.n, d))
        if self.last_stamp is not None and stamp_s != 0.0 and self.last_stamp != 0.0:
            ds = stamp_s - self.last_stamp
            self.stamp_dts.append(ds)
            if ds < 0.0:
                self.regressions.append((self.n, self.last_stamp, stamp_s))
            elif self._gap_threshold() and ds > self._gap_threshold():
                self.jumps.append((self.n, ds))
        self.last_recv = recv_s
        if stamp_s != 0.0:
            self.last_stamp = stamp_s
            self.offsets.append(recv_s - stamp_s)

    def _gap_threshold(self):
        if not self.expect_hz:
            return None
        return self.gap_factor / self.expect_hz

    @property
    def rate_hz(self):
        if self.n < 2 or self.first_recv is None:
            return None
        span = self.last_recv - self.first_recv
        return (self.n - 1) / span if span > 0 else None

    def summary(self):
        off = sorted(self.offsets)
        return {
            'topic': self.name,
            'count': self.n,
            'rate_hz': _r(self.rate_hz),
            'expect_hz': self.expect_hz,
            'stamp_regressions': len(self.regressions),
            'stamp_forward_jumps': len(self.jumps),
            'arrival_gaps': len(self.gaps),
            'zero_stamps': self.zero_stamps,
            'age_p50_s': _r(_pct(off, 50)),
            'age_p95_s': _r(_pct(off, 95)),
            'age_max_s': _r(off[-1] if off else None),
            'clock_domain': classify_time_domain(self.offsets),
        }

    def problems(self):
        out = []
        if self.n == 0:
            out.append(f'{self.name}: NO MESSAGES — publisher absent, QoS '
                       'incompatible, or the sensor is not producing')
            return out
        if self.regressions:
            i, a, b = self.regressions[0]
            out.append(f'{self.name}: {len(self.regressions)} header-stamp '
                       f'REGRESSION(s); first at msg {i}: {a:.6f} -> {b:.6f}. '
                       'tf2 and every message filter downstream treat this as '
                       'a time jump; do not proceed.')
        if self.expect_hz and self.rate_hz is not None:
            lo, hi = 0.8 * self.expect_hz, 1.2 * self.expect_hz
            if not (lo <= self.rate_hz <= hi):
                out.append(f'{self.name}: {self.rate_hz:.2f} Hz outside '
                           f'{lo:.1f}-{hi:.1f} Hz (expected {self.expect_hz})')
        if self.gaps:
            worst = max(d for _, d in self.gaps)
            out.append(f'{self.name}: {len(self.gaps)} arrival gap(s), worst '
                       f'{worst * 1e3:.0f} ms — packet loss or a stalled '
                       'publisher')
        return out


def classify_time_domain(offsets, tight_s=0.05, drift_s=0.5):
    """recv_wall - header_stamp, over a run. Decides §14.3's open question.

    The Mid-360 has no timestamp-mode field in MID360_config.json: the driver
    reads the sync mode out of each packet header and, unsynced, stamps with
    the HOST clock at packet reception. Which is live is only observable at
    runtime — this is the observation.
    """
    if not offsets:
        return 'unknown (no stamped messages)'
    s = sorted(offsets)
    med = _pct(s, 50)
    spread = s[-1] - s[0]
    if abs(med) > drift_s:
        return (f'sensor-clock or unsynced device clock (median offset '
                f'{med:.3f} s from this host)')
    if abs(med) <= tight_s and spread <= tight_s:
        return f'host-clock (median offset {med * 1e3:.1f} ms, stable)'
    return (f'host-clock with jitter (median {med * 1e3:.1f} ms, spread '
            f'{spread * 1e3:.0f} ms)')


def imu_plausibility(accel_mags, gyro_mags, g=9.80665, tol=1.5,
                     still_gyro=0.2):
    """Stationary sanity. Not a calibration — a "is this an IMU at all" test.

    A Mid-360 at rest reads |a| ~ g regardless of mount orientation, so this
    works without knowing H-1. It cannot detect a wrong SIGN or a swapped
    axis; that is stage 6's job (drive the sensor and watch odometry signs).
    """
    out = []
    if not accel_mags:
        out.append('IMU: no messages')
        return out
    a50 = _pct(sorted(accel_mags), 50)
    if not (g - tol <= a50 <= g + tol):
        out.append(f'IMU: median |accel| {a50:.2f} m/s^2 is not ~{g:.2f} — '
                   'wrong units, wrong message, or the sensor is moving')
    if gyro_mags:
        w50 = _pct(sorted(gyro_mags), 50)
        if w50 > still_gyro:
            out.append(f'IMU: median |gyro| {w50:.3f} rad/s while nominally '
                       'stationary — robot moving, or a biased/failed gyro. '
                       "DLIO's boot calibration will absorb this as bias.")
    return out


class TfLookupStats:
    """Stamped-lookup bookkeeping for hw_tf_probe.

    The whole point of the probe is that it asks for the transform AT THE
    LIDAR MESSAGE STAMP, never `Time()` (latest). A latest-lookup succeeds
    against a TF tree that is minutes stale, which is exactly the failure the
    probe exists to catch.
    """

    def __init__(self, pair):
        self.pair = pair
        self.attempts = 0
        self.ok = 0
        self.extrapolation_past = 0
        self.extrapolation_future = 0
        self.connectivity = 0
        self.other = 0
        self.delays = []            # seconds waited before the lookup answered
        self.first_error = None

    def record_ok(self, delay_s):
        self.attempts += 1
        self.ok += 1
        self.delays.append(delay_s)

    def record_fail(self, message):
        self.attempts += 1
        m = (message or '').lower()
        if 'extrapolation' in m and 'future' in m:
            self.extrapolation_future += 1
        elif 'extrapolation' in m or 'earlier than all the data' in m:
            self.extrapolation_past += 1
        elif 'connect' in m or 'does not exist' in m:
            self.connectivity += 1
        else:
            self.other += 1
        if self.first_error is None:
            self.first_error = message

    @property
    def success_fraction(self):
        return (self.ok / self.attempts) if self.attempts else None

    def summary(self):
        d = sorted(self.delays)
        return {
            'transform': self.pair,
            'attempts': self.attempts,
            'ok': self.ok,
            'success_fraction': _r(self.success_fraction),
            'delay_p50_s': _r(_pct(d, 50)),
            'delay_max_s': _r(d[-1] if d else None),
            'extrapolation_into_past': self.extrapolation_past,
            'extrapolation_into_future': self.extrapolation_future,
            'connectivity_failures': self.connectivity,
            'other_failures': self.other,
            'first_error': self.first_error,
        }

    def problems(self, min_fraction=0.95):
        out = []
        if self.attempts == 0:
            out.append(f'{self.pair}: never attempted (no LiDAR messages)')
            return out
        if self.success_fraction < min_fraction:
            out.append(f'{self.pair}: stamped lookup succeeded '
                       f'{self.success_fraction * 100:.1f} % of the time '
                       f'(< {min_fraction * 100:.0f} %) — '
                       f'first error: {self.first_error}')
        if self.extrapolation_future:
            out.append(f'{self.pair}: {self.extrapolation_future} lookups '
                       'extrapolated into the FUTURE — the TF source is '
                       'behind the cloud, or a static stand-in is being used '
                       'where real odometry is required')
        return out


# --- diagnostics levels ---------------------------------------------------
# Same numeric values as diagnostic_msgs/DiagnosticStatus, restated here so
# the decision is testable without ROS. hw_diagnostics.py calls this rather
# than deciding inline: "a failure must not be hidden merely because the
# topic exists" is a rule, and a rule belongs somewhere a test can reach it.
LEVEL_OK, LEVEL_WARN, LEVEL_ERROR = 0, 1, 2


def rate_level(count, rate_hz, expect_hz, age_s, max_age_s):
    """Returns (level, message) for one topic row.

    No data is ERROR, never OK-with-no-message. Half the expected rate is
    ERROR because at 5 of 10 Hz the pipeline is dropping frames, not running
    slowly. Age is only consulted once the rate is acceptable — a stale
    stamp on a healthy rate is the clock-domain problem, a different fault.
    """
    if not count:
        return LEVEL_ERROR, 'NO DATA'
    if expect_hz and rate_hz < 0.5 * expect_hz:
        return (LEVEL_ERROR,
                f'{rate_hz:.1f} Hz, expected ~{expect_hz:.0f} Hz '
                '(more than half the messages are missing)')
    if expect_hz and rate_hz < 0.8 * expect_hz:
        return LEVEL_WARN, f'{rate_hz:.1f} Hz below {0.8 * expect_hz:.0f} Hz'
    if age_s is not None and max_age_s and age_s > max_age_s:
        return LEVEL_WARN, f'stale: newest stamp is {age_s:.2f} s old'
    return LEVEL_OK, f'{rate_hz:.1f} Hz'


# --- small helpers --------------------------------------------------------
def _pct(sorted_vals, p):
    if not sorted_vals:
        return None
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


def _r(v, nd=6):
    return None if v is None else round(float(v), nd)


def render_text(title, summaries, problems):
    """Human-readable half of the machine+human output contract."""
    lines = [f'=== {title} ===']
    for s in summaries:
        lines.append('')
        for k, v in s.items():
            lines.append(f'  {k:<28} {v}')
    lines.append('')
    if problems:
        lines.append(f'PROBLEMS ({len(problems)}):')
        lines += [f'  ! {p}' for p in problems]
    else:
        lines.append('PROBLEMS: none detected by this probe')
    lines.append('')
    lines.append('A clean probe is NOT a pass for the stage — it is the '
                 'absence of the failures this probe can see.')
    return '\n'.join(lines)
