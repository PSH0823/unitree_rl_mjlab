"""pyqtgraph backend: the live DPCBF dashboard.

Layout (one window):

  +--------------------------+----------------------------------+
  |                          |  linear commands  nominal vs safe|
  |   2-D top-down (odom)    |  yaw commands     nominal vs safe|
  |   robot pose + heading   |  intervention + command_scale    |
  |   trail, obstacle circles|  min_h + min_clearance           |
  |   velocity + cmd arrows  |  ages: obstacle / latency        |
  |   stale banner           |                                  |
  +--------------------------+----------------------------------+

The GUI timer (default 25 Hz) pulls DataHub.snapshot() and redraws; ROS
callbacks never touch Qt. When a source stops arriving its banner turns red
and shows the age — the window keeps updating, which is itself the "link is
dead" signal (a frozen window can't tell you anything).
"""
import math

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from .data_hub import MODE_NAMES, STALENESS_NAMES

pg.setConfigOptions(antialias=True)

_COL_NOMINAL = (120, 180, 255)
_COL_SAFE = (255, 170, 60)
_COL_INTERV = (255, 80, 80)
_COL_SCALE = (160, 160, 160)
_COL_H = (130, 220, 130)
_COL_CLEAR = (200, 200, 120)
_COL_AGE = (200, 140, 255)
_COL_LATENCY = (120, 200, 220)
_COL_TRAIL = (90, 130, 170)
_COL_ROBOT = (80, 200, 255)
_COL_OBS_PLOT = (255, 120, 120)
_COL_OBS_SAFE = (255, 60, 60, 90)


def _circle(x, y, r, n=40):
    ts = [2.0 * math.pi * i / n for i in range(n + 1)]
    return ([x + r * math.cos(t) for t in ts],
            [y + r * math.sin(t) for t in ts])


class PlotApp:
    def __init__(self, hub, gui_rate_hz: float = 25.0, trail_s: float = 30.0):
        self.hub = hub
        self.app = QtWidgets.QApplication.instance() or \
            QtWidgets.QApplication([])
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle('DPCBF live plot (read-only client)')
        self.win.resize(1500, 900)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        self.win.setCentralWidget(central)

        # ---- left: top-down view -----------------------------------------
        left = pg.GraphicsLayoutWidget()
        layout.addWidget(left, stretch=5)
        self.view = left.addPlot(row=0, col=0)
        self.view.setAspectLocked(True)
        self.view.showGrid(x=True, y=True, alpha=0.25)
        self.view.setLabel('bottom', 'x [m] (odom)')
        self.view.setLabel('left', 'y [m] (odom)')

        self.trail_curve = self.view.plot([], [], pen=pg.mkPen(_COL_TRAIL,
                                                               width=1))
        self.robot_marker = self.view.plot(
            [], [], pen=None, symbol='o', symbolSize=10,
            symbolBrush=_COL_ROBOT)
        self.heading_line = self.view.plot([], [],
                                           pen=pg.mkPen(_COL_ROBOT, width=2))
        self.nominal_arrow = self.view.plot(
            [], [], pen=pg.mkPen(_COL_NOMINAL, width=2,
                                 style=QtCore.Qt.DashLine))
        self.safe_arrow = self.view.plot([], [],
                                         pen=pg.mkPen(_COL_SAFE, width=3))
        self.obstacle_curves = []   # recreated per frame (<= ~20 items)

        # anchor (0, 0): the text's TOP-left sits at the given position, so
        # the banner hangs down INTO the view instead of being clipped above
        # it. The command legend moves to the top-right to leave this corner
        # free.
        self.banner = pg.TextItem(anchor=(0, 0))
        self.view.addItem(self.banner, ignoreBounds=True)

        legend = self.view.addLegend(offset=(-10, 10))
        legend.addItem(self.nominal_arrow, 'nominal cmd')
        legend.addItem(self.safe_arrow, 'safe cmd')

        # ---- right: time series ------------------------------------------
        right = pg.GraphicsLayoutWidget()
        layout.addWidget(right, stretch=4)

        def ts_plot(row, title, y_label):
            p = right.addPlot(row=row, col=0)
            p.setTitle(title, size='9pt')
            p.setLabel('left', y_label)
            p.showGrid(x=True, y=True, alpha=0.2)
            p.setXRange(-30.0, 0.0)
            return p

        self.p_lin = ts_plot(0, 'linear command [m/s]', 'm/s')
        self.c_nom_sag = self.p_lin.plot([], [], pen=_COL_NOMINAL,
                                         name='nominal sag')
        self.c_safe_sag = self.p_lin.plot([], [], pen=_COL_SAFE)
        self.c_nom_lat = self.p_lin.plot(
            [], [], pen=pg.mkPen(_COL_NOMINAL, style=QtCore.Qt.DotLine))
        self.c_safe_lat = self.p_lin.plot(
            [], [], pen=pg.mkPen(_COL_SAFE, style=QtCore.Qt.DotLine))
        leg = self.p_lin.addLegend(offset=(2, 2))
        leg.addItem(self.c_nom_sag, 'nominal sag')
        leg.addItem(self.c_safe_sag, 'safe sag')
        leg.addItem(self.c_nom_lat, 'nominal lat')
        leg.addItem(self.c_safe_lat, 'safe lat')

        self.p_yaw = ts_plot(1, 'yaw rate command [rad/s]', 'rad/s')
        self.c_nom_yaw = self.p_yaw.plot([], [], pen=_COL_NOMINAL)
        self.c_safe_yaw = self.p_yaw.plot([], [], pen=_COL_SAFE)

        self.p_int = ts_plot(2, 'DPCBF intervention / command scale', '')
        self.p_int.setYRange(-0.05, 1.15)
        self.c_interv = self.p_int.plot(
            [], [], pen=_COL_INTERV, fillLevel=0.0,
            brush=(255, 80, 80, 60), stepMode=None)
        self.c_scale = self.p_int.plot([], [], pen=_COL_SCALE)
        leg = self.p_int.addLegend(offset=(2, 2))
        leg.addItem(self.c_interv, 'intervention')
        leg.addItem(self.c_scale, 'command_scale')

        self.p_h = ts_plot(3, 'barrier: min h / min clearance [m]', '')
        self.c_min_h = self.p_h.plot([], [], pen=_COL_H)
        self.c_min_clear = self.p_h.plot([], [], pen=_COL_CLEAR)
        self.p_h.addLine(y=0.0, pen=pg.mkPen((255, 80, 80, 120),
                                             style=QtCore.Qt.DashLine))
        leg = self.p_h.addLegend(offset=(2, 2))
        leg.addItem(self.c_min_h, 'min h')
        leg.addItem(self.c_min_clear, 'min clearance')

        self.p_age = ts_plot(4, 'obstacle age / plot latency [s]', 's')
        self.p_age.setLabel('bottom', 'seconds ago')
        self.c_age = self.p_age.plot([], [], pen=_COL_AGE)
        self.c_lat = self.p_age.plot([], [], pen=_COL_LATENCY)
        leg = self.p_age.addLegend(offset=(2, 2))
        leg.addItem(self.c_age, 'obstacle age (ctrl-side)')
        leg.addItem(self.c_lat, 'plot latency (needs NTP)')

        self.frames_rendered = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(max(10, int(1000.0 / gui_rate_hz)))

    # ------------------------------------------------------------------
    def refresh(self):
        snap = self.hub.snapshot()
        self._refresh_series(snap)
        self._refresh_topdown(snap)
        self.frames_rendered += 1

    def _refresh_series(self, snap):
        s = snap['series']
        self.c_nom_sag.setData(*s['nominal_sagittal'])
        self.c_safe_sag.setData(*s['safe_sagittal'])
        self.c_nom_lat.setData(*s['nominal_lateral'])
        self.c_safe_lat.setData(*s['safe_lateral'])
        self.c_nom_yaw.setData(*s['nominal_yaw'])
        self.c_safe_yaw.setData(*s['safe_yaw'])
        self.c_interv.setData(*s['intervention'])
        self.c_scale.setData(*s['command_scale'])
        self.c_min_h.setData(*s['min_h'])
        self.c_min_clear.setData(*s['min_clearance'])
        self.c_age.setData(*s['obstacle_age'])
        self.c_lat.setData(*s['latency'])

    def _refresh_topdown(self, snap):
        for item in self.obstacle_curves:
            self.view.removeItem(item)
        self.obstacle_curves = []

        # Robot pose: prefer /odom (survives when the DPCBF binary is down),
        # fall back to the plot sample's robot state.
        pose = None
        odom = snap['odom']
        sample = snap['sample']
        if odom is not None:
            pose = (odom['x'], odom['y'], odom['yaw'])
        elif sample is not None:
            pose = (sample.robot_x, sample.robot_y, sample.robot_phi)

        self.trail_curve.setData(*snap['trail'])

        if pose is not None:
            x, y, yaw = pose
            self.robot_marker.setData([x], [y])
            hx = x + 0.4 * math.cos(yaw)
            hy = y + 0.4 * math.sin(yaw)
            self.heading_line.setData([x, hx], [y, hy])

            if sample is not None:
                def cmd_arrow(cmd, scale=1.0):
                    # body sagittal/lateral -> world, drawn from the robot
                    wx = (cmd.sagittal * math.cos(yaw)
                          - cmd.lateral * math.sin(yaw)) * scale
                    wy = (cmd.sagittal * math.sin(yaw)
                          + cmd.lateral * math.cos(yaw)) * scale
                    return [x, x + wx], [y, y + wy]
                self.nominal_arrow.setData(*cmd_arrow(sample.nominal))
                self.safe_arrow.setData(*cmd_arrow(sample.safe))

        # Obstacles: filter-selected set (colored by barrier h) + the full
        # /obstacles_safe stream (faint red) when available.
        if sample is not None:
            for o in sample.obstacles:
                color = (_COL_INTERV if o.h < 0.0 else _COL_OBS_PLOT)
                cx, cy = _circle(o.x, o.y, max(o.radius, 0.02))
                item = self.view.plot(cx, cy, pen=pg.mkPen(color, width=2))
                self.obstacle_curves.append(item)
                varrow = self.view.plot(
                    [o.x, o.x + o.velocity_x], [o.y, o.y + o.velocity_y],
                    pen=pg.mkPen(color, width=1, style=QtCore.Qt.DashLine))
                self.obstacle_curves.append(varrow)
        if snap['obstacles']:
            for o in snap['obstacles']:
                cx, cy = _circle(o['x'], o['y'], max(o['radius'], 0.02))
                item = self.view.plot(cx, cy, pen=pg.mkPen(_COL_OBS_SAFE,
                                                           width=1))
                self.obstacle_curves.append(item)

        self._refresh_banner(snap)

    def _refresh_banner(self, snap):
        ages = snap['ages']
        stale_after = snap['stale_after_s']
        lines = []
        any_stale = False

        def describe(name, key):
            nonlocal any_stale
            age = ages[key]
            if age is None:
                any_stale = True
                return f'{name}: NO DATA'
            if age > stale_after:
                any_stale = True
                return f'{name}: STALE {age:.1f}s'
            return f'{name}: ok {age * 1e3:.0f}ms'

        lines.append(describe('dpcbf/plot', 'plot'))
        lines.append(describe('odom', 'odom'))
        if snap['obstacles_available']:
            lines.append(describe('obstacles_safe', 'obstacles'))
        sample = snap['sample']
        if sample is not None:
            mode = MODE_NAMES.get(sample.mode, '?')
            stale = STALENESS_NAMES.get(sample.staleness_state, '?')
            lines.append(f'mode={mode} ctrl-staleness={stale} '
                         f'tick={sample.tick}')
        color = (255, 80, 80) if any_stale else (140, 220, 140)
        self.banner.setColor(color)
        self.banner.setText('\n'.join(lines))
        # pin to the view's top-left corner
        vr = self.view.viewRange()
        self.banner.setPos(vr[0][0], vr[1][1])

    # ------------------------------------------------------------------
    def run(self):
        self.win.show()
        return self.app.exec_()
