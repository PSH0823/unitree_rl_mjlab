"""Pyqtgraph DPCBF dashboard with the original white visualizer style.

The left panel is an odom-frame top-down view. Every detected obstacle is a
blue disc; obstacles selected by the QP get a colored rim, LoS, relative
velocity and world-projected h=0 parabola. The right panel contains the
matching per-obstacle h=0 boundary in its LoS velocity frame.
"""
import math

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from .data_hub import MODE_NAMES, STALENESS_NAMES

pg.setConfigOptions(antialias=True, background='w', foreground='k')

_PEN_STYLE = getattr(QtCore.Qt, 'PenStyle', QtCore.Qt)
_DASH_LINE = _PEN_STYLE.DashLine

_COL_NOMINAL = (54, 103, 173)
_COL_SAFE = (224, 122, 38)
_COL_TRAIL = (145, 153, 160)
_COL_ROBOT = (25, 25, 25)
_COL_OBS_FILL = (86, 139, 196, 185)
_COL_OBS_EDGE = (39, 91, 148)
_COL_UNSAFE = (224, 224, 238, 105)
_ACTIVE_COLORS = ((230, 126, 34), (22, 138, 157), (76, 120, 168),
                  (139, 106, 174), (193, 63, 138), (181, 139, 22))
_VELOCITY_AXIS_LIMIT = 3.0


def _circle(x, y, radius, n=64):
    angles = [2.0 * math.pi * i / n for i in range(n + 1)]
    return ([x + radius * math.cos(a) for a in angles],
            [y + radius * math.sin(a) for a in angles])


class PlotApp:
    def __init__(self, hub, gui_rate_hz: float = 25.0, trail_s: float = 30.0):
        del trail_s  # DataHub owns the configured trail window.
        self.hub = hub
        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.win = QtWidgets.QMainWindow()
        self.win.setWindowTitle('DPCBF top-down and velocity boundaries')
        self.win.resize(1500, 900)
        self.win.setStyleSheet('background-color: white; color: black;')

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        self.win.setCentralWidget(central)

        left = pg.GraphicsLayoutWidget()
        layout.addWidget(left, stretch=5)
        left.addLabel('World top-down', row=0, col=0, color='#2d2d2d',
                      size='12pt')
        self.view = left.addPlot(row=1, col=0)
        self.view.setAspectLocked(True)
        self.view.showGrid(x=True, y=True, alpha=0.18)
        self.view.setLabel('bottom', 'x [m] (odom)')
        self.view.setLabel('left', 'y [m] (odom)')

        self.trail_curve = self.view.plot(
            [], [], pen=pg.mkPen(_COL_TRAIL, width=1))
        self.robot_marker = self.view.plot(
            [], [], pen=None, symbol='o', symbolSize=15,
            symbolBrush=_COL_ROBOT,
            symbolPen=pg.mkPen(_COL_ROBOT, width=2))
        self.heading_line = self.view.plot(
            [], [], pen=pg.mkPen(_COL_ROBOT, width=2))
        self.nominal_arrow = self.view.plot(
            [], [], pen=pg.mkPen(_COL_NOMINAL, width=2, style=_DASH_LINE))
        self.safe_arrow = self.view.plot(
            [], [], pen=pg.mkPen(_COL_SAFE, width=3))
        self.dynamic_items = []

        self.banner = pg.TextItem(anchor=(0, 0), color='#237a3b')
        self.view.addItem(self.banner, ignoreBounds=True)
        legend = self.view.addLegend(offset=(-10, 10))
        legend.addItem(self.nominal_arrow, 'nominal cmd')
        legend.addItem(self.safe_arrow, 'safe cmd')

        self.right = pg.GraphicsLayoutWidget()
        layout.addWidget(self.right, stretch=4)
        self.velocity_cards = []
        self._velocity_card_count = -1
        self._build_velocity_cards(0)

        self.frames_rendered = 0
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(max(10, int(1000.0 / gui_rate_hz)))

    def _build_velocity_cards(self, count):
        self.right.clear()
        self.velocity_cards = []
        self._velocity_card_count = count
        self.right.addLabel(
            'DPCBF h=0 in each obstacle LoS velocity frame', row=0, col=0,
            colspan=2, color='#2d2d2d', size='11pt')
        if count == 0:
            self.right.addLabel('No obstacle selected inside p_max', row=1,
                                col=0, colspan=2, color='#666666')
            return

        columns = 1 if count <= 3 else 2
        for index in range(count):
            row, column = divmod(index, columns)
            plot = self.right.addPlot(row=row + 1, col=column)
            plot.setAspectLocked(True)
            plot.setXRange(-_VELOCITY_AXIS_LIMIT, _VELOCITY_AXIS_LIMIT,
                           padding=0.0)
            plot.setYRange(-_VELOCITY_AXIS_LIMIT, _VELOCITY_AXIS_LIMIT,
                           padding=0.0)
            plot.showGrid(x=True, y=True, alpha=0.18)
            plot.setLabel('bottom', 'X radial [m/s]')
            plot.setLabel('left', 'Y tangential [m/s]')
            plot.addLine(x=0.0, pen=pg.mkPen((185, 185, 185), width=1))
            plot.addLine(y=0.0, pen=pg.mkPen((185, 185, 185), width=1))

            color = _ACTIVE_COLORS[index % len(_ACTIVE_COLORS)]
            boundary = plot.plot([], [], pen=pg.mkPen(color, width=2))
            left_edge = plot.plot([], [], pen=None)
            unsafe = pg.FillBetweenItem(
                left_edge, boundary, brush=pg.mkBrush(_COL_UNSAFE))
            unsafe.setZValue(-10)
            plot.addItem(unsafe)
            relative = plot.plot([], [], pen=pg.mkPen(color, width=2))
            tip = plot.plot([], [], pen=None, symbol='o', symbolSize=7,
                            symbolBrush=color, symbolPen=pg.mkPen(color))
            self.velocity_cards.append({
                'plot': plot, 'boundary': boundary, 'left': left_edge,
                'relative': relative, 'tip': tip,
            })

    def refresh(self):
        snap = self.hub.snapshot()
        self._refresh_topdown(snap)
        self._refresh_velocity_cards(snap)
        self.frames_rendered += 1

    def _remove_dynamic_items(self):
        for item in self.dynamic_items:
            self.view.removeItem(item)
        self.dynamic_items = []

    def _add_obstacle_disc(self, x, y, radius):
        radius = max(radius, 0.02)
        disc = QtWidgets.QGraphicsEllipseItem(
            x - radius, y - radius, 2.0 * radius, 2.0 * radius)
        disc.setPen(pg.mkPen(_COL_OBS_EDGE, width=2))
        disc.setBrush(pg.mkBrush(_COL_OBS_FILL))
        disc.setZValue(-5)
        self.view.addItem(disc)
        self.dynamic_items.append(disc)

    def _refresh_topdown(self, snap):
        self._remove_dynamic_items()
        sample = snap['sample']
        odom = snap['odom']
        pose = None
        if odom is not None:
            pose = (odom['x'], odom['y'], odom['yaw'])
        elif sample is not None:
            pose = (sample.robot_x, sample.robot_y, sample.robot_phi)

        self.trail_curve.setData(*snap['trail'])
        if pose is None:
            self.robot_marker.setData([], [])
            self.heading_line.setData([], [])
            self.nominal_arrow.setData([], [])
            self.safe_arrow.setData([], [])
        else:
            x, y, yaw = pose
            self.robot_marker.setData([x], [y])
            self.heading_line.setData(
                [x, x + 0.4 * math.cos(yaw)],
                [y, y + 0.4 * math.sin(yaw)])
            if sample is not None:
                def command_arrow(command):
                    wx = command.sagittal * math.cos(yaw) - \
                         command.lateral * math.sin(yaw)
                    wy = command.sagittal * math.sin(yaw) + \
                         command.lateral * math.cos(yaw)
                    return [x, x + wx], [y, y + wy]
                self.nominal_arrow.setData(*command_arrow(sample.nominal))
                self.safe_arrow.setData(*command_arrow(sample.safe))
            else:
                self.nominal_arrow.setData([], [])
                self.safe_arrow.setData([], [])

        base_obstacles = snap['obstacles']
        if base_obstacles:
            for obstacle in base_obstacles:
                self._add_obstacle_disc(
                    obstacle['x'], obstacle['y'], obstacle['radius'])
        elif sample is not None:
            for obstacle in sample.obstacles:
                self._add_obstacle_disc(
                    obstacle.x, obstacle.y, obstacle.radius)

        if sample is not None:
            for index, obstacle in enumerate(sample.obstacles):
                color = _ACTIVE_COLORS[index % len(_ACTIVE_COLORS)]
                cx, cy = _circle(obstacle.x, obstacle.y,
                                 max(obstacle.radius, 0.02))
                rim = self.view.plot(cx, cy, pen=pg.mkPen(color, width=3))
                self.dynamic_items.append(rim)
                if pose is None:
                    continue
                x, y, _ = pose
                los = self.view.plot(
                    [x, obstacle.x], [y, obstacle.y],
                    pen=pg.mkPen(color, width=1, style=_DASH_LINE))
                self.dynamic_items.append(los)

                co = math.cos(obstacle.los_angle)
                si = math.sin(obstacle.los_angle)
                lateral = sample.world_parabola_lateral_limit
                scale = sample.velocity_arrow_seconds
                px, py = [], []
                for k in range(101):
                    local_y = -lateral + 2.0 * lateral * k / 100.0
                    local_x = (obstacle.boundary_vertex_x -
                               obstacle.boundary_curvature * local_y * local_y)
                    if local_x < -sample.world_parabola_backward_limit:
                        continue
                    px.append(x + scale * (co * local_x - si * local_y))
                    py.append(y + scale * (si * local_x + co * local_y))
                boundary = self.view.plot(px, py, pen=pg.mkPen(color, width=2))
                relative = self.view.plot(
                    [x, x + scale * obstacle.relative_velocity_x],
                    [y, y + scale * obstacle.relative_velocity_y],
                    pen=pg.mkPen(color, width=2))
                relative_tip = self.view.plot(
                    [x + scale * obstacle.relative_velocity_x],
                    [y + scale * obstacle.relative_velocity_y],
                    pen=None, symbol='o', symbolSize=6,
                    symbolBrush=color, symbolPen=pg.mkPen(color))
                self.dynamic_items.extend((boundary, relative, relative_tip))

        self._refresh_banner(snap)

    def _refresh_velocity_cards(self, snap):
        sample = snap['sample']
        obstacles = list(sample.obstacles) if sample is not None else []
        if len(obstacles) != self._velocity_card_count:
            self._build_velocity_cards(len(obstacles))
        for index, (card, obstacle) in enumerate(
                zip(self.velocity_cards, obstacles)):
            color = _ACTIVE_COLORS[index % len(_ACTIVE_COLORS)]
            ys = [-_VELOCITY_AXIS_LIMIT +
                  2.0 * _VELOCITY_AXIS_LIMIT * k / 100.0
                  for k in range(101)]
            xs = [max(-_VELOCITY_AXIS_LIMIT,
                      min(_VELOCITY_AXIS_LIMIT,
                          obstacle.boundary_vertex_x -
                          obstacle.boundary_curvature * y * y))
                  for y in ys]
            card['boundary'].setData(xs, ys)
            card['left'].setData([-_VELOCITY_AXIS_LIMIT] * len(ys), ys)
            co = math.cos(obstacle.los_angle)
            si = math.sin(obstacle.los_angle)
            rel_x = (co * obstacle.relative_velocity_x +
                     si * obstacle.relative_velocity_y)
            rel_y = (-si * obstacle.relative_velocity_x +
                     co * obstacle.relative_velocity_y)
            card['relative'].setData([0.0, rel_x], [0.0, rel_y])
            card['tip'].setData([rel_x], [rel_y])
            card['plot'].setTitle(
                f'#{index + 1}  id={obstacle.id}  '
                f'd={obstacle.distance:.2f} m  h={obstacle.h:.3f}',
                color=color, size='9pt')

    def _refresh_banner(self, snap):
        ages = snap['ages']
        stale_after = snap['stale_after_s']
        lines = []
        any_stale = False

        def describe(name, key, optional=False):
            nonlocal any_stale
            age = ages[key]
            if age is None:
                if optional:
                    return f'{name}: NO DATA (no publisher)'
                any_stale = True
                return f'{name}: NO DATA'
            if age > stale_after:
                any_stale = True
                return f'{name}: STALE {age:.1f}s'
            return f'{name}: ok {age * 1e3:.0f}ms'

        lines.append(describe('dpcbf/plot', 'plot', optional=True))
        lines.append(describe('odom', 'odom'))
        if snap['obstacles_available']:
            lines.append(describe('obstacles_safe', 'obstacles'))
        sample = snap['sample']
        if sample is not None:
            lines.append(
                f"mode={MODE_NAMES.get(sample.mode, '?')} "
                f"ctrl-staleness={STALENESS_NAMES.get(sample.staleness_state, '?')} "
                f"tick={sample.tick}")
        self.banner.setColor('#b72f3f' if any_stale else '#237a3b')
        self.banner.setText('\n'.join(lines))
        view_range = self.view.viewRange()
        self.banner.setPos(view_range[0][0], view_range[1][1])

    def run(self):
        self.win.show()
        run_event_loop = getattr(self.app, 'exec', None)
        if run_event_loop is None:
            run_event_loop = self.app.exec_
        return run_event_loop()
