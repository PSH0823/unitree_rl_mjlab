"""matplotlib fallback backend (used when pyqtgraph/Qt is unavailable).

Same DataHub, reduced polish: top-down view + three time-series axes,
FuncAnimation at ~10 Hz. Interaction (pan/zoom) is matplotlib's default
toolbar; the stale banner behaves like the pyqtgraph one.
"""
import math

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle

from .data_hub import MODE_NAMES, STALENESS_NAMES


class PlotAppMpl:
    def __init__(self, hub, gui_rate_hz: float = 10.0, trail_s: float = 30.0):
        self.hub = hub
        self.fig = plt.figure('DPCBF live plot (read-only client)',
                              figsize=(15, 8))
        grid = self.fig.add_gridspec(4, 2, width_ratios=[5, 4])
        self.ax_view = self.fig.add_subplot(grid[:, 0])
        self.ax_cmd = self.fig.add_subplot(grid[0, 1])
        self.ax_int = self.fig.add_subplot(grid[1, 1], sharex=self.ax_cmd)
        self.ax_h = self.fig.add_subplot(grid[2, 1], sharex=self.ax_cmd)
        self.ax_age = self.fig.add_subplot(grid[3, 1], sharex=self.ax_cmd)

        self.ax_view.set_aspect('equal')
        self.ax_view.grid(alpha=0.3)
        self.ax_view.set_xlabel('x [m] (odom)')
        self.ax_view.set_ylabel('y [m] (odom)')
        (self.trail_line,) = self.ax_view.plot([], [], '-', lw=0.8,
                                               color='#5a82aa')
        (self.robot_dot,) = self.ax_view.plot([], [], 'o', ms=8,
                                              color='#50c8ff')
        (self.heading_line,) = self.ax_view.plot([], [], '-', lw=2,
                                                 color='#50c8ff')
        (self.nom_arrow,) = self.ax_view.plot([], [], '--', lw=1.5,
                                              color='#78b4ff',
                                              label='nominal cmd')
        (self.safe_arrow,) = self.ax_view.plot([], [], '-', lw=2.5,
                                               color='#ffaa3c',
                                               label='safe cmd')
        self.ax_view.legend(loc='lower right', fontsize=8)
        self.banner = self.ax_view.text(
            0.01, 0.99, '', transform=self.ax_view.transAxes,
            va='top', ha='left', fontsize=8, family='monospace')
        self._patches = []

        self.ax_cmd.set_ylabel('cmd [m/s, rad/s]')
        (self.l_nom_sag,) = self.ax_cmd.plot([], [], color='#78b4ff',
                                             label='nominal sag')
        (self.l_safe_sag,) = self.ax_cmd.plot([], [], color='#ffaa3c',
                                              label='safe sag')
        (self.l_nom_yaw,) = self.ax_cmd.plot([], [], ':', color='#78b4ff',
                                             label='nominal yaw')
        (self.l_safe_yaw,) = self.ax_cmd.plot([], [], ':', color='#ffaa3c',
                                              label='safe yaw')
        self.ax_cmd.legend(fontsize=7, ncol=2)
        self.ax_cmd.grid(alpha=0.3)

        self.ax_int.set_ylabel('interv / scale')
        self.ax_int.set_ylim(-0.05, 1.15)
        (self.l_interv,) = self.ax_int.plot([], [], color='#ff5050',
                                            label='intervention')
        (self.l_scale,) = self.ax_int.plot([], [], color='#a0a0a0',
                                           label='command_scale')
        self.ax_int.legend(fontsize=7)
        self.ax_int.grid(alpha=0.3)

        self.ax_h.set_ylabel('h / clear [m]')
        (self.l_min_h,) = self.ax_h.plot([], [], color='#82dc82',
                                         label='min h')
        (self.l_clear,) = self.ax_h.plot([], [], color='#c8c878',
                                         label='min clearance')
        self.ax_h.axhline(0.0, color='#ff5050', lw=0.8, ls='--')
        self.ax_h.legend(fontsize=7)
        self.ax_h.grid(alpha=0.3)

        self.ax_age.set_ylabel('age [s]')
        self.ax_age.set_xlabel('seconds ago')
        (self.l_age,) = self.ax_age.plot([], [], color='#c88cff',
                                         label='obstacle age')
        (self.l_lat,) = self.ax_age.plot([], [], color='#78c8dc',
                                         label='plot latency')
        self.ax_age.legend(fontsize=7)
        self.ax_age.grid(alpha=0.3)
        self.ax_age.set_xlim(-30.0, 0.0)

        self.frames_rendered = 0
        self._anim = FuncAnimation(
            self.fig, self._on_frame,
            interval=max(50, int(1000.0 / gui_rate_hz)),
            cache_frame_data=False)

    # ------------------------------------------------------------------
    def _on_frame(self, _frame):
        snap = self.hub.snapshot()
        s = snap['series']
        self.l_nom_sag.set_data(*s['nominal_sagittal'])
        self.l_safe_sag.set_data(*s['safe_sagittal'])
        self.l_nom_yaw.set_data(*s['nominal_yaw'])
        self.l_safe_yaw.set_data(*s['safe_yaw'])
        self.l_interv.set_data(*s['intervention'])
        self.l_scale.set_data(*s['command_scale'])
        self.l_min_h.set_data(*s['min_h'])
        self.l_clear.set_data(*s['min_clearance'])
        self.l_age.set_data(*s['obstacle_age'])
        self.l_lat.set_data(*s['latency'])
        for ax in (self.ax_cmd, self.ax_h, self.ax_age):
            ax.relim()
            ax.autoscale_view(scalex=False)

        for p in self._patches:
            p.remove()
        self._patches = []

        self.trail_line.set_data(*snap['trail'])
        pose = None
        if snap['odom'] is not None:
            o = snap['odom']
            pose = (o['x'], o['y'], o['yaw'])
        elif snap['sample'] is not None:
            m = snap['sample']
            pose = (m.robot_x, m.robot_y, m.robot_phi)
        sample = snap['sample']
        if pose is not None:
            x, y, yaw = pose
            self.robot_dot.set_data([x], [y])
            self.heading_line.set_data([x, x + 0.4 * math.cos(yaw)],
                                       [y, y + 0.4 * math.sin(yaw)])
            if sample is not None:
                def arrow(cmd):
                    wx = cmd.sagittal * math.cos(yaw) - \
                        cmd.lateral * math.sin(yaw)
                    wy = cmd.sagittal * math.sin(yaw) + \
                        cmd.lateral * math.cos(yaw)
                    return [x, x + wx], [y, y + wy]
                self.nom_arrow.set_data(*arrow(sample.nominal))
                self.safe_arrow.set_data(*arrow(sample.safe))
        if sample is not None:
            for o in sample.obstacles:
                color = '#ff5050' if o.h < 0.0 else '#ff7878'
                c = Circle((o.x, o.y), max(o.radius, 0.02), fill=False,
                           color=color, lw=1.5)
                self.ax_view.add_patch(c)
                self._patches.append(c)
        if snap['obstacles']:
            # See plot_app.py: with no control sample this is the ONLY
            # obstacle layer, so it must not be drawn as a faint overlay.
            hw_only = sample is None
            alpha, lw = (0.9, 1.5) if hw_only else (0.35, 0.8)
            for o in snap['obstacles']:
                c = Circle((o['x'], o['y']), max(o['radius'], 0.02),
                           fill=False, color='#ff3c3c', alpha=alpha, lw=lw)
                self.ax_view.add_patch(c)
                self._patches.append(c)
                if o['vx'] or o['vy']:
                    (v,) = self.ax_view.plot(
                        [o['x'], o['x'] + o['vx']],
                        [o['y'], o['y'] + o['vy']],
                        color='#ff3c3c', alpha=alpha, lw=1.0, ls='--')
                    self._patches.append(v)
        self.ax_view.relim()
        self.ax_view.autoscale_view()

        ages = snap['ages']
        stale_after = snap['stale_after_s']
        lines = []
        any_stale = False
        for name, key in (('dpcbf/plot', 'plot'), ('odom', 'odom'),
                          ('obstacles_safe', 'obstacles')):
            if key == 'obstacles' and not snap['obstacles_available']:
                continue
            age = ages[key]
            if age is None:
                lines.append(f'{name}: NO DATA')
                any_stale = True
            elif age > stale_after:
                lines.append(f'{name}: STALE {age:.1f}s')
                any_stale = True
            else:
                lines.append(f'{name}: ok {age * 1e3:.0f}ms')
        if sample is not None:
            lines.append(
                f"mode={MODE_NAMES.get(sample.mode, '?')} "
                f"ctrl-staleness="
                f"{STALENESS_NAMES.get(sample.staleness_state, '?')}")
        self.banner.set_text('\n'.join(lines))
        self.banner.set_color('#ff5050' if any_stale else '#8cdc8c')
        self.frames_rendered += 1
        return []

    def run(self):
        plt.show()
        return 0
