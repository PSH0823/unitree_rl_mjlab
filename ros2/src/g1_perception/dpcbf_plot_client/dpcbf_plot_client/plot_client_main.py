"""Entry point: pick a GUI backend, spin the DataHub, run the window.

    ros2 run dpcbf_plot_client dpcbf_plot_client
    ros2 run dpcbf_plot_client dpcbf_plot_client --ros-args \
        -p backend:=matplotlib -p window_s:=60.0 -p gui_rate_hz:=20.0

backend:=auto (default) tries pyqtgraph first and falls back to matplotlib;
either can be forced. The choice is announced once so a session log records
which renderer produced the screenshots.
"""
import sys

from .data_hub import HubRunner


def _make_app(backend: str, hub, gui_rate_hz: float, logger):
    if backend in ('auto', 'pyqtgraph'):
        try:
            from .plot_app import PlotApp
            logger.info('plot backend: pyqtgraph')
            return PlotApp(hub, gui_rate_hz=gui_rate_hz)
        except ImportError as exc:
            if backend == 'pyqtgraph':
                raise RuntimeError(
                    f'backend:=pyqtgraph requested but unavailable: {exc}'
                ) from exc
            logger.warning(f'pyqtgraph unavailable ({exc}); '
                           'falling back to matplotlib')
    from .plot_app_mpl import PlotAppMpl
    logger.info('plot backend: matplotlib')
    return PlotAppMpl(hub, gui_rate_hz=min(gui_rate_hz, 15.0))


def main(args=None):
    runner = HubRunner(args=args)
    hub = runner.start()
    backend = hub.declare_parameter('backend', 'auto').value
    gui_rate_hz = float(hub.declare_parameter('gui_rate_hz', 25.0).value)
    if backend not in ('auto', 'pyqtgraph', 'matplotlib'):
        hub.get_logger().error(f'unknown backend {backend!r} '
                               '(auto|pyqtgraph|matplotlib)')
        runner.shutdown()
        return 2
    try:
        app = _make_app(backend, hub, gui_rate_hz, hub.get_logger())
        return app.run()
    except KeyboardInterrupt:
        return 0
    finally:
        runner.shutdown()


if __name__ == '__main__':
    sys.exit(main())
