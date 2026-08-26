"""Process-isolated real-time dashboard for prediction-based inverse TTC.

The simulation process deliberately imports only Python's standard library from
this module.  PyQtGraph, Qt, and NumPy are imported after the spawned child has
started so the optional dashboard cannot add GUI dependencies to headless runs.
"""

from collections import deque
import html
import math
import multiprocessing
import pickle
import queue
import threading
import time
import traceback


__all__ = ["NullRiskDashboard", "ProcessRiskDashboard"]


_CLOSE_MESSAGE = "__risk_dashboard_close__"
_DEFAULT_TITLE = "Prediction-based Inverse TTC"
_WINDOW_HORIZONTAL_PADDING_PX = 40
_SCREEN_WIDTH_FRACTION = 0.95
_SCREEN_HEIGHT_FRACTION = 0.90
_PLOT_TITLE_SIZE = "8pt"
_HEADER_TEXT_SIZE = "9pt"


def _positive_integer(name, value):
    try:
        result = int(value)
    except (TypeError, ValueError):
        raise ValueError("%s must be a positive integer" % name)
    if result <= 0 or result != value:
        raise ValueError("%s must be a positive integer" % name)
    return result


def _dashboard_window_size(column_width_px, max_pairs,
                           available_width_px=None,
                           available_height_px=None):
    """Return a two-column logical-pixel window size bounded by the screen."""
    width = 2 * int(column_width_px) + _WINDOW_HORIZONTAL_PADDING_PX
    height = max(420, min(900, 150 + 105 * int(max_pairs)))
    if available_width_px is not None and int(available_width_px) > 0:
        width = min(
            width,
            max(1, int(int(available_width_px) * _SCREEN_WIDTH_FRACTION)))
    if available_height_px is not None and int(available_height_px) > 0:
        height = min(
            height,
            max(1, int(int(available_height_px) * _SCREEN_HEIGHT_FRACTION)))
    return int(width), int(height)


def _compact_actor_id(actor_id, maximum_length=24):
    """Keep plot titles bounded while retaining a recognizable actor ID."""
    actor_id = str(actor_id)
    maximum_length = max(7, int(maximum_length))
    if len(actor_id) <= maximum_length:
        return actor_id
    retained = maximum_length - 3
    left_count = (retained + 1) // 2
    right_count = retained - left_count
    return "%s...%s" % (actor_id[:left_count], actor_id[-right_count:])


def _actor_plot_text(actor, distance_text, warmup, frame_status):
    """Build width-bounded titles plus a complete hover tooltip."""
    actor_id = str(actor["actor_id"])
    display_id = html.escape(_compact_actor_id(actor_id))
    mode_count = int(actor["evaluated_mode_count"])
    worst = float(actor["worst_inverse_ttc_s_inv"])
    expected = float(actor["expected_inverse_ttc_s_inv"])
    bar_title = (
        "%s<br>"
        "%s | %s<br>"
        "Worst %.3f | Expected %.3f" % (
            display_id, html.escape(str(distance_text)),
            html.escape(str(warmup)), worst, expected))
    history_title = "%s<br>Worst-risk history" % display_id
    tooltip = (
        "<b>Actor:</b> %s<br>"
        "<b>Distance:</b> %s<br>"
        "<b>History:</b> %s<br>"
        "<b>Status:</b> %s<br>"
        "<b>Worst inverse TTC (all %d modes):</b> %.3f s^-1<br>"
        "<b>Expected inverse TTC (all %d modes):</b> %.3f s^-1" % (
            html.escape(actor_id), html.escape(str(distance_text)),
            html.escape(str(warmup)), html.escape(str(frame_status)),
            mode_count, worst, mode_count, expected))
    return bar_title, history_title, tooltip


def _dashboard_status_text(sim_time_s, frame_status, actor_count):
    """Format the spanning header without one layout-expanding text line."""
    return (
        "Simulation time: %.2f s | Active actors: %d<br>Status: %s" % (
            float(sim_time_s), int(actor_count),
            html.escape(str(frame_status))))


def _finite_float(value, default=0.0, minimum=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    if minimum is not None:
        result = max(float(minimum), result)
    return result


def _nonnegative_values(values, count):
    if not isinstance(values, (list, tuple)):
        values = []
    result = [
        _finite_float(value, default=0.0, minimum=0.0)
        for value in values[:count]
    ]
    if len(result) < count:
        result.extend([0.0] * (count - len(result)))
    return result


def _normalise_actor(item, mode_count):
    """Convert one wire-format actor dictionary into safe primitive values."""
    if not isinstance(item, dict):
        return None
    actor_id = str(item.get("actor_id", ""))
    if not actor_id:
        return None

    inverse_ttc = _nonnegative_values(
        item.get("inverse_ttc_s_inv", []), mode_count)
    probabilities = _nonnegative_values(item.get("probabilities", []), mode_count)
    probability_sum = sum(probabilities)

    provided_worst = item.get("worst_inverse_ttc_s_inv")
    if provided_worst is None:
        worst = max(inverse_ttc) if inverse_ttc else 0.0
    else:
        worst = _finite_float(provided_worst, default=0.0, minimum=0.0)

    provided_expected = item.get("expected_inverse_ttc_s_inv")
    if provided_expected is None:
        if probability_sum > 0.0:
            expected = sum(
                risk * probability
                for risk, probability in zip(inverse_ttc, probabilities)
            ) / probability_sum
        elif inverse_ttc:
            expected = sum(inverse_ttc) / float(len(inverse_ttc))
        else:
            expected = 0.0
    else:
        expected = _finite_float(provided_expected, default=0.0, minimum=0.0)

    history_samples = max(0, int(_finite_float(
        item.get("history_samples", 0), default=0.0, minimum=0.0)))
    history_required = max(0, int(_finite_float(
        item.get("history_required", 0), default=0.0, minimum=0.0)))
    evaluated_mode_count = max(0, int(_finite_float(
        item.get("evaluated_mode_count", mode_count),
        default=mode_count, minimum=0.0)))
    displayed_mode_count = max(0, int(_finite_float(
        item.get("displayed_mode_count", mode_count),
        default=mode_count, minimum=0.0)))
    return {
        "actor_id": actor_id,
        "distance_m": _finite_float(item.get("distance_m"), default=None,
                                      minimum=0.0),
        "history_samples": history_samples,
        "history_required": history_required,
        "ready": bool(item.get("ready", False)),
        "evaluated_mode_count": evaluated_mode_count,
        "displayed_mode_count": displayed_mode_count,
        "inverse_ttc_s_inv": inverse_ttc,
        "probabilities": probabilities,
        "worst_inverse_ttc_s_inv": worst,
        "expected_inverse_ttc_s_inv": expected,
    }


def _normalise_frame(frame, mode_count):
    """Return a render-safe frame, ignoring malformed or duplicate actors."""
    if not isinstance(frame, dict) or frame.get("type") != "risk_frame":
        return None
    actors = []
    seen = set()
    source_actors = frame.get("actors", [])
    if not isinstance(source_actors, (list, tuple)):
        source_actors = []
    for item in source_actors:
        actor = _normalise_actor(item, mode_count)
        if actor is None or actor["actor_id"] in seen:
            continue
        seen.add(actor["actor_id"])
        actors.append(actor)
    return {
        "type": "risk_frame",
        "sim_time_s": _finite_float(frame.get("sim_time_s"), default=0.0,
                                     minimum=0.0),
        "status": str(frame.get("status", "")),
        "actors": actors,
    }


def _offer_latest(target_queue, item, attempts=3):
    """Offer an item without blocking, replacing an older queued frame."""
    for _ in range(max(1, int(attempts))):
        try:
            target_queue.put_nowait(item)
            return True
        except queue.Full:
            try:
                target_queue.get_nowait()
            except queue.Empty:
                pass
        except (EOFError, OSError, ValueError):
            return False
    try:
        target_queue.put_nowait(item)
        return True
    except (queue.Full, EOFError, OSError, ValueError):
        return False


def _send_startup(startup_queue, message):
    try:
        startup_queue.put(message, timeout=1.0)
    except (queue.Full, EOFError, OSError, ValueError):
        pass


def _dashboard_process_main(frame_queue, startup_queue, settings):
    """Spawn target.  All optional GUI/scientific imports stay in this child."""
    ready_sent = False
    try:
        try:
            import numpy as np
            import pyqtgraph as pg
            from pyqtgraph.Qt import QtCore, QtWidgets
        except Exception as exc:
            _send_startup(startup_queue, {
                "type": "error",
                "message": (
                    "Could not import the risk dashboard GUI dependencies. "
                    "Install pyqtgraph, NumPy, and a compatible Qt binding "
                    "(for example PyQt5) in the CARLA Python environment. "
                    "Original error: %s: %s" %
                    (exc.__class__.__name__, exc)
                ),
            })
            return

        max_pairs = int(settings["max_pairs"])
        mode_count = int(settings["mode_count"])
        column_width_px = int(settings["column_width_px"])
        history_length = int(settings["history_length"])
        title = str(settings["title"])

        app = QtWidgets.QApplication.instance()
        if app is None:
            app = QtWidgets.QApplication([])
        app.setQuitOnLastWindowClosed(True)

        window = pg.GraphicsLayoutWidget(title=title)
        status_label = pg.LabelItem(justify="left")
        window.addItem(status_label, row=0, col=0, colspan=2)

        bar_plots = []
        bar_items = []
        line_plots = []
        line_curves = []
        zero_modes = np.zeros(mode_count, dtype=float)
        zero_history = np.zeros(history_length, dtype=float)
        history_x = np.arange(-history_length + 1, 1, dtype=float)

        for row_index in range(max_pairs):
            layout_row = row_index + 1
            bar_plot = window.addPlot(
                row=layout_row, col=0, title="No active actor")
            bar_plot.setLabel("bottom", "Prediction mode")
            bar_plot.setLabel("left", "Inverse TTC", units="s^-1")
            bar_plot.setXRange(-0.5, mode_count - 0.5, padding=0.0)
            bar_plot.setMouseEnabled(x=False, y=False)
            bar_item = pg.BarGraphItem(
                x=np.arange(mode_count), height=zero_modes.copy(), width=0.65,
                brush=pg.mkBrush(70, 150, 255, 190),
                pen=pg.mkPen(115, 190, 255, 230))
            bar_plot.addItem(bar_item)

            line_plot = window.addPlot(
                row=layout_row, col=1, title="Worst inverse TTC history")
            line_plot.setLabel("bottom", "Recent update")
            line_plot.setLabel("left", "Worst inverse TTC", units="s^-1")
            line_plot.setXRange(history_x[0], history_x[-1], padding=0.0)
            line_plot.setMouseEnabled(x=False, y=False)
            line_curve = line_plot.plot(
                history_x, zero_history.copy(), pen=pg.mkPen(255, 215, 70, width=2))

            bar_plots.append(bar_plot)
            bar_items.append(bar_item)
            line_plots.append(line_plot)
            line_curves.append(line_curve)

        # Titles and axes participate in the graphics-layout size hint. Equal
        # stretch plus a post-construction resize keeps both columns visible
        # instead of allowing the first column to consume the whole viewport.
        graphics_grid = window.ci.layout
        graphics_grid.setColumnPreferredWidth(0, column_width_px)
        graphics_grid.setColumnPreferredWidth(1, column_width_px)
        graphics_grid.setColumnStretchFactor(0, 1)
        graphics_grid.setColumnStretchFactor(1, 1)

        screen_getter = getattr(window, "screen", None)
        screen = screen_getter() if callable(screen_getter) else None
        if screen is None:
            primary_screen = getattr(app, "primaryScreen", None)
            screen = primary_screen() if callable(primary_screen) else None
        available_width = None
        available_height = None
        if screen is not None:
            available = screen.availableGeometry()
            available_width = available.width()
            available_height = available.height()
        elif hasattr(app, "desktop"):
            available = app.desktop().availableGeometry(window)
            available_width = available.width()
            available_height = available.height()
        window_width, window_height = _dashboard_window_size(
            column_width_px, max_pairs,
            available_width_px=available_width,
            available_height_px=available_height)
        window.resize(window_width, window_height)

        histories = {}
        last_seen = {}
        generation = [0]

        def set_row_empty(row_index):
            bar_plots[row_index].setTitle(
                "No active actor", size=_PLOT_TITLE_SIZE)
            bar_plots[row_index].setToolTip("")
            bar_items[row_index].setOpts(height=zero_modes)
            line_plots[row_index].setTitle(
                "Worst inverse TTC history", size=_PLOT_TITLE_SIZE)
            line_plots[row_index].setToolTip("")
            line_curves[row_index].setData(history_x, zero_history)

        def render_frame(frame):
            clean = _normalise_frame(frame, mode_count)
            if clean is None:
                return
            generation[0] += 1
            current_generation = generation[0]
            frame_status = clean["status"] or "running"
            status_label.setText(
                _dashboard_status_text(
                    clean["sim_time_s"], frame_status, len(clean["actors"])),
                size=_HEADER_TEXT_SIZE)

            for actor in clean["actors"]:
                actor_id = actor["actor_id"]
                history = histories.get(actor_id)
                if history is None:
                    history = deque(maxlen=history_length)
                    histories[actor_id] = history
                history.append(actor["worst_inverse_ttc_s_inv"])
                last_seen[actor_id] = current_generation

            # Preserve brief dropouts while bounding state for long actor churn.
            stale_before = current_generation - history_length
            for actor_id in list(histories):
                if last_seen.get(actor_id, current_generation) < stale_before:
                    del histories[actor_id]
                    last_seen.pop(actor_id, None)

            active = clean["actors"][:max_pairs]
            for row_index in range(max_pairs):
                if row_index >= len(active):
                    set_row_empty(row_index)
                    continue

                actor = active[row_index]
                actor_id = actor["actor_id"]
                distance = actor["distance_m"]
                distance_text = "--" if distance is None else "%.1f m" % distance
                if actor["ready"]:
                    warmup = "ready"
                else:
                    warmup = "warmup %d/%d" % (
                        actor["history_samples"], actor["history_required"])
                bar_title, history_title, tooltip = _actor_plot_text(
                    actor, distance_text, warmup, frame_status)
                bar_plots[row_index].setTitle(
                    bar_title, size=_PLOT_TITLE_SIZE)
                bar_plots[row_index].setToolTip(tooltip)
                bar_items[row_index].setOpts(
                    height=np.asarray(actor["inverse_ttc_s_inv"], dtype=float))

                values = list(histories[actor_id])
                padded = [0.0] * (history_length - len(values)) + values
                line_plots[row_index].setTitle(
                    history_title, size=_PLOT_TITLE_SIZE)
                line_plots[row_index].setToolTip(tooltip)
                line_curves[row_index].setData(
                    history_x, np.asarray(padded, dtype=float))

        should_close = [False]

        def poll_queue():
            latest = None
            try:
                while True:
                    item = frame_queue.get_nowait()
                    if item == _CLOSE_MESSAGE:
                        should_close[0] = True
                    else:
                        latest = item
            except queue.Empty:
                pass
            except (EOFError, OSError, ValueError):
                should_close[0] = True

            if should_close[0]:
                window.close()
                app.quit()
                return
            if latest is not None:
                try:
                    frame = pickle.loads(latest) if isinstance(
                        latest, (bytes, bytearray)) else latest
                    render_frame(frame)
                except Exception:
                    # A malformed frame must not take down the GUI event loop.
                    status_label.setText(
                        "Dashboard ignored an invalid risk frame.<br>%s" %
                        html.escape(traceback.format_exc().splitlines()[-1]),
                        size=_HEADER_TEXT_SIZE)

        timer = QtCore.QTimer()
        timer.timeout.connect(poll_queue)
        timer.start(25)
        app.lastWindowClosed.connect(app.quit)
        window.show()
        app.processEvents()

        _send_startup(startup_queue, {"type": "ready"})
        ready_sent = True
        execute = getattr(app, "exec", None)
        if execute is None:
            execute = app.exec_
        execute()
        timer.stop()
    except BaseException as exc:
        if not ready_sent:
            _send_startup(startup_queue, {
                "type": "error",
                "message": (
                    "The risk dashboard GUI could not start. Verify desktop "
                    "display access and the Qt platform plugins in the CARLA "
                    "Python environment. Original error: %s: %s\n%s" % (
                        exc.__class__.__name__, exc, traceback.format_exc())
                ),
            })


def _close_queue(target_queue):
    if target_queue is None:
        return
    try:
        # Never let a feeder thread block shutdown if the GUI was closed by the
        # user while a frame was still in transit.
        target_queue.cancel_join_thread()
    except (AttributeError, OSError, ValueError):
        pass
    try:
        target_queue.close()
    except (AttributeError, OSError, ValueError):
        pass


class NullRiskDashboard(object):
    """No-op dashboard with the same interface as ``ProcessRiskDashboard``."""

    def __init__(self, max_pairs=3, mode_count=5, history_length=200,
                 title=_DEFAULT_TITLE, startup_timeout=10.0,
                 column_width_px=450):
        del max_pairs, mode_count, history_length, title, startup_timeout
        del column_width_px

    @property
    def alive(self):
        return False

    def publish(self, frame_dict):
        del frame_dict
        return False

    def close(self):
        return None


class ProcessRiskDashboard(object):
    """Render risk frames in an isolated spawned GUI process.

    Construction waits for a child ready/error handshake.  ``publish`` is
    non-blocking and retains at most the newest frame, so dashboard rendering
    can never impose backpressure on the simulation tick.
    """

    def __init__(self, max_pairs=3, mode_count=5, history_length=200,
                 title=_DEFAULT_TITLE, startup_timeout=10.0,
                 column_width_px=450):
        self.max_pairs = _positive_integer("max_pairs", max_pairs)
        self.mode_count = _positive_integer("mode_count", mode_count)
        self.column_width_px = _positive_integer(
            "column_width_px", column_width_px)
        self.history_length = _positive_integer("history_length", history_length)
        self.title = str(title)
        self.startup_timeout = _finite_float(startup_timeout, default=-1.0)
        if self.startup_timeout <= 0.0:
            raise ValueError("startup_timeout must be positive")

        self._lock = threading.RLock()
        self._closed = False
        self._context = None
        self._frame_queue = None
        self._startup_queue = None
        self._process = None
        settings = {
            "max_pairs": self.max_pairs,
            "mode_count": self.mode_count,
            "column_width_px": self.column_width_px,
            "history_length": self.history_length,
            "title": self.title,
        }
        try:
            self._context = multiprocessing.get_context("spawn")
            self._frame_queue = self._context.Queue(maxsize=1)
            self._startup_queue = self._context.Queue(maxsize=1)
            self._process = self._context.Process(
                target=_dashboard_process_main,
                args=(self._frame_queue, self._startup_queue, settings),
                name="inverse-ttc-dashboard",
            )
            self._process.daemon = True
        except Exception as exc:
            self._closed = True
            _close_queue(self._frame_queue)
            _close_queue(self._startup_queue)
            raise RuntimeError(
                "Could not initialize the risk dashboard spawn process and "
                "IPC queues. Check OS process permissions and security "
                "software. Original error: %s: %s" %
                (exc.__class__.__name__, exc))

        try:
            self._process.start()
        except Exception as exc:
            self._closed = True
            _close_queue(self._frame_queue)
            _close_queue(self._startup_queue)
            raise RuntimeError(
                "Could not start the risk dashboard process. On Windows, "
                "construct it from code reached through an "
                "'if __name__ == \"__main__\"' guard. Original error: %s: %s" %
                (exc.__class__.__name__, exc))

        try:
            message = self._wait_for_startup()
            if not isinstance(message, dict) or message.get("type") != "ready":
                detail = (message.get("message") if isinstance(message, dict)
                          else repr(message))
                raise RuntimeError(detail or "dashboard child returned no startup detail")
        except Exception as exc:
            self._abort_startup()
            if isinstance(exc, RuntimeError):
                raise RuntimeError(str(exc))
            raise RuntimeError("Risk dashboard startup failed: %s" % exc)

    def _wait_for_startup(self):
        deadline = time.monotonic() + self.startup_timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                exit_code = self._process.exitcode
                suffix = ("; child exit code %s" % exit_code
                          if exit_code is not None else "")
                raise RuntimeError(
                    "Risk dashboard did not report ready within %.1f seconds%s. "
                    "Check pyqtgraph/Qt installation, desktop display access, "
                    "and Qt platform plugins." % (self.startup_timeout, suffix))
            try:
                return self._startup_queue.get(timeout=min(0.1, remaining))
            except queue.Empty:
                if not self._process.is_alive():
                    raise RuntimeError(
                        "Risk dashboard process exited before its startup "
                        "handshake (exit code %s). Check pyqtgraph/Qt and the "
                        "desktop display environment." % self._process.exitcode)
            except (EOFError, OSError, ValueError) as exc:
                raise RuntimeError(
                    "Risk dashboard startup channel failed: %s" % exc)

    def _abort_startup(self):
        self._closed = True
        if self._process is not None:
            self._process.join(timeout=0.5)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=2.0)
        _close_queue(self._frame_queue)
        _close_queue(self._startup_queue)

    @property
    def alive(self):
        with self._lock:
            return (not self._closed and self._process is not None and
                    self._process.is_alive())

    def publish(self, frame_dict):
        """Publish one frame without waiting; an older pending frame is dropped."""
        with self._lock:
            if (self._closed or self._process is None or
                    not self._process.is_alive()):
                return False

        if not isinstance(frame_dict, dict):
            raise TypeError("risk dashboard frame must be a dictionary")
        if frame_dict.get("type") != "risk_frame":
            raise ValueError("risk dashboard frame type must be 'risk_frame'")
        try:
            payload = pickle.dumps(frame_dict, protocol=pickle.HIGHEST_PROTOCOL)
        except Exception as exc:
            raise TypeError(
                "risk dashboard frame must contain multiprocessing-serializable "
                "values: %s" % exc)

        with self._lock:
            if (self._closed or self._process is None or
                    not self._process.is_alive()):
                return False
            return _offer_latest(self._frame_queue, payload)

    def close(self):
        """Ask the GUI to close, then terminate only if graceful exit times out."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            process = self._process
            if process is not None and process.is_alive():
                _offer_latest(self._frame_queue, _CLOSE_MESSAGE, attempts=10)

        if process is not None:
            process.join(timeout=2.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=2.0)

        with self._lock:
            _close_queue(self._frame_queue)
            _close_queue(self._startup_queue)
