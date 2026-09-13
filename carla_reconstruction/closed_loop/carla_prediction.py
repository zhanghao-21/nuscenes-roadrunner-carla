"""Optional, read-only HGT display adapter for the CARLA-only runtime.

Keep model/NumPy/Qt imports behind the enable switch. The shared prediction
monitor owns inference and drawing; this adapter owns CLI settings, filtering,
failure isolation and cleanup, never vehicle control or simulation ticks.
"""

import copy
import json
import os
import time


def add_prediction_arguments(parser):
    master = parser.add_mutually_exclusive_group()
    master.add_argument(
        "--prediction-risk", "--enable-prediction-risk", dest="prediction_risk",
        action="store_true", help="enable HGT risk prediction and its dashboard")
    master.add_argument(
        "--no-prediction-risk", "--disable-prediction-risk", dest="prediction_risk",
        action="store_false", help="disable all HGT inference, dashboard and drawing")
    dashboard = parser.add_mutually_exclusive_group()
    dashboard.add_argument(
        "--prediction-dashboard", dest="prediction_dashboard", action="store_true",
        help="show the inverse-TTC dashboard (also enables prediction)")
    dashboard.add_argument(
        "--no-prediction-dashboard", dest="prediction_dashboard", action="store_false",
        help="hide the dashboard without disabling requested trajectory drawing")
    drawing = parser.add_mutually_exclusive_group()
    drawing.add_argument(
        "--draw-predictions", dest="draw_predictions", action="store_true",
        help="draw predicted trajectories in CARLA (also enables prediction)")
    drawing.add_argument(
        "--no-draw-predictions", dest="draw_predictions", action="store_false",
        help="disable CARLA trajectory lines without disabling the dashboard")
    parser.set_defaults(prediction_risk=None, prediction_dashboard=None,
                        draw_predictions=None)
    parser.add_argument("--prediction-device", help="HGT Torch device: auto, cpu, cuda, cuda:0")
    parser.add_argument(
        "--prediction-config", help="optional JSON with a prediction_risk section "
        "(same schema as hybrid_config.json); no SUMO preparation required")


def _normalise_settings(config, path, enabled_override, device_override):
    from carla_reconstruction.closed_loop.prediction_risk import prediction_risk_settings
    return prediction_risk_settings(
        config, path, enabled_override=enabled_override, device_override=device_override)


def _create_monitor(settings, step_length):
    from carla_reconstruction.closed_loop.prediction_risk import PredictionRiskMonitor
    return PredictionRiskMonitor(settings, step_length)


class CarlaPredictionObserver:
    """Guard optional visualization so failures cannot interrupt CARLA control."""

    def __init__(self, args, manifest_path, moving_track_ids, excluded_vehicle_ids):
        self.monitor = None
        self.settings = None
        self.step_length = None
        self._closed = False
        self._moving_ids = set(moving_track_ids)
        self._audit = {
            "requested": False, "started": False, "active": False, "closed": False,
            "observed_ticks": 0, "risk_frames": 0, "ready_frames": 0,
            "drawing_frames": 0,
            "actor_filter": {
                "policy": "recorded_moving_vehicles_only",
                "eligible_track_ids": sorted(self._moving_ids),
                "excluded_static_vehicle_track_ids": sorted(excluded_vehicle_ids),
                "pedestrians_excluded": True,
                "keep_temporarily_stopped_moving_tracks": True,
            },
        }
        master = getattr(args, "prediction_risk", None)
        # Explicit disable wins even over an enabled config or visual switches.
        # Do not read config or import optional dependencies on this fast path.
        if master is False:
            return
        dashboard = getattr(args, "prediction_dashboard", None)
        drawing = getattr(args, "draw_predictions", None)
        self._audit["requested"] = bool(master or dashboard or drawing)
        try:
            path = getattr(args, "prediction_config", None)
            config = {}
            if path:
                path = os.path.abspath(path)
                with open(path, encoding="utf-8-sig") as stream:
                    config = json.load(stream)
                if not isinstance(config, dict) or not isinstance(
                        config.get("prediction_risk"), dict):
                    raise ValueError("--prediction-config must contain a prediction_risk object")
                self._audit["config"] = path
            else:
                path = manifest_path
            raw = config.get("prediction_risk", {})
            requested = self._audit["requested"] or bool(raw.get("enabled", False))
            self._audit["requested"] = requested
            if not requested:
                return
            # Use exactly the hybrid defaults/model/heading correction. The
            # optional file is read-only and relative model paths stay relative
            # to that file, never the current working directory.
            self.settings = _normalise_settings(
                config, path, True, getattr(args, "prediction_device", None))
            if dashboard is not None:
                self.settings["dashboard"]["enabled"] = dashboard
            if drawing is not None:
                self.settings["drawing"]["enabled"] = drawing
            self._audit["settings"] = copy.deepcopy(self.settings)
        except Exception as exc:
            self._fail("configuration", exc)

    def _fail(self, stage, exc):
        self._audit.update(active=False, error=str(exc), error_stage=stage)
        print("WARNING: CARLA-only prediction risk %s failed; "
              "continuing simulation without the observer: %s" % (stage, exc))
        self.close()

    def start(self, step_length):
        if self.settings is None or self._closed or self.monitor is not None:
            return
        try:
            self.step_length = float(step_length)
            self.monitor = _create_monitor(self.settings, self.step_length)
            self._audit.update(self.monitor.metadata())
            self._audit.update(started=True, active=True)
            print("Prediction risk: HGT started; dashboard=%s, trajectory lines=%s; "
                  "recorded moving vehicles only" % (
                      "on" if self.settings["dashboard"]["enabled"] else "off",
                      "on" if self.settings["drawing"]["enabled"] else "off"))
        except Exception as exc:
            self._fail("startup", exc)

    def observe(self, sim_time, ego_state, others, ego_actor, actors, world):
        if self.monitor is None:
            return
        try:
            # Copy states: the existing metrics retain their original full
            # population, including pedestrians and parked/single-frame cars.
            prediction_ego = dict(ego_state)
            prediction_ego["z"] = float(ego_actor.get_location().z)
            prediction_others = {}
            for actor_id, state in others.items():
                if actor_id not in self._moving_ids:
                    continue
                actor = actors.get(actor_id)
                if actor is None or not actor.is_alive:
                    continue
                prediction_others[actor_id] = dict(state)
                prediction_others[actor_id]["z"] = float(actor.get_location().z)
            frame = self.monitor.observe(
                sim_time, prediction_ego, prediction_others, world=world)
            self._audit["observed_ticks"] += 1
            if frame is not None:
                self._audit["risk_frames"] += 1
                self._audit["last_frame"] = copy.deepcopy(frame)
                if any(row.get("ready", False) for row in frame.get("actors", [])):
                    self._audit["ready_frames"] += 1
                    if self.settings["drawing"]["enabled"] and world is not None:
                        self._audit["drawing_frames"] += 1
        except Exception as exc:
            self._fail("observation", exc)

    def pace(self, wall_start):
        # This is wall-clock pacing only. The main runner remains the sole tick
        # owner; inference latency can slow wall time but cannot advance CARLA.
        try:
            if self.monitor is not None and self.monitor.realtime_pacing:
                remaining = self.step_length - (time.monotonic() - wall_start)
                if remaining > 0.0:
                    time.sleep(remaining)
        except Exception as exc:
            self._fail("pacing", exc)

    def metadata(self):
        return copy.deepcopy(self._audit)

    def close(self):
        if self._closed:
            return
        self._closed = True
        monitor, self.monitor = self.monitor, None
        try:
            if monitor is not None:
                monitor.close()
        except Exception as exc:
            self._audit["cleanup_error"] = str(exc)
            print("WARNING: prediction observer cleanup failed:", exc)
        finally:
            self._audit.update(active=False, closed=True)
