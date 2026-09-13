"""Optional HGT trajectory prediction and prediction-based inverse TTC.

This module is deliberately independent of CARLA and SUMO at import time.  The
closed-loop runners feed it plain actor-state dictionaries after each synchronized
tick.  Heavy model and GUI dependencies are imported only when this optional
feature is enabled.
"""

from collections import deque
import math
import os

import numpy as np


EGO_HISTORY_ID = "__ego__"


def _mapping(value, name):
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("%s must be a JSON object" % name)
    return dict(value)


def _positive_float(value, name):
    if isinstance(value, bool):
        raise ValueError("%s must be a positive finite number" % name)
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("%s must be a positive finite number" % name)
    return value


def _positive_int(value, name, minimum=1):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("%s must be an integer >= %d" % (name, minimum))
    if value < minimum:
        raise ValueError("%s must be an integer >= %d" % (name, minimum))
    return value


def _output_frame(value, name):
    value = str(value).strip().lower().replace("-", "_")
    if value not in ("actor_heading", "world"):
        raise ValueError("%s must be 'actor_heading' or 'world'" % name)
    return value


def _resolve_path(value, default_path, config_directory):
    path = default_path if value in (None, "") else os.fspath(value)
    if not os.path.isabs(path):
        path = os.path.join(config_directory, path)
    return os.path.abspath(path)


def prediction_risk_settings(hybrid_config, hybrid_config_path,
                             enabled_override=None, device_override=None):
    """Normalize and validate the optional ``prediction_risk`` JSON section."""
    raw = _mapping(hybrid_config.get("prediction_risk"), "prediction_risk")
    model = _mapping(raw.get("model"), "prediction_risk.model")
    dashboard = _mapping(raw.get("dashboard"), "prediction_risk.dashboard")
    drawing = _mapping(raw.get("drawing"), "prediction_risk.drawing")

    enabled = bool(raw.get("enabled", False))
    if enabled_override is not None:
        enabled = bool(enabled_override)

    package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    hgt_root = os.path.join(package_root, "HGT_model")
    config_directory = os.path.dirname(os.path.abspath(hybrid_config_path))
    model_config = _resolve_path(
        model.get("config"), os.path.join(hgt_root, "configs", "mart_carla.yaml"),
        config_directory)
    bundled_checkpoint = os.path.abspath(os.path.join(
        hgt_root, "checkpoints", "mart_carla", "carla_ckpt_best.pth"))
    checkpoint = _resolve_path(
        model.get("checkpoint"), bundled_checkpoint, config_directory)
    default_output_frame = "world"
    if os.path.normcase(checkpoint) == os.path.normcase(bundled_checkpoint):
        default_output_frame = "actor_heading"

    device = str(device_override or model.get("device", "auto"))
    if not device:
        raise ValueError("prediction_risk.model.device cannot be empty")

    settings = {
        "enabled": enabled,
        "pace_realtime": bool(raw.get("pace_realtime", True)),
        "update_interval_s": _positive_float(
            raw.get("update_interval_s", 0.10),
            "prediction_risk.update_interval_s"),
        "sample_step_s": _positive_float(
            raw.get("sample_step_s", 0.05),
            "prediction_risk.sample_step_s"),
        "neighbor_radius_m": _positive_float(
            raw.get("neighbor_radius_m", 50.0),
            "prediction_risk.neighbor_radius_m"),
        "max_neighbors": _positive_int(
            raw.get("max_neighbors", 3), "prediction_risk.max_neighbors"),
        "mode_count": _positive_int(
            raw.get("mode_count", 5), "prediction_risk.mode_count"),
        "overlap_half_x_m": _positive_float(
            raw.get("overlap_half_x_m",
                    raw.get("collision_half_length_m", 5.0)),
            "prediction_risk.overlap_half_x_m"),
        "overlap_half_y_m": _positive_float(
            raw.get("overlap_half_y_m",
                    raw.get("collision_half_width_m", 2.0)),
            "prediction_risk.overlap_half_y_m"),
        "model": {
            "config": model_config,
            "checkpoint": checkpoint,
            "device": device,
            "seed": int(model.get("seed", 1)),
            # These preserve the preprocessing convention in the supplied
            # example.  They are intentionally explicit rather than inferred
            # from the training YAML's unrelated ``scale`` field.
            "input_scale": _positive_float(
                model.get("input_scale", 2.5),
                "prediction_risk.model.input_scale"),
            "output_scale": _positive_float(
                model.get("output_scale", 2.5),
                "prediction_risk.model.output_scale"),
            # The bundled checkpoint behaves as a fixed +X/highway-frame
            # predictor. Rotate decoded canonical displacements into each
            # actor's recent CARLA-world heading unless explicitly disabled.
            "output_frame": _output_frame(
                model.get("output_frame", default_output_frame),
                "prediction_risk.model.output_frame"),
            "heading_history_samples": _positive_int(
                model.get("heading_history_samples", 5),
                "prediction_risk.model.heading_history_samples", minimum=2),
            "minimum_heading_displacement_m": _positive_float(
                model.get("minimum_heading_displacement_m", 0.05),
                "prediction_risk.model.minimum_heading_displacement_m"),
        },
        "dashboard": {
            "enabled": bool(dashboard.get("enabled", True)),
            "column_width_px": _positive_int(
                dashboard.get("column_width_px", 450),
                "prediction_risk.dashboard.column_width_px"),
            "history_length": _positive_int(
                dashboard.get("history_length", 200),
                "prediction_risk.dashboard.history_length"),
            "startup_timeout_s": _positive_float(
                dashboard.get("startup_timeout_s", 10.0),
                "prediction_risk.dashboard.startup_timeout_s"),
        },
        "drawing": {
            # Thousands of CARLA debug RPCs per second can disturb the sole
            # synchronous tick owner.  Keep trajectory drawing opt-in; the
            # process-isolated dashboard remains enabled by default.
            "enabled": bool(drawing.get("enabled", False)),
            "mode_count": _positive_int(
                drawing.get("mode_count", 3),
                "prediction_risk.drawing.mode_count"),
            "life_time_s": _positive_float(
                drawing.get("life_time_s", 0.20),
                "prediction_risk.drawing.life_time_s"),
        },
    }
    return settings


class PositionHistoryBuffer(object):
    """Fixed-length XY histories that reset immediately when an actor leaves."""

    def __init__(self, history_length):
        self.history_length = _positive_int(history_length, "history_length")
        self._points = {}

    def observe(self, positions):
        """Record one tick from ``{actor_id: (x, y)}`` and prune absent IDs."""
        current_ids = set(str(actor_id) for actor_id in positions)
        for actor_id in list(self._points):
            if actor_id not in current_ids:
                del self._points[actor_id]

        for actor_id, position in positions.items():
            actor_id = str(actor_id)
            point = np.asarray(position, dtype=np.float32)
            if point.shape != (2,) or not np.all(np.isfinite(point)):
                raise ValueError("position for %s must contain two finite values" % actor_id)
            history = self._points.get(actor_id)
            if history is None:
                history = deque(maxlen=self.history_length)
                self._points[actor_id] = history
            history.append((float(point[0]), float(point[1])))

    def sample_count(self, actor_id):
        history = self._points.get(str(actor_id))
        return len(history) if history is not None else 0

    def ready(self, actor_id):
        return self.sample_count(actor_id) >= self.history_length

    def array(self, actor_id):
        history = self._points.get(str(actor_id))
        if history is None:
            raise KeyError(str(actor_id))
        return np.asarray(history, dtype=np.float32)

    def actor_ids(self):
        return tuple(self._points)


def constant_velocity_plan(history, prediction_length):
    """Extrapolate the last per-tick displacement for a future XY plan."""
    history = np.asarray(history, dtype=np.float32)
    prediction_length = _positive_int(prediction_length, "prediction_length")
    if history.ndim != 2 or history.shape[1:] != (2,) or history.shape[0] < 2:
        raise ValueError("ego history must have shape [at least 2, 2]")
    if not np.all(np.isfinite(history)):
        raise ValueError("ego history contains non-finite values")
    displacement = history[-1] - history[-2]
    steps = np.arange(1, prediction_length + 1, dtype=np.float32)[:, None]
    return history[-1][None, :] + steps * displacement[None, :]


def compute_inverse_ttc(ego_plan, predicted_trajectories, step_s,
                        half_length_m, half_width_m, mode_count=None):
    """Return legacy rectangular-overlap inverse TTC values in s^-1.

    ``ego_plan`` has shape ``[H, 2]`` and predictions have shape
    ``[N, K, H, 2]``.  The first future overlap is at time ``(index + 1) *
    step_s``.  Missing requested modes are padded with zero, matching the
    supplied reference implementation.
    """
    ego_plan = np.asarray(ego_plan, dtype=np.float32)
    predictions = np.asarray(predicted_trajectories, dtype=np.float32)
    step_s = _positive_float(step_s, "step_s")
    half_length_m = _positive_float(half_length_m, "half_length_m")
    half_width_m = _positive_float(half_width_m, "half_width_m")

    if ego_plan.ndim != 2 or ego_plan.shape[1:] != (2,) or ego_plan.shape[0] == 0:
        raise ValueError("ego_plan must have shape [H, 2]")
    if predictions.ndim != 4 or predictions.shape[-1] != 2:
        raise ValueError("predicted_trajectories must have shape [N, K, H, 2]")
    if predictions.shape[2] != ego_plan.shape[0]:
        raise ValueError("prediction horizon does not match the ego plan")
    if not np.all(np.isfinite(ego_plan)) or not np.all(np.isfinite(predictions)):
        raise ValueError("trajectory inputs contain non-finite values")

    available_modes = predictions.shape[1]
    requested_modes = (available_modes if mode_count is None
                       else _positive_int(mode_count, "mode_count"))
    evaluated_modes = min(available_modes, requested_modes)
    values = np.zeros((predictions.shape[0], requested_modes), dtype=np.float64)
    if predictions.shape[0] == 0 or evaluated_modes == 0:
        return values

    selected = predictions[:, :evaluated_modes]
    delta = np.abs(selected - ego_plan[None, None, :, :])
    hits = ((delta[:, :, :, 0] <= half_length_m) &
            (delta[:, :, :, 1] <= half_width_m))
    has_hit = np.any(hits, axis=2)
    first_hit = np.argmax(hits, axis=2)
    times = (first_hit.astype(np.float64) + 1.0) * step_s
    selected_values = np.zeros_like(times, dtype=np.float64)
    selected_values[has_hit] = 1.0 / times[has_hit]
    values[:, :evaluated_modes] = selected_values
    return values


def mode_probabilities_for_display(probabilities, mode_count):
    probabilities = np.asarray(probabilities, dtype=np.float64)
    mode_count = _positive_int(mode_count, "mode_count")
    if probabilities.ndim != 2:
        raise ValueError("probabilities must have shape [N, K]")
    if not np.all(np.isfinite(probabilities)) or np.any(probabilities < 0.0):
        raise ValueError("probabilities must be finite and non-negative")
    result = np.zeros((probabilities.shape[0], mode_count), dtype=np.float64)
    copied = min(mode_count, probabilities.shape[1])
    result[:, :copied] = probabilities[:, :copied]
    return result


def aggregate_inverse_ttc(inverse_ttc, probabilities):
    """Return row-wise worst and selected-mode probability-weighted values."""
    values = np.asarray(inverse_ttc, dtype=np.float64)
    probabilities = np.asarray(probabilities, dtype=np.float64)
    if values.ndim != 2 or probabilities.shape != values.shape:
        raise ValueError("inverse TTC and probabilities must have matching [N, K] shapes")
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError("inverse TTC values must be finite and non-negative")
    if not np.all(np.isfinite(probabilities)) or np.any(probabilities < 0.0):
        raise ValueError("probabilities must be finite and non-negative")
    if values.shape[0] == 0:
        return np.zeros(0), np.zeros(0)

    worst = np.max(values, axis=1) if values.shape[1] else np.zeros(values.shape[0])
    totals = np.sum(probabilities, axis=1)
    expected = np.zeros(values.shape[0], dtype=np.float64)
    weighted_rows = totals > 0.0
    expected[weighted_rows] = (
        np.sum(values[weighted_rows] * probabilities[weighted_rows], axis=1) /
        totals[weighted_rows])
    if values.shape[1]:
        expected[~weighted_rows] = np.mean(values[~weighted_rows], axis=1)
    return worst, expected


class PredictionRiskMonitor(object):
    """Coordinate histories, HGT inference, risk evaluation, and publishing."""

    def __init__(self, settings, step_length, predictor=None, dashboard=None):
        self.settings = dict(settings)
        self.step_length = _positive_float(step_length, "step_length")
        sample_step_s = _positive_float(
            self.settings.get("sample_step_s", self.step_length),
            "prediction_risk.sample_step_s")
        if abs(self.step_length - sample_step_s) > 1.0e-9:
            raise ValueError(
                "HGT prediction expects %.6f s samples, but the simulation runner "
                "uses %.6f s; regenerate/resample for the model cadence or "
                "disable prediction risk" % (sample_step_s, self.step_length))
        self.sample_step_s = sample_step_s
        self.realtime_pacing = bool(self.settings.get("pace_realtime", False))
        self._next_update_s = None
        self._closed = False

        if predictor is None:
            from carla_reconstruction.closed_loop.hgt_predictor import HgtMartsPredictor
            model = self.settings["model"]
            predictor = HgtMartsPredictor(
                model_config_path=model["config"],
                checkpoint_path=model["checkpoint"],
                device=model["device"],
                seed=model["seed"],
                input_scale=model["input_scale"],
                output_scale=model["output_scale"],
                output_frame=model["output_frame"],
                heading_history_samples=model["heading_history_samples"],
                minimum_heading_displacement_m=(
                    model["minimum_heading_displacement_m"]))
        self.predictor = predictor

        if self.predictor.history_length < 2:
            self.predictor.close()
            raise ValueError("the predictor must require at least two history samples")
        if self.predictor.prediction_length < 1 or self.predictor.mode_count < 1:
            self.predictor.close()
            raise ValueError("the predictor reported invalid output dimensions")

        self.histories = PositionHistoryBuffer(self.predictor.history_length)
        if dashboard is None:
            dashboard_settings = self.settings["dashboard"]
            try:
                if dashboard_settings["enabled"]:
                    from carla_reconstruction.closed_loop.risk_dashboard import ProcessRiskDashboard
                    dashboard = ProcessRiskDashboard(
                        max_pairs=self.settings["max_neighbors"],
                        mode_count=self.settings["mode_count"],
                        column_width_px=dashboard_settings["column_width_px"],
                        history_length=dashboard_settings["history_length"],
                        title="Prediction-based inverse TTC",
                        startup_timeout=dashboard_settings["startup_timeout_s"])
                else:
                    from carla_reconstruction.closed_loop.risk_dashboard import NullRiskDashboard
                    dashboard = NullRiskDashboard()
            except Exception:
                self.predictor.close()
                raise
        self.dashboard = dashboard

    def _select_neighbors(self, ego_position, other_states):
        candidates = []
        for actor_id, state in other_states.items():
            position = np.asarray(state["position"], dtype=np.float64)
            if position.shape != (2,) or not np.all(np.isfinite(position)):
                continue
            distance = float(np.linalg.norm(position - ego_position))
            if distance <= self.settings["neighbor_radius_m"]:
                candidates.append((distance, str(actor_id)))
        candidates.sort(key=lambda item: (item[0], item[1]))
        return candidates[:self.settings["max_neighbors"]]

    def _base_actor_row(self, actor_id, distance, sample_count):
        return {
            "actor_id": str(actor_id),
            "distance_m": float(distance),
            "history_samples": int(sample_count),
            "history_required": int(self.predictor.history_length),
            "ready": False,
            "evaluated_mode_count": int(self.predictor.mode_count),
            "displayed_mode_count": int(self.settings["mode_count"]),
            "inverse_ttc_s_inv": [0.0] * self.settings["mode_count"],
            "probabilities": [0.0] * self.settings["mode_count"],
            "worst_inverse_ttc_s_inv": 0.0,
            "expected_inverse_ttc_s_inv": 0.0,
        }

    def observe(self, sim_time_s, ego_state, other_states, world=None):
        """Record one post-synchronization tick and publish when inference is due."""
        if self._closed:
            return None
        sim_time_s = float(sim_time_s)
        if not math.isfinite(sim_time_s):
            raise ValueError("sim_time_s must be finite")
        ego_position = np.asarray(ego_state["position"], dtype=np.float64)
        if ego_position.shape != (2,) or not np.all(np.isfinite(ego_position)):
            raise ValueError("ego_state.position must contain two finite values")

        positions = {EGO_HISTORY_ID: tuple(ego_position)}
        for actor_id, state in other_states.items():
            positions[str(actor_id)] = state["position"]
        self.histories.observe(positions)

        if self._next_update_s is not None and sim_time_s + 1.0e-9 < self._next_update_s:
            return None
        self._next_update_s = sim_time_s + self.settings["update_interval_s"]

        selected = self._select_neighbors(ego_position, other_states)
        rows = [self._base_actor_row(
            actor_id, distance, self.histories.sample_count(actor_id))
            for distance, actor_id in selected]

        if not selected:
            frame = {
                "type": "risk_frame",
                "sim_time_s": sim_time_s,
                "status": "No surrounding vehicles within %.1f m" %
                          self.settings["neighbor_radius_m"],
                "actors": rows,
            }
            self.dashboard.publish(frame)
            return frame

        eligible = [actor_id for _distance, actor_id in selected
                    if self.histories.ready(actor_id)]
        if not eligible or self.histories.sample_count(EGO_HISTORY_ID) < 2:
            frame = {
                "type": "risk_frame",
                "sim_time_s": sim_time_s,
                "status": "Warming trajectory histories (0/%d ready)" % len(selected),
                "actors": rows,
            }
            self.dashboard.publish(frame)
            return frame

        histories = np.stack([self.histories.array(actor_id) for actor_id in eligible], axis=0)
        ego_plan = constant_velocity_plan(
            self.histories.array(EGO_HISTORY_ID), self.predictor.prediction_length)
        if getattr(self.predictor, "supports_actor_headings", False):
            actor_headings = np.asarray([
                other_states[actor_id].get(
                    "heading",
                    other_states[actor_id].get("velocity", (0.0, 0.0)))
                for actor_id in eligible
            ], dtype=np.float32)
            predictions, probabilities = self.predictor.predict(
                histories, ego_plan, actor_headings=actor_headings)
        else:
            predictions, probabilities = self.predictor.predict(
                histories, ego_plan)
        all_inverse_ttc = compute_inverse_ttc(
            ego_plan, predictions, self.sample_step_s,
            self.settings["overlap_half_x_m"],
            self.settings["overlap_half_y_m"])
        inverse_ttc = np.zeros(
            (all_inverse_ttc.shape[0], self.settings["mode_count"]),
            dtype=np.float64)
        copied_modes = min(all_inverse_ttc.shape[1], self.settings["mode_count"])
        inverse_ttc[:, :copied_modes] = all_inverse_ttc[:, :copied_modes]
        display_probabilities = mode_probabilities_for_display(
            probabilities, self.settings["mode_count"])
        worst, expected = aggregate_inverse_ttc(all_inverse_ttc, probabilities)

        eligible_index = {actor_id: index for index, actor_id in enumerate(eligible)}
        for row in rows:
            index = eligible_index.get(row["actor_id"])
            if index is None:
                continue
            row["ready"] = True
            row["inverse_ttc_s_inv"] = inverse_ttc[index].tolist()
            row["probabilities"] = display_probabilities[index].tolist()
            row["worst_inverse_ttc_s_inv"] = float(worst[index])
            row["expected_inverse_ttc_s_inv"] = float(expected[index])

        warming = len(selected) - len(eligible)
        status = "Live HGT prediction"
        if warming:
            status += "; %d vehicle(s) warming" % warming
        frame = {
            "type": "risk_frame",
            "sim_time_s": sim_time_s,
            "status": status,
            "actors": rows,
        }
        self.dashboard.publish(frame)

        if world is not None and self.settings["drawing"]["enabled"]:
            self._draw(world, ego_state, other_states, ego_plan, eligible,
                       predictions)
        return frame

    def _draw(self, world, ego_state, other_states, ego_plan, actor_ids,
              predictions):
        try:
            import carla
        except ImportError as exc:
            raise RuntimeError("CARLA is required to draw predicted trajectories") from exc

        drawing = self.settings["drawing"]
        life_time = drawing["life_time_s"]
        ego_z = float(ego_state.get("z", 0.0)) + 1.0
        for index in range(1, ego_plan.shape[0]):
            world.debug.draw_line(
                carla.Location(x=float(ego_plan[index - 1, 0]),
                               y=float(ego_plan[index - 1, 1]), z=ego_z),
                carla.Location(x=float(ego_plan[index, 0]),
                               y=float(ego_plan[index, 1]), z=ego_z),
                thickness=0.10, color=carla.Color(162, 217, 77),
                life_time=life_time)

        draw_modes = min(
            drawing["mode_count"], self.settings["mode_count"],
            predictions.shape[1])
        for actor_index, actor_id in enumerate(actor_ids):
            # Match the dashboard bars exactly; the overlay is opt-in and must
            # not silently depict a different set of model heads.
            mode_indices = range(draw_modes)
            z = float(other_states[actor_id].get("z", 0.0)) + 1.0
            for mode_index in mode_indices:
                trajectory = predictions[actor_index, mode_index]
                actor_position = other_states[actor_id]["position"]
                world.debug.draw_line(
                    carla.Location(x=float(actor_position[0]),
                                   y=float(actor_position[1]), z=z),
                    carla.Location(x=float(trajectory[0, 0]),
                                   y=float(trajectory[0, 1]), z=z),
                    thickness=0.06, color=carla.Color(66, 27, 123),
                    life_time=life_time)
                for point_index in range(1, trajectory.shape[0]):
                    world.debug.draw_line(
                        carla.Location(x=float(trajectory[point_index - 1, 0]),
                                       y=float(trajectory[point_index - 1, 1]), z=z),
                        carla.Location(x=float(trajectory[point_index, 0]),
                                       y=float(trajectory[point_index, 1]), z=z),
                        thickness=0.06, color=carla.Color(66, 27, 123),
                        life_time=life_time)

    def metadata(self):
        return {
            "enabled": True,
            "model": "HGT MARTS",
            "model_config": self.settings["model"]["config"],
            "checkpoint": self.settings["model"]["checkpoint"],
            "device": getattr(self.predictor, "device_name", "unknown"),
            "history_length": int(self.predictor.history_length),
            "prediction_length": int(self.predictor.prediction_length),
            "model_modes": int(self.predictor.mode_count),
            "display_modes": int(self.settings["mode_count"]),
            "output_frame": self.settings["model"].get(
                "output_frame", "unspecified"),
            "heading_history_samples": self.settings["model"].get(
                "heading_history_samples"),
            "minimum_heading_displacement_m": self.settings["model"].get(
                "minimum_heading_displacement_m"),
            "sample_step_s": float(self.sample_step_s),
            "update_interval_s": float(self.settings["update_interval_s"]),
        }

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self.dashboard.close()
        finally:
            self.predictor.close()
