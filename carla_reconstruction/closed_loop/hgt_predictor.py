"""Lazy MARTS trajectory-prediction adapter for closed-loop runtimes.

Importing this module intentionally does not import NumPy, PyYAML, or PyTorch.
Those optional dependencies are loaded only when :class:`HgtMartsPredictor` is
constructed, so the existing co-simulation can run without prediction enabled.
"""

import math
from numbers import Integral
import os
from collections.abc import Mapping
from types import SimpleNamespace


__all__ = ["HgtMartsPredictor", "decode_relative_predictions"]


def _dependency_error(module_name, install_name, exc):
    message = (
        "HGT trajectory prediction requires '{0}'. Install the '{1}' package "
        "in the Python environment used to run the co-simulation."
    ).format(module_name, install_name)
    return RuntimeError(message)


def _recursive_namespace(value):
    """Convert mappings recursively without depending on ``python-box``."""
    if isinstance(value, Mapping):
        namespace = SimpleNamespace()
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(
                    "Model configuration keys must be strings; got {!r}.".format(key))
            setattr(namespace, key, _recursive_namespace(item))
        return namespace
    if isinstance(value, list):
        return [_recursive_namespace(item) for item in value]
    if isinstance(value, tuple):
        return tuple(_recursive_namespace(item) for item in value)
    return value


def _positive_int(config, name, minimum=1):
    if not hasattr(config, name):
        raise ValueError("Model configuration is missing required key '{0}'.".format(name))
    value = getattr(config, name)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(
            "Model configuration key '{0}' must be an integer >= {1}; got {2!r}."
            .format(name, minimum, value))
    return int(value)


def _finite_positive(value, name):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise ValueError("{0} must be a finite positive number; got {1!r}.".format(
            name, value))
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError("{0} must be a finite positive number; got {1!r}.".format(
            name, value))
    return parsed


def _plain_positive_int(value, name, minimum=1):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ValueError(
            "{0} must be an integer >= {1}; got {2!r}."
            .format(name, minimum, value))
    return int(value)


def _validated_output_frame(value):
    if not isinstance(value, str):
        raise ValueError(
            "output_frame must be 'actor_heading' or 'world'; got {!r}."
            .format(value))
    normalized = value.strip().lower().replace("-", "_")
    if normalized not in ("actor_heading", "world"):
        raise ValueError(
            "output_frame must be 'actor_heading' or 'world'; got {!r}."
            .format(value))
    return normalized


def decode_relative_predictions(raw_predictions, histories, output_scale=2.5,
                                output_frame="world",
                                heading_history_samples=5,
                                minimum_heading_displacement_m=0.05,
                                actor_headings=None):
    """Decode MARTS step deltas into CARLA-world trajectories.

    ``actor_heading`` treats the checkpoint's output axes as canonical
    longitudinal/lateral axes and rotates them to each actor's CARLA heading.
    If ``actor_headings`` is omitted or contains a zero vector, recent travel
    direction is used as a fallback. Histories remain in the reference
    implementation's world-axis input frame; only decoded displacements are
    transformed. ``world`` keeps the supplied example's fixed-axis decoding.
    """
    try:
        import numpy as np
    except (ImportError, OSError) as exc:
        raise _dependency_error("numpy", "numpy", exc) from exc

    scale = _finite_positive(output_scale, "output_scale")
    frame = _validated_output_frame(output_frame)
    heading_samples = _plain_positive_int(
        heading_history_samples, "heading_history_samples", minimum=2)
    minimum_displacement = _finite_positive(
        minimum_heading_displacement_m,
        "minimum_heading_displacement_m")

    raw = np.asarray(raw_predictions)
    history = np.asarray(histories)
    if raw.ndim != 4 or raw.shape[-1] != 2:
        raise ValueError(
            "raw_predictions must have shape [N, K, H, 2]; got {}."
            .format(tuple(raw.shape)))
    if history.ndim != 3 or history.shape[-1] != 2 or history.shape[1] < 2:
        raise ValueError(
            "histories must have shape [N, T, 2] with T >= 2; got {}."
            .format(tuple(history.shape)))
    if raw.shape[0] < 1 or raw.shape[1] < 1 or raw.shape[2] < 1:
        raise ValueError(
            "raw_predictions must contain at least one actor, mode, and step.")
    if history.shape[0] != raw.shape[0]:
        raise ValueError(
            "raw_predictions and histories must contain the same actors; got "
            "{} and {}.".format(raw.shape[0], history.shape[0]))

    supplied_headings = None
    if actor_headings is not None:
        supplied_headings = np.asarray(actor_headings)
        if supplied_headings.ndim != 2 or supplied_headings.shape != (
                raw.shape[0], 2):
            raise ValueError(
                "actor_headings must have shape [N, 2]; got {}."
                .format(tuple(supplied_headings.shape)))

    validated_values = [(raw, "raw_predictions"), (history, "histories")]
    if supplied_headings is not None:
        validated_values.append((supplied_headings, "actor_headings"))
    for value, name in validated_values:
        if not (np.issubdtype(value.dtype, np.number)
                and not np.issubdtype(value.dtype, np.complexfloating)
                and not np.issubdtype(value.dtype, np.bool_)):
            raise TypeError(
                "{} must contain real numeric values; got dtype {}."
                .format(name, value.dtype))
        if not np.isfinite(value).all():
            raise ValueError("{} contains NaN or infinite values.".format(name))

    raw_float = np.ascontiguousarray(raw, dtype=np.float32)
    history_float = np.ascontiguousarray(history, dtype=np.float32)
    supplied_headings_float = None
    if supplied_headings is not None:
        supplied_headings_float = np.ascontiguousarray(
            supplied_headings, dtype=np.float32)
    if (not np.isfinite(raw_float).all()
            or not np.isfinite(history_float).all()
            or (supplied_headings_float is not None
                and not np.isfinite(supplied_headings_float).all())):
        raise ValueError("prediction inputs contain values outside float32 range.")

    displacements = np.cumsum(
        raw_float * np.float32(scale), axis=2, dtype=np.float32)
    current = history_float[:, -1, :]
    if frame == "world":
        trajectories = displacements + current[:, np.newaxis, np.newaxis, :]
        if not np.isfinite(trajectories).all():
            raise RuntimeError(
                "decoded MARTS trajectories contain non-finite values.")
        return np.ascontiguousarray(trajectories, dtype=np.float32)

    # Estimate a stable current direction from the latest short history.  If
    # an actor barely moved in that window, use its complete available history.
    # A fully stationary actor without a supplied heading deterministically
    # retains the canonical +X/world basis.
    first_recent = max(0, history_float.shape[1] - heading_samples)
    direction = current - history_float[:, first_recent, :]
    norm = np.linalg.norm(direction, axis=1)
    use_full_history = norm < minimum_displacement
    if np.any(use_full_history):
        full_direction = current - history_float[:, 0, :]
        direction[use_full_history] = full_direction[use_full_history]
        norm = np.linalg.norm(direction, axis=1)

    forward = np.zeros_like(direction, dtype=np.float32)
    forward[:, 0] = 1.0
    moving = norm >= minimum_displacement
    if np.any(moving):
        forward[moving] = direction[moving] / norm[moving, np.newaxis]

    # Synchronized actor orientation is reliable even while a vehicle is
    # stopped and takes precedence over the displacement fallback above.
    if supplied_headings_float is not None:
        supplied_norm = np.linalg.norm(supplied_headings_float, axis=1)
        has_supplied_heading = supplied_norm > 1.0e-6
        if np.any(has_supplied_heading):
            forward[has_supplied_heading] = (
                supplied_headings_float[has_supplied_heading] /
                supplied_norm[has_supplied_heading, np.newaxis])

    canonical_x = displacements[..., 0]
    canonical_y = displacements[..., 1]
    forward_x = forward[:, 0, np.newaxis, np.newaxis]
    forward_y = forward[:, 1, np.newaxis, np.newaxis]
    rotated = np.empty_like(displacements)
    rotated[..., 0] = canonical_x * forward_x - canonical_y * forward_y
    rotated[..., 1] = canonical_x * forward_y + canonical_y * forward_x
    trajectories = rotated + current[:, np.newaxis, np.newaxis, :]
    if not np.isfinite(trajectories).all():
        raise RuntimeError("decoded MARTS trajectories contain non-finite values.")
    return np.ascontiguousarray(trajectories, dtype=np.float32)


def _existing_file(path, label):
    try:
        path_string = os.fspath(path)
    except TypeError:
        raise TypeError("{0} must be a filesystem path; got {1!r}.".format(label, path))
    resolved = os.path.abspath(os.path.expanduser(path_string))
    if not os.path.isfile(resolved):
        raise FileNotFoundError(
            "{0} does not exist or is not a file: {1}".format(label, resolved))
    return resolved


class HgtMartsPredictor:
    """Load MARTS and predict world-frame trajectories for nearby actors.

    The preprocessing and decoding deliberately preserve the supplied reference
    implementation: each history is centered on its last sample, centered
    positions and step differences are multiplied by ``input_scale``, and raw
    relative predictions are multiplied by ``output_scale`` and cumulatively
    summed. When ``output_frame='actor_heading'``, the resulting canonical
    displacements are rotated to each actor's heading before translation to its
    latest world position. The general adapter default, ``output_frame='world'``,
    preserves the supplied example for other checkpoints. ``ego_plan`` remains
    an absolute, unscaled world-frame trajectory.
    """

    supports_actor_headings = True

    def __init__(self, model_config_path, checkpoint_path, device="auto", seed=1,
                 input_scale=2.5, output_scale=2.5,
                 output_frame="world", heading_history_samples=5,
                 minimum_heading_displacement_m=0.05):
        self._closed = False
        self._model = None
        self._torch = None
        self._np = None

        self._model_config_path = _existing_file(
            model_config_path, "HGT model configuration")
        self._checkpoint_path = _existing_file(
            checkpoint_path, "HGT model checkpoint")
        self._input_scale = _finite_positive(input_scale, "input_scale")
        self._output_scale = _finite_positive(output_scale, "output_scale")
        self._output_frame = _validated_output_frame(output_frame)
        self._heading_history_samples = _plain_positive_int(
            heading_history_samples, "heading_history_samples", minimum=2)
        self._minimum_heading_displacement_m = _finite_positive(
            minimum_heading_displacement_m,
            "minimum_heading_displacement_m")

        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("seed must be an integer; got {!r}.".format(seed))
        self._seed = int(seed)

        try:
            import numpy as np
        except (ImportError, OSError) as exc:
            raise _dependency_error("numpy", "numpy", exc) from exc
        try:
            import yaml
        except (ImportError, OSError) as exc:
            raise _dependency_error("yaml", "PyYAML", exc) from exc
        try:
            import torch
        except (ImportError, OSError) as exc:
            raise _dependency_error("torch", "torch", exc) from exc

        self._np = np
        self._torch = torch

        with open(self._model_config_path, "r", encoding="utf-8") as stream:
            try:
                config_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                raise ValueError(
                    "Could not parse HGT model configuration '{}': {}"
                    .format(self._model_config_path, exc)) from exc
        if not isinstance(config_data, Mapping):
            raise ValueError(
                "HGT model configuration must contain a YAML mapping: {}"
                .format(self._model_config_path))
        config = _recursive_namespace(config_data)

        self._history_length = _positive_int(config, "past_length", minimum=2)
        self._prediction_length = _positive_int(config, "future_length")
        self._mode_count = _positive_int(config, "sample_k")
        if self._history_length > 200 or self._prediction_length > 200:
            raise ValueError(
                "MARTS positional encodings support at most 200 history and "
                "prediction steps; got past_length={} and future_length={}."
                .format(self._history_length, self._prediction_length))
        dimensions = {}
        for required_name in (
                "num_layers", "num_heads", "model_dim", "hidden_dim",
                "decoder_hidden_dim"):
            dimensions[required_name] = _positive_int(config, required_name)
        if dimensions["model_dim"] % dimensions["num_heads"] != 0:
            raise ValueError(
                "Model configuration model_dim ({}) must be divisible by "
                "num_heads ({}).".format(
                    dimensions["model_dim"], dimensions["num_heads"]))
        for required_name in (
                "dropout", "aggregation", "function_type", "inputs", "pred_rel"):
            if not hasattr(config, required_name):
                raise ValueError(
                    "Model configuration is missing required key '{0}'."
                    .format(required_name))
        if config.pred_rel is not True:
            raise ValueError(
                "HgtMartsPredictor requires model configuration pred_rel: true "
                "because reference-compatible decoding cumulatively sums relative "
                "predictions.")
        if not isinstance(config.inputs, (list, tuple)) or not all(
                isinstance(item, str) for item in config.inputs):
            raise ValueError(
                "Model configuration key 'inputs' must be a list of strings.")
        input_names = set(config.inputs)
        supported_inputs = {"pos_x", "pos_y", "vel_x", "vel_y"}
        has_position_pair = {"pos_x", "pos_y"}.issubset(input_names)
        has_velocity_pair = {"vel_x", "vel_y"}.issubset(input_names)
        if not (has_position_pair or has_velocity_pair):
            raise ValueError(
                "Model configuration 'inputs' must contain a complete position "
                "pair (pos_x,pos_y) or velocity pair (vel_x,vel_y).")
        if input_names - supported_inputs or len(config.inputs) != (
                (2 if has_position_pair else 0)
                + (2 if has_velocity_pair else 0)):
            raise ValueError(
                "MARTS only supports the input pairs pos_x,pos_y and "
                "vel_x,vel_y; got {!r}.".format(config.inputs))

        if device is None:
            requested_device = "auto"
        else:
            requested_device = str(device).strip()
        if not requested_device:
            raise ValueError("device must be 'auto' or a valid PyTorch device string.")
        if requested_device.lower() == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"
        try:
            resolved_device = torch.device(requested_device)
        except (TypeError, ValueError, RuntimeError) as exc:
            raise ValueError(
                "Invalid PyTorch device {!r}: {}".format(device, exc)) from exc
        if resolved_device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError(
                "CUDA device {!r} was requested, but CUDA is not available. "
                "Use device='cpu' or install a CUDA-enabled PyTorch build."
                .format(requested_device))
        self._device = resolved_device
        self._device_name = str(resolved_device)

        torch.manual_seed(self._seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(self._seed)
        torch.backends.cudnn.deterministic = True

        try:
            from ..HGT_model.models.mart_s import MARTS
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "Could not import the bundled MARTS model from "
                "carla_reconstruction.HGT_model. Ensure the repository root is "
                "on PYTHONPATH and the HGT_model folder is complete: {}".format(exc)
            ) from exc

        try:
            model = MARTS(config).to(self._device)
        except Exception as exc:
            raise RuntimeError(
                "Could not construct the MARTS model from '{}': {}"
                .format(self._model_config_path, exc)) from exc

        try:
            try:
                checkpoint = torch.load(
                    self._checkpoint_path,
                    map_location="cpu",
                    weights_only=True,
                )
            except TypeError as exc:
                # ``weights_only`` is unavailable in older supported Torch builds.
                if "weights_only" not in str(exc):
                    raise
                checkpoint = torch.load(
                    self._checkpoint_path, map_location="cpu")
        except Exception as exc:
            raise RuntimeError(
                "Could not load trusted HGT checkpoint '{}': {}"
                .format(self._checkpoint_path, exc)) from exc

        if not isinstance(checkpoint, Mapping) or "state_dict" not in checkpoint:
            raise ValueError(
                "HGT checkpoint must be a mapping containing 'state_dict': {}"
                .format(self._checkpoint_path))
        state_dict = checkpoint["state_dict"]
        if not isinstance(state_dict, Mapping):
            raise ValueError(
                "HGT checkpoint 'state_dict' is not a parameter mapping: {}"
                .format(self._checkpoint_path))
        try:
            model.load_state_dict(state_dict, strict=True)
        except Exception as exc:
            raise RuntimeError(
                "HGT checkpoint is incompatible with model configuration '{}': {}"
                .format(self._model_config_path, exc)) from exc

        model.eval()
        self._model = model

    @property
    def history_length(self):
        return self._history_length

    @property
    def prediction_length(self):
        return self._prediction_length

    @property
    def mode_count(self):
        return self._mode_count

    @property
    def device_name(self):
        return self._device_name

    @property
    def output_frame(self):
        return self._output_frame

    def _validated_inputs(self, histories, ego_plan):
        np = self._np
        if not isinstance(histories, np.ndarray):
            raise TypeError(
                "histories must be a NumPy array with shape [N, {}, 2]."
                .format(self._history_length))
        if not isinstance(ego_plan, np.ndarray):
            raise TypeError(
                "ego_plan must be a NumPy array with shape [{}, 2]."
                .format(self._prediction_length))

        expected_history_suffix = (self._history_length, 2)
        if histories.ndim != 3 or tuple(histories.shape[1:]) != expected_history_suffix:
            raise ValueError(
                "histories must have shape [N, {}, 2]; got {}."
                .format(self._history_length, tuple(histories.shape)))
        if histories.shape[0] < 1:
            raise ValueError("histories must contain at least one actor (N >= 1).")
        expected_plan_shape = (self._prediction_length, 2)
        if ego_plan.ndim != 2 or tuple(ego_plan.shape) != expected_plan_shape:
            raise ValueError(
                "ego_plan must have shape [{}, 2]; got {}."
                .format(self._prediction_length, tuple(ego_plan.shape)))

        for value, name in ((histories, "histories"), (ego_plan, "ego_plan")):
            if not (np.issubdtype(value.dtype, np.number)
                    and not np.issubdtype(value.dtype, np.complexfloating)
                    and not np.issubdtype(value.dtype, np.bool_)):
                raise TypeError(
                    "{} must contain real numeric values; got dtype {}."
                    .format(name, value.dtype))
            if not np.isfinite(value).all():
                raise ValueError("{} contains NaN or infinite values.".format(name))

        histories_float = np.ascontiguousarray(histories, dtype=np.float32)
        ego_plan_float = np.ascontiguousarray(ego_plan, dtype=np.float32)
        if not np.isfinite(histories_float).all():
            raise ValueError(
                "histories contains values outside the finite float32 range.")
        if not np.isfinite(ego_plan_float).all():
            raise ValueError(
                "ego_plan contains values outside the finite float32 range.")
        return histories_float, ego_plan_float

    def predict(self, histories, ego_plan, actor_headings=None):
        """Return world-frame trajectories ``[N,K,H,2]`` and probabilities.

        ``histories`` must be a finite NumPy array in oldest-to-newest order with
        shape ``[N, history_length, 2]``. ``ego_plan`` must be a finite absolute
        world-frame NumPy array with shape ``[prediction_length, 2]``.
        """
        if self._closed or self._model is None:
            raise RuntimeError("HgtMartsPredictor is closed and cannot predict.")

        histories_np, ego_plan_np = self._validated_inputs(histories, ego_plan)
        torch = self._torch

        absolute = torch.from_numpy(histories_np).unsqueeze(0).float().to(self._device)
        centered = absolute - absolute[:, :, -1:, :]
        x_abs = centered[:, :, :self._history_length, :]

        x_rel = torch.zeros_like(x_abs)
        x_rel[:, :, 1:] = x_abs[:, :, 1:] - x_abs[:, :, :-1]
        x_rel[:, :, 0] = x_rel[:, :, 1]

        planned = (
            torch.from_numpy(ego_plan_np).unsqueeze(0).float().to(self._device))

        with torch.no_grad():
            raw_predictions, probabilities, _ = self._model(
                x_abs * self._input_scale,
                x_rel * self._input_scale,
                planned,
            )

            actor_count = histories_np.shape[0]
            expected_trajectory_shape = (
                1, actor_count, self._mode_count, self._prediction_length, 2)
            expected_probability_shape = (1, actor_count, self._mode_count)
            if tuple(raw_predictions.shape) != expected_trajectory_shape:
                raise RuntimeError(
                    "MARTS returned trajectory shape {}; expected {}."
                    .format(tuple(raw_predictions.shape), expected_trajectory_shape))
            if tuple(probabilities.shape) != expected_probability_shape:
                raise RuntimeError(
                    "MARTS returned probability shape {}; expected {}."
                    .format(tuple(probabilities.shape), expected_probability_shape))

        raw_predictions_np = raw_predictions[0].detach().cpu().numpy()
        trajectories_np = decode_relative_predictions(
            raw_predictions_np,
            histories_np,
            output_scale=self._output_scale,
            output_frame=self._output_frame,
            heading_history_samples=self._heading_history_samples,
            minimum_heading_displacement_m=(
                self._minimum_heading_displacement_m),
            actor_headings=actor_headings,
        )
        probabilities_np = probabilities[0].detach().cpu().numpy()
        if not self._np.isfinite(trajectories_np).all():
            raise RuntimeError("MARTS produced NaN or infinite trajectory values.")
        if not self._np.isfinite(probabilities_np).all():
            raise RuntimeError("MARTS produced NaN or infinite probabilities.")
        return trajectories_np, probabilities_np

    def close(self):
        """Release references to model resources; safe to call more than once."""
        if self._closed:
            return
        self._closed = True
        self._model = None
        self._torch = None
        self._np = None
