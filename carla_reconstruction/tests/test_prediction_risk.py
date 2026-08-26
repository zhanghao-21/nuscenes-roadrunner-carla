import builtins
import os
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

from closed_loop.hgt_predictor import decode_relative_predictions
from closed_loop.prediction_risk import (
    PositionHistoryBuffer,
    PredictionRiskMonitor,
    aggregate_inverse_ttc,
    compute_inverse_ttc,
    constant_velocity_plan,
    mode_probabilities_for_display,
    prediction_risk_settings,
)
from tools import prepare_sumo


class PredictionRiskSettingsTests(unittest.TestCase):
    def test_relative_model_paths_and_cli_overrides(self):
        hybrid_path = os.path.abspath(os.path.join(
            "scenario", "sumo", "hybrid_config.json"))
        settings = prediction_risk_settings({
            "prediction_risk": {
                "enabled": False,
                "model": {
                    "config": "model.yaml",
                    "checkpoint": "weights.pth",
                    "output_frame": "world",
                    "heading_history_samples": 7,
                    "minimum_heading_displacement_m": 0.2,
                },
                "dashboard": {
                    "column_width_px": 450,
                },
            },
        }, hybrid_path, enabled_override=True, device_override="cpu")

        self.assertTrue(settings["enabled"])
        self.assertEqual(settings["model"]["device"], "cpu")
        self.assertEqual(settings["model"]["output_frame"], "world")
        self.assertEqual(settings["model"]["heading_history_samples"], 7)
        self.assertEqual(
            settings["model"]["minimum_heading_displacement_m"], 0.2)
        self.assertEqual(settings["dashboard"]["column_width_px"], 450)
        self.assertEqual(
            settings["model"]["config"],
            os.path.abspath(os.path.join("scenario", "sumo", "model.yaml")))
        self.assertEqual(
            settings["model"]["checkpoint"],
            os.path.abspath(os.path.join("scenario", "sumo", "weights.pth")))

    def test_malformed_section_and_invalid_radius_are_rejected(self):
        with self.assertRaises(ValueError):
            prediction_risk_settings(
                {"prediction_risk": []}, "hybrid_config.json")
        with self.assertRaises(ValueError):
            prediction_risk_settings({
                "prediction_risk": {"neighbor_radius_m": 0.0},
            }, "hybrid_config.json")
        with self.assertRaises(ValueError):
            prediction_risk_settings({
                "prediction_risk": {
                    "dashboard": {"column_width_px": 0},
                },
            }, "hybrid_config.json")
        with self.assertRaises(ValueError):
            prediction_risk_settings({
                "prediction_risk": {
                    "model": {"output_frame": "flip_y"},
                },
            }, "hybrid_config.json")
        with self.assertRaises(ValueError):
            prediction_risk_settings({
                "prediction_risk": {
                    "model": {"heading_history_samples": 1},
                },
            }, "hybrid_config.json")
        with self.assertRaises(ValueError):
            prediction_risk_settings({
                "prediction_risk": {
                    "model": {"heading_history_samples": 2.5},
                },
            }, "hybrid_config.json")
        with self.assertRaises(ValueError):
            prediction_risk_settings({
                "prediction_risk": {
                    "model": {"minimum_heading_displacement_m": True},
                },
            }, "hybrid_config.json")

    def test_frame_and_dashboard_defaults_follow_checkpoint_compatibility(self):
        bundled = prediction_risk_settings({}, "hybrid_config.json")
        self.assertEqual(bundled["model"]["output_frame"], "actor_heading")
        self.assertEqual(bundled["max_neighbors"], 3)
        self.assertEqual(bundled["mode_count"], 5)
        self.assertEqual(bundled["dashboard"]["column_width_px"], 450)

        custom = prediction_risk_settings({
            "prediction_risk": {
                "model": {"checkpoint": "custom-weights.pth"},
            },
        }, os.path.abspath(os.path.join("scenario", "hybrid_config.json")))
        self.assertEqual(custom["model"]["output_frame"], "world")

    def test_prepare_sumo_rejects_model_cadence_mismatch(self):
        with self.assertRaisesRegex(ValueError, "requires --step-length 0.05"):
            prepare_sumo.run(SimpleNamespace(
                enable_prediction_risk=True, step_length=0.1))


class HgtPredictionDecodingTests(unittest.TestCase):
    def test_actor_heading_rotates_canonical_forward_for_four_directions(self):
        directions = np.asarray(
            ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)),
            dtype=np.float32)
        current = np.asarray(
            ((10.0, 20.0), (30.0, 40.0), (50.0, 60.0), (70.0, 80.0)),
            dtype=np.float32)
        offsets = np.arange(-5, 1, dtype=np.float32)
        histories = (
            current[:, np.newaxis, :] +
            offsets[np.newaxis, :, np.newaxis] *
            directions[:, np.newaxis, :])

        raw = np.zeros((4, 2, 3, 2), dtype=np.float32)
        raw[..., 0] = 1.0
        raw[:, 1, :, 1] = 0.2
        predictions = decode_relative_predictions(
            raw, histories, output_scale=2.5,
            output_frame="actor_heading", heading_history_samples=5,
            minimum_heading_displacement_m=0.05)

        step_numbers = np.arange(1, 4, dtype=np.float32)
        expected_forward = (
            current[:, np.newaxis, :] +
            2.5 * step_numbers[np.newaxis, :, np.newaxis] *
            directions[:, np.newaxis, :])
        np.testing.assert_allclose(predictions[:, 0], expected_forward, atol=1e-6)

        left = np.stack((-directions[:, 1], directions[:, 0]), axis=1)
        expected_lateral = expected_forward + (
            0.5 * step_numbers[np.newaxis, :, np.newaxis] *
            left[:, np.newaxis, :])
        np.testing.assert_allclose(predictions[:, 1], expected_lateral, atol=1e-6)

        anchored = np.concatenate(
            (current[:, np.newaxis, :], predictions[:, 0]), axis=1)
        increments = np.diff(anchored, axis=1)
        longitudinal = np.sum(
            increments * directions[:, np.newaxis, :], axis=2)
        np.testing.assert_allclose(longitudinal, 2.5, atol=1e-6)

    def test_world_mode_and_stationary_heading_fallback_are_deterministic(self):
        raw = np.zeros((1, 1, 2, 2), dtype=np.float32)
        raw[..., 0] = 1.0
        stationary = np.asarray(
            (((4.0, 9.0), (4.0, 9.0), (4.0, 9.0)),),
            dtype=np.float32)

        world = decode_relative_predictions(
            raw, stationary, output_scale=2.0, output_frame="world")
        heading = decode_relative_predictions(
            raw, stationary, output_scale=2.0,
            output_frame="actor_heading")
        expected = np.asarray(((((6.0, 9.0), (8.0, 9.0)),),))
        np.testing.assert_allclose(world, expected)
        np.testing.assert_allclose(heading, expected)
        self.assertTrue(np.isfinite(heading).all())

    def test_supplied_actor_heading_orients_a_stationary_vehicle(self):
        raw = np.zeros((4, 1, 2, 2), dtype=np.float32)
        raw[..., 0] = 1.0
        histories = np.zeros((4, 3, 2), dtype=np.float32)
        headings = np.asarray(
            ((1.0, 0.0), (-2.0, 0.0), (0.0, 3.0), (0.0, -4.0)),
            dtype=np.float32)

        predictions = decode_relative_predictions(
            raw, histories, output_scale=1.0,
            output_frame="actor_heading", actor_headings=headings)
        expected = np.asarray((1.0, 2.0), dtype=np.float32)
        longitudinal = np.sum(
            predictions[:, 0] *
            (headings / np.linalg.norm(headings, axis=1)[:, np.newaxis])
            [:, np.newaxis, :],
            axis=2)
        np.testing.assert_allclose(
            longitudinal, np.tile(expected, (headings.shape[0], 1)))

    def test_invalid_decode_frame_and_shapes_are_rejected(self):
        raw = np.zeros((1, 1, 2, 2), dtype=np.float32)
        history = np.zeros((1, 3, 2), dtype=np.float32)
        with self.assertRaisesRegex(ValueError, "output_frame"):
            decode_relative_predictions(raw, history, output_frame="flip_y")
        with self.assertRaisesRegex(ValueError, "raw_predictions"):
            decode_relative_predictions(raw[0], history)
        with self.assertRaisesRegex(ValueError, "same actors"):
            decode_relative_predictions(
                np.zeros((2, 1, 2, 2), dtype=np.float32), history)
        with self.assertRaisesRegex(ValueError, "actor_headings"):
            decode_relative_predictions(
                raw, history, actor_headings=np.zeros((1, 3)))


class InverseTtcTests(unittest.TestCase):
    def test_first_hit_no_hit_inclusive_threshold_and_mode_padding(self):
        ego_plan = np.zeros((3, 2), dtype=np.float32)
        predictions = np.full((1, 2, 3, 2), 10.0, dtype=np.float32)
        # The first mode reaches the inclusive x/y limits at future index 1.
        predictions[0, 0, 1] = (1.0, 0.5)

        values = compute_inverse_ttc(
            ego_plan, predictions, step_s=0.25,
            half_length_m=1.0, half_width_m=0.5, mode_count=4)

        self.assertEqual(values.shape, (1, 4))
        np.testing.assert_allclose(values[0], (2.0, 0.0, 0.0, 0.0))

    def test_empty_neighbors(self):
        values = compute_inverse_ttc(
            np.zeros((3, 2), dtype=np.float32),
            np.zeros((0, 2, 3, 2), dtype=np.float32),
            step_s=0.1, half_length_m=1.0, half_width_m=1.0,
            mode_count=3)
        self.assertEqual(values.shape, (0, 3))

    def test_invalid_shapes_are_rejected(self):
        valid_plan = np.zeros((3, 2), dtype=np.float32)
        valid_predictions = np.zeros((1, 2, 3, 2), dtype=np.float32)
        cases = [
            (np.zeros(3), valid_predictions),
            (valid_plan, np.zeros((1, 3, 2))),
            (valid_plan, np.zeros((1, 2, 4, 2))),
            (valid_plan, np.zeros((1, 2, 3, 3))),
        ]
        for plan, predictions in cases:
            with self.subTest(plan_shape=plan.shape,
                              prediction_shape=predictions.shape):
                with self.assertRaises(ValueError):
                    compute_inverse_ttc(
                        plan, predictions, step_s=0.1,
                        half_length_m=1.0, half_width_m=1.0)

    def test_probability_padding_and_aggregation(self):
        probabilities = mode_probabilities_for_display(
            np.asarray(((0.25, 0.75), (0.50, 0.50))), 3)
        np.testing.assert_allclose(
            probabilities,
            ((0.25, 0.75, 0.0), (0.50, 0.50, 0.0)))

        values = np.asarray(((5.0, 0.0, 0.0), (10.0, 2.0, 0.0)))
        worst, expected = aggregate_inverse_ttc(values, probabilities)
        np.testing.assert_allclose(worst, (5.0, 10.0))
        np.testing.assert_allclose(expected, (1.25, 6.0))


class ConstantVelocityPlanTests(unittest.TestCase):
    def test_stationary_plan(self):
        plan = constant_velocity_plan(((3.0, 4.0), (3.0, 4.0)), 3)
        np.testing.assert_allclose(plan, ((3.0, 4.0),) * 3)

    def test_moving_plan(self):
        plan = constant_velocity_plan(((0.0, 0.0), (1.0, 2.0)), 3)
        np.testing.assert_allclose(plan, ((2.0, 4.0), (3.0, 6.0), (4.0, 8.0)))


class PositionHistoryBufferTests(unittest.TestCase):
    def test_exact_readiness_and_departure_reentry_reset(self):
        histories = PositionHistoryBuffer(3)
        histories.observe({"vehicle": (0.0, 0.0)})
        histories.observe({"vehicle": (1.0, 0.0)})
        self.assertEqual(histories.sample_count("vehicle"), 2)
        self.assertFalse(histories.ready("vehicle"))

        histories.observe({"vehicle": (2.0, 0.0)})
        self.assertEqual(histories.sample_count("vehicle"), 3)
        self.assertTrue(histories.ready("vehicle"))
        np.testing.assert_allclose(
            histories.array("vehicle"),
            ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)))

        histories.observe({})
        self.assertEqual(histories.sample_count("vehicle"), 0)
        self.assertFalse(histories.ready("vehicle"))
        self.assertNotIn("vehicle", histories.actor_ids())

        histories.observe({"vehicle": (9.0, 1.0)})
        self.assertEqual(histories.sample_count("vehicle"), 1)
        self.assertFalse(histories.ready("vehicle"))
        np.testing.assert_allclose(histories.array("vehicle"), ((9.0, 1.0),))


class FakePredictor(object):
    history_length = 3
    prediction_length = 3
    mode_count = 2
    device_name = "fake-cpu"

    def __init__(self):
        self.calls = []
        self.close_count = 0

    def predict(self, histories, ego_plan):
        histories = np.asarray(histories, dtype=np.float32)
        ego_plan = np.asarray(ego_plan, dtype=np.float32)
        self.calls.append((histories.copy(), ego_plan.copy()))

        actor_count = histories.shape[0]
        predictions = np.full(
            (actor_count, self.mode_count, self.prediction_length, 2),
            10.0, dtype=np.float32)
        probabilities = np.zeros((actor_count, self.mode_count), dtype=np.float32)
        for index in range(actor_count):
            # The tests use x=1 for actor-b and x=2 for actor-a.  Distinct hit
            # times make an actor/index mapping error visible in the payload.
            if histories[index, -1, 0] < 1.5:
                predictions[index, 0, 0] = (1.0, 0.0)
                probabilities[index] = (0.50, 0.50)
            else:
                predictions[index, 0, 1] = (1.0, 0.0)
                probabilities[index] = (0.25, 0.75)
        return predictions, probabilities

    def close(self):
        self.close_count += 1


class HeadingAwareFakePredictor(FakePredictor):
    supports_actor_headings = True

    def __init__(self):
        super(HeadingAwareFakePredictor, self).__init__()
        self.received_headings = []

    def predict(self, histories, ego_plan, actor_headings=None):
        self.received_headings.append(np.asarray(actor_headings).copy())
        return super(HeadingAwareFakePredictor, self).predict(
            histories, ego_plan)


class FakeDashboard(object):
    def __init__(self):
        self.frames = []
        self.close_count = 0

    def publish(self, frame):
        self.frames.append(frame)

    def close(self):
        self.close_count += 1


class HiddenDangerModePredictor(object):
    history_length = 2
    prediction_length = 2
    mode_count = 3
    device_name = "fake-cpu"

    def predict(self, histories, ego_plan):
        actor_count = histories.shape[0]
        predictions = np.full(
            (actor_count, 3, 2, 2), 100.0, dtype=np.float32)
        predictions[:, 2] = ego_plan
        probabilities = np.tile(
            np.asarray((0.1, 0.1, 0.8), dtype=np.float32),
            (actor_count, 1))
        return predictions, probabilities

    def close(self):
        return None


class DrawingMustStayDisabled(object):
    @property
    def debug(self):
        raise AssertionError("drawing was accessed while disabled")


def monitor_settings():
    return {
        "pace_realtime": False,
        "update_interval_s": 0.1,
        "neighbor_radius_m": 20.0,
        "max_neighbors": 3,
        "mode_count": 3,
        "overlap_half_x_m": 1.0,
        "overlap_half_y_m": 1.0,
        "model": {
            "config": "fake-model.yaml",
            "checkpoint": "fake-checkpoint.pth",
            "device": "cpu",
            "seed": 1,
            "input_scale": 1.0,
            "output_scale": 1.0,
        },
        "dashboard": {
            "enabled": False,
            "history_length": 20,
            "startup_timeout_s": 1.0,
        },
        "drawing": {
            "enabled": False,
            "mode_count": 1,
            "life_time_s": 0.1,
        },
    }


class PredictionRiskMonitorTests(unittest.TestCase):
    def test_passes_synchronized_actor_heading_to_capable_predictor(self):
        predictor = HeadingAwareFakePredictor()
        dashboard = FakeDashboard()
        monitor = PredictionRiskMonitor(
            monitor_settings(), step_length=0.1,
            predictor=predictor, dashboard=dashboard)
        ego = {"position": (0.0, 0.0)}
        other = {
            "actor": {
                "position": (2.0, 0.0),
                "heading": (0.0, -2.0),
            },
        }

        monitor.observe(0.0, ego, other)
        monitor.observe(0.1, ego, other)
        monitor.observe(0.2, ego, other)

        self.assertEqual(len(predictor.received_headings), 1)
        np.testing.assert_allclose(
            predictor.received_headings[0], ((0.0, -2.0),))
        monitor.close()

    def test_rejects_simulation_step_that_does_not_match_model_samples(self):
        settings = monitor_settings()
        settings["sample_step_s"] = 0.05
        with self.assertRaisesRegex(ValueError, "expects 0.050000 s samples"):
            PredictionRiskMonitor(
                settings, step_length=0.1,
                predictor=FakePredictor(), dashboard=FakeDashboard())

    def test_aggregates_all_model_modes_when_dashboard_shows_fewer(self):
        settings = monitor_settings()
        settings["mode_count"] = 2
        predictor = HiddenDangerModePredictor()
        dashboard = FakeDashboard()
        monitor = PredictionRiskMonitor(
            settings, step_length=0.1,
            predictor=predictor, dashboard=dashboard)
        ego = {"position": (0.0, 0.0)}
        other = {"actor": {"position": (3.0, 0.0)}}

        monitor.observe(0.0, ego, other)
        frame = monitor.observe(0.1, ego, other)
        actor = frame["actors"][0]

        np.testing.assert_allclose(actor["inverse_ttc_s_inv"], (0.0, 0.0))
        self.assertEqual(actor["evaluated_mode_count"], 3)
        self.assertEqual(actor["displayed_mode_count"], 2)
        self.assertAlmostEqual(actor["worst_inverse_ttc_s_inv"], 10.0)
        self.assertAlmostEqual(actor["expected_inverse_ttc_s_inv"], 8.0)
        monitor.close()

    def test_independent_warmup_stable_ids_exact_payload_and_close(self):
        predictor = FakePredictor()
        dashboard = FakeDashboard()

        real_import = builtins.__import__
        blocked = (
            "torch", "carla", "pyqtgraph",
            "carla_reconstruction.closed_loop.risk_dashboard",
        )

        def import_without_optional_backends(name, *args, **kwargs):
            if name == blocked[3] or name.split(".", 1)[0] in blocked[:3]:
                raise AssertionError("optional backend imported: %s" % name)
            return real_import(name, *args, **kwargs)

        with mock.patch("builtins.__import__",
                        side_effect=import_without_optional_backends):
            monitor = PredictionRiskMonitor(
                monitor_settings(), step_length=0.1,
                predictor=predictor, dashboard=dashboard)
            world = DrawingMustStayDisabled()
            ego = {"position": (0.0, 0.0), "z": 0.0}

            monitor.observe(0.0, ego, {"actor-a": {"position": (2.0, 0.0)}}, world)
            monitor.observe(0.1, ego, {"actor-a": {"position": (2.0, 0.0)}}, world)
            frame = monitor.observe(
                0.2, ego,
                {
                    "actor-a": {"position": (2.0, 0.0)},
                    "actor-b": {"position": (1.0, 0.0)},
                }, world)

            self.assertEqual([row["actor_id"] for row in frame["actors"]],
                             ["actor-b", "actor-a"])
            self.assertIn("1 vehicle(s) warming", frame["status"])
            actor_b, actor_a = frame["actors"]
            self.assertFalse(actor_b["ready"])
            self.assertEqual(actor_b["history_samples"], 1)
            self.assertTrue(actor_a["ready"])
            self.assertEqual(actor_a["history_samples"], 3)
            np.testing.assert_allclose(
                actor_a["inverse_ttc_s_inv"], (5.0, 0.0, 0.0))
            np.testing.assert_allclose(
                actor_a["probabilities"], (0.25, 0.75, 0.0))
            self.assertAlmostEqual(actor_a["worst_inverse_ttc_s_inv"], 5.0)
            self.assertAlmostEqual(actor_a["expected_inverse_ttc_s_inv"], 1.25)
            self.assertEqual(len(predictor.calls), 1)
            np.testing.assert_allclose(predictor.calls[0][0][:, -1, 0], (2.0,))

            # Reverse input insertion order. Selection and payload association
            # remain distance/ID based rather than dictionary-order based.
            monitor.observe(
                0.3, ego,
                {
                    "actor-b": {"position": (1.0, 0.0)},
                    "actor-a": {"position": (2.0, 0.0)},
                }, world)
            final_frame = monitor.observe(
                0.4, ego,
                {
                    "actor-a": {"position": (2.0, 0.0)},
                    "actor-b": {"position": (1.0, 0.0)},
                }, world)

            self.assertEqual(
                [row["actor_id"] for row in final_frame["actors"]],
                ["actor-b", "actor-a"])
            self.assertEqual(final_frame["status"], "Live HGT prediction")
            actor_b, actor_a = final_frame["actors"]
            self.assertTrue(actor_b["ready"])
            self.assertTrue(actor_a["ready"])
            np.testing.assert_allclose(
                actor_b["inverse_ttc_s_inv"], (10.0, 0.0, 0.0))
            self.assertAlmostEqual(actor_b["worst_inverse_ttc_s_inv"], 10.0)
            self.assertAlmostEqual(actor_b["expected_inverse_ttc_s_inv"], 5.0)
            np.testing.assert_allclose(
                actor_a["inverse_ttc_s_inv"], (5.0, 0.0, 0.0))
            self.assertAlmostEqual(actor_a["expected_inverse_ttc_s_inv"], 1.25)
            np.testing.assert_allclose(
                predictor.calls[-1][0][:, -1, 0], (1.0, 2.0))
            self.assertIs(dashboard.frames[-1], final_frame)

            metadata = monitor.metadata()
            self.assertEqual(metadata["device"], "fake-cpu")
            self.assertEqual(metadata["history_length"], 3)
            self.assertEqual(metadata["prediction_length"], 3)
            self.assertEqual(metadata["model_modes"], 2)
            self.assertEqual(metadata["display_modes"], 3)
            self.assertEqual(metadata["output_frame"], "unspecified")
            self.assertEqual(metadata["model_config"], "fake-model.yaml")
            self.assertEqual(metadata["checkpoint"], "fake-checkpoint.pth")

            monitor.close()
            monitor.close()
            self.assertEqual(dashboard.close_count, 1)
            self.assertEqual(predictor.close_count, 1)
            published_count = len(dashboard.frames)
            self.assertIsNone(monitor.observe(0.5, ego, {}, world))
            self.assertEqual(len(dashboard.frames), published_count)


if __name__ == "__main__":
    unittest.main()
