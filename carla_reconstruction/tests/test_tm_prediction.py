"""Keep optional CARLA prediction observational and isolated from TM control."""

import argparse
import copy
import json
import os
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.modules.setdefault("carla", types.ModuleType("carla"))

from runtime import run_tm_closed_loop as runner
from carla_reconstruction.closed_loop import carla_prediction as adapter
from carla_reconstruction.closed_loop.prediction_risk import (
    PredictionRiskMonitor, prediction_risk_settings)
from carla_reconstruction.closed_loop.tracks import ActorTrack, TrackBundle, TrackPoint


def arguments(*flags):
    parser = argparse.ArgumentParser()
    adapter.add_prediction_arguments(parser)
    return parser.parse_args(list(flags))


def fake_actor(actor_id, z=0.7, alive=True):
    actor = mock.Mock(id=actor_id, is_alive=alive, type_id="vehicle.test")
    actor.get_location.return_value = types.SimpleNamespace(z=z)
    return actor


def actor_state(x=0.0, velocity=(0.0, 0.0), heading=(-1.0, 0.0)):
    return {"position": (x, 2.0), "velocity": velocity,
            "heading": heading, "radius": 1.5}


def mock_monitor():
    monitor = mock.Mock(realtime_pacing=False)
    monitor.metadata.return_value = {"device": "test", "history_length": 5}
    monitor.observe.return_value = None
    return monitor


class PredictionArgumentTests(unittest.TestCase):
    def start_observer(self, flags, config_path="manifest.json"):
        monitor = mock_monitor()
        with mock.patch.object(adapter, "_normalise_settings",
                               wraps=prediction_risk_settings) as normalise, \
                mock.patch.object(adapter, "_create_monitor", return_value=monitor) as create:
            observer = adapter.CarlaPredictionObserver(
                arguments(*flags), config_path, ["moving"], ["parked"])
            observer.start(0.05)
        return observer, monitor, normalise, create

    def test_disabled_defaults_do_not_import_prediction_dependencies(self):
        observer, monitor, normalise, create = self.start_observer([])
        normalise.assert_not_called()
        create.assert_not_called()
        observer.observe(0, {}, {}, None, {}, None)
        observer.close()
        monitor.observe.assert_not_called()

    def test_enabled_aliases_share_defaults_and_explicit_visual_switches(self):
        cases = [
            (["--enable-prediction-risk"], True, False),
            (["--prediction-risk"], True, False),
            (["--draw-predictions"], True, True),
            (["--prediction-dashboard"], True, False),
            (["--prediction-risk", "--no-prediction-dashboard", "--draw-predictions"], False, True),
            (["--prediction-risk", "--prediction-dashboard", "--no-draw-predictions"], True, False),
        ]
        for flags, dashboard, drawing in cases:
            with self.subTest(flags=flags):
                observer, monitor, normalise, create = self.start_observer(flags)
                settings, step = create.call_args.args
                self.assertTrue(settings["enabled"])
                self.assertEqual(settings["dashboard"]["enabled"], dashboard)
                self.assertEqual(settings["drawing"]["enabled"], drawing)
                self.assertEqual(settings["max_neighbors"], 3)
                self.assertEqual(settings["mode_count"], 5)
                self.assertEqual(step, 0.05)
                observer.close()
                monitor.close.assert_called_once()

    def test_master_disable_wins_and_negative_visual_switches_do_not_enable(self):
        for flags in (["--no-prediction-dashboard"], ["--no-draw-predictions"],
                      ["--no-prediction-risk", "--draw-predictions"],
                      ["--prediction-dashboard", "--disable-prediction-risk"]):
            with self.subTest(flags=flags):
                observer, _, normalise, create = self.start_observer(flags)
                create.assert_not_called()
                normalise.assert_not_called()
                observer.close()

    def test_config_paths_resolve_relative_to_json_and_cli_overrides_config(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "prediction.json")
            with open(path, "w", encoding="utf-8") as stream:
                json.dump({"prediction_risk": {
                    "enabled": True,
                    "model": {"config": "hgt.yaml", "checkpoint": "weights.pth", "device": "cuda"},
                    "dashboard": {"enabled": False},
                    "drawing": {"enabled": True},
                }}, stream)
            flags = ["--prediction-config", path, "--prediction-device", "cpu",
                     "--prediction-dashboard", "--no-draw-predictions"]
            observer, _, _, create = self.start_observer(flags)
            settings = create.call_args.args[0]
            self.assertEqual(settings["model"]["device"], "cpu")
            self.assertEqual(settings["model"]["config"], os.path.join(directory, "hgt.yaml"))
            self.assertEqual(settings["model"]["checkpoint"], os.path.join(directory, "weights.pth"))
            self.assertTrue(settings["dashboard"]["enabled"])
            self.assertFalse(settings["drawing"]["enabled"])
            observer.close()

    def test_config_can_enable_prediction_without_an_enable_switch(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "prediction.json")
            with open(path, "w", encoding="utf-8") as stream:
                json.dump({"prediction_risk": {"enabled": True}}, stream)
            observer, _, _, create = self.start_observer(["--prediction-config", path])
            create.assert_called_once()
            self.assertTrue(create.call_args.args[0]["enabled"])
            observer.close()

    def test_master_disable_does_not_even_read_a_broken_config_path(self):
        with mock.patch("builtins.open") as opener:
            observer, _, normalise, create = self.start_observer([
                "--prediction-config", "missing-file.json",
                "--prediction-dashboard", "--no-prediction-risk"])
        opener.assert_not_called()
        normalise.assert_not_called()
        create.assert_not_called()
        observer.close()

    def test_disabled_import_in_fresh_python_is_lightweight(self):
        package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        code = """import argparse, sys
from carla_reconstruction.closed_loop.carla_prediction import (
    add_prediction_arguments, CarlaPredictionObserver)
p = argparse.ArgumentParser()
add_prediction_arguments(p)
o = CarlaPredictionObserver(p.parse_args([]), 'manifest.json', [], [])
o.start(0.05)
o.close()
assert not any(name in sys.modules for name in (
    'numpy', 'torch', 'matplotlib', 'tkinter',
    'carla_reconstruction.closed_loop.prediction_risk',
    'carla_reconstruction.closed_loop.hgt_predictor',
    'carla_reconstruction.closed_loop.risk_dashboard'))
"""
        result = subprocess.run([sys.executable, "-c", code], cwd=package_root,
                                capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class PredictionObservationTests(unittest.TestCase):
    def observer(self, monitor=None):
        monitor = monitor or mock_monitor()
        with mock.patch.object(adapter, "_normalise_settings", wraps=prediction_risk_settings), \
                mock.patch.object(adapter, "_create_monitor", return_value=monitor):
            observer = adapter.CarlaPredictionObserver(
                arguments("--prediction-risk", "--draw-predictions"),
                "manifest.json", ["moving", "stopped", "dead", "unspawned"], ["parked"])
            observer.start(0.05)
        return observer, monitor

    def test_only_live_recorded_movers_are_observed_and_metric_state_is_unchanged(self):
        observer, monitor = self.observer()
        ego_state = actor_state(10, heading=(0.0, 1.0))
        others = {name: actor_state(i) for i, name in enumerate(
            ("moving", "stopped", "parked", "walker", "dead", "unspawned"))}
        # A genuine mover stopped for a signal is still a relevant neighbor.
        others["stopped"]["velocity"] = (0.0, 0.0)
        before = copy.deepcopy((ego_state, others))
        actors = {name: fake_actor(i, z=0.2 + i) for i, name in enumerate(
            ("moving", "stopped", "parked", "walker", "dead"))}
        actors["dead"].is_alive = False
        ego = fake_actor(99, z=3.1)
        world = mock.Mock()
        observer.observe(1.5, ego_state, others, ego, actors, world)
        call = monitor.observe.call_args
        self.assertEqual(call.args[0], 1.5)
        observed_ego, observed_others = call.args[1:3]
        self.assertEqual(set(observed_others), {"moving", "stopped"})
        self.assertEqual(observed_ego["z"], 3.1)
        self.assertEqual(observed_others["stopped"]["z"], 1.2)
        self.assertEqual(observed_ego["heading"], (0.0, 1.0))
        self.assertEqual(observed_others["moving"]["heading"], (-1.0, 0.0))
        self.assertIs(call.kwargs["world"], world)
        self.assertEqual((ego_state, others), before)
        for actor in [ego] + list(actors.values()):
            actor.apply_control.assert_not_called()
            actor.set_transform.assert_not_called()
            actor.set_autopilot.assert_not_called()
        world.tick.assert_not_called()
        observer.close()

    def test_dependency_failure_is_contained_and_reported(self):
        with mock.patch.object(adapter, "_normalise_settings", wraps=prediction_risk_settings), \
                mock.patch.object(adapter, "_create_monitor", side_effect=ImportError("torch unavailable")):
            observer = adapter.CarlaPredictionObserver(
                arguments("--prediction-risk"), "manifest.json", ["moving"], [])
            observer.start(0.05)
        observer.observe(0, {}, {}, None, {}, None)
        observer.close()
        self.assertIn("torch unavailable", json.dumps(observer.metadata()))

    def test_cadence_failure_is_contained_without_loading_model(self):
        predictor = mock.Mock()
        def create(settings, step):
            return PredictionRiskMonitor(settings, step, predictor=predictor)
        with mock.patch.object(adapter, "_normalise_settings", wraps=prediction_risk_settings), \
                mock.patch.object(adapter, "_create_monitor", side_effect=create):
            observer = adapter.CarlaPredictionObserver(
                arguments("--prediction-risk"), "manifest.json", [], [])
            observer.start(0.1)
        self.assertEqual(observer.metadata()["error_stage"], "startup")
        self.assertIn("0.050000 s samples", observer.metadata()["error"])
        predictor.predict.assert_not_called()
        observer.close()

    def test_partial_startup_failure_closes_created_monitor(self):
        monitor = mock_monitor()
        monitor.metadata.side_effect = RuntimeError("invalid monitor metadata")
        observer, _ = self.observer(monitor)
        self.assertEqual(observer.metadata()["error_stage"], "startup")
        self.assertIn("invalid monitor metadata", observer.metadata()["error"])
        monitor.close.assert_called_once()
        observer.close()
        monitor.close.assert_called_once()

    def test_frame_counters_and_metadata_do_not_expose_mutable_internal_state(self):
        observer, monitor = self.observer()
        frame = {"sim_time_s": 2.0, "actors": [{"actor_id": "moving", "ready": True}]}
        monitor.observe.side_effect = [None, frame]
        for sim_time in (1.95, 2.0):
            observer.observe(sim_time, actor_state(), {}, fake_actor(1), {}, mock.Mock())
        frame["actors"][0]["ready"] = False
        audit = observer.metadata()
        self.assertEqual(audit["observed_ticks"], 2)
        self.assertEqual(audit["risk_frames"], 1)
        self.assertEqual(audit["ready_frames"], 1)
        self.assertEqual(audit["drawing_frames"], 1)
        self.assertTrue(audit["last_frame"]["actors"][0]["ready"])
        audit["last_frame"]["actors"].clear()
        self.assertEqual(len(observer.metadata()["last_frame"]["actors"]), 1)
        observer.close()

    def test_pacing_is_wall_time_only_and_inactive_after_close(self):
        observer, monitor = self.observer()
        with mock.patch.object(adapter.time, "sleep") as sleep, \
                mock.patch.object(adapter.time, "monotonic", return_value=10.02):
            observer.pace(10.0)
            sleep.assert_not_called()
            monitor.realtime_pacing = True
            observer.pace(10.0)
            self.assertAlmostEqual(sleep.call_args.args[0], 0.03)
            sleep.reset_mock()
            observer.pace(9.0)
            sleep.assert_not_called()
            observer.close()
            observer.pace(10.0)
            sleep.assert_not_called()

    def test_start_and_close_are_idempotent(self):
        monitor = mock_monitor()
        with mock.patch.object(adapter, "_normalise_settings", wraps=prediction_risk_settings), \
                mock.patch.object(adapter, "_create_monitor", return_value=monitor) as create:
            observer = adapter.CarlaPredictionObserver(
                arguments("--prediction-risk"), "manifest.json", [], [])
            observer.start(0.05)
            observer.start(0.05)
            observer.close()
            observer.close()
            observer.start(0.05)
        create.assert_called_once()
        monitor.close.assert_called_once()
        self.assertFalse(observer.metadata()["active"])
        self.assertTrue(observer.metadata()["closed"])

    def test_pacing_failure_disables_only_observer(self):
        observer, monitor = self.observer()
        monitor.realtime_pacing = True
        with mock.patch.object(adapter.time, "monotonic", side_effect=OSError("clock failed")):
            observer.pace(0.0)
        self.assertEqual(observer.metadata()["error_stage"], "pacing")
        monitor.close.assert_called_once()
        observer.pace(0.0)
        monitor.close.assert_called_once()

    def test_observation_and_close_failure_are_contained_without_retries(self):
        monitor = mock_monitor()
        monitor.observe.side_effect = RuntimeError("inference unavailable")
        monitor.close.side_effect = RuntimeError("dashboard cleanup unavailable")
        observer, _ = self.observer(monitor)
        observer.observe(0, actor_state(), {}, fake_actor(1), {}, mock.Mock())
        observer.observe(0.05, actor_state(), {}, fake_actor(1), {}, mock.Mock())
        observer.close()
        observer.close()
        self.assertEqual(monitor.observe.call_count, 1)
        monitor.close.assert_called_once()
        metadata = json.dumps(observer.metadata())
        self.assertIn("inference unavailable", metadata)
        self.assertIn("dashboard cleanup unavailable", metadata)

    def test_missing_or_malformed_configuration_is_nonfatal(self):
        with tempfile.TemporaryDirectory() as directory:
            for payload in (None, [], {"prediction_risk": []}):
                path = os.path.join(directory, "invalid.json")
                if payload is not None:
                    with open(path, "w", encoding="utf-8") as stream:
                        json.dump(payload, stream)
                with self.subTest(payload=payload), \
                        mock.patch.object(adapter, "_normalise_settings", wraps=prediction_risk_settings), \
                        mock.patch.object(adapter, "_create_monitor") as create:
                    observer = adapter.CarlaPredictionObserver(
                        arguments("--prediction-risk", "--prediction-config", path),
                        "manifest.json", [], [])
                    observer.start(0.05)
                    observer.close()
                    create.assert_not_called()
                    self.assertTrue(observer.metadata().get("error"))


def population():
    def track(actor_id, positions, category="vehicle.car"):
        return ActorTrack(actor_id, category, [
            TrackPoint(i, i * 0.5, x, 0.0, 0.0)
            for i, x in enumerate(positions)], is_ego=actor_id == "ego")
    actors = [track("moving", [10, 15]), track("parked", [30, 30]),
              track("walker", [20, 21], "human.pedestrian.adult")]
    return TrackBundle("test", 0.5, track("ego", [0, 5]),
                       {actor.actor_id: actor for actor in actors})


class PredictionRuntimeTests(unittest.TestCase):
    def run_baseline(self, enabled, fail_prediction=False):
        bundle = population()
        actors = {name: fake_actor(i + 1) for i, name in enumerate(
            ["ego"] + list(bundle.actors))}
        world, tm, client, metrics = mock.Mock(), mock.Mock(), mock.Mock(), mock.Mock()
        events = []
        world.get_settings.side_effect = [types.SimpleNamespace(), types.SimpleNamespace()]
        client.load_world.return_value = world
        client.get_trafficmanager.return_value = tm
        world.tick.side_effect = lambda: events.append("tick")
        metrics.update.side_effect = lambda *_: events.append("metrics")
        metrics.write.return_value = ("metrics.csv", "summary.json")
        monitor = mock_monitor()
        def observe(*_args, **_kwargs):
            events.append("prediction")
            if fail_prediction:
                raise RuntimeError("prediction failed")
        monitor.observe.side_effect = observe

        with tempfile.TemporaryDirectory() as directory:
            flags = ["--manifest", "manifest.json", "--ego-mode", "tm",
                     "--duration", "0.1", "--replay-pedestrians", "--output", directory]
            if enabled:
                flags += ["--prediction-risk", "--draw-predictions"]
            args = runner.build_parser().parse_args(flags)
            with mock.patch.object(runner, "load_manifest_tracks", return_value=(
                    {"scene": "test", "map": {"runtime_name": "TestMap"}}, bundle)), \
                    mock.patch.object(runner.carla, "Client", return_value=client, create=True), \
                    mock.patch.object(runner.carla, "Location", types.SimpleNamespace, create=True), \
                    mock.patch.object(runner, "prepare_ego_route", return_value=None), \
                    mock.patch.object(runner, "GroundProjector"), \
                    mock.patch.object(runner, "spawn_track_actor", side_effect=lambda _w, tr, *_a, **_k: (actors[tr.actor_id], 0.0)), \
                    mock.patch.object(runner, "CollisionMonitor"), \
                    mock.patch.object(runner, "SafetyMetrics", return_value=metrics), \
                    mock.patch.object(runner, "actor_state", side_effect=lambda a: actor_state(a.id)), \
                    mock.patch.object(runner, "chase_transform"), \
                    mock.patch.object(runner, "place_from_point"), \
                    mock.patch.object(adapter, "_normalise_settings", wraps=prediction_risk_settings), \
                    mock.patch.object(adapter, "_create_monitor", return_value=monitor), \
                    mock.patch("builtins.print"):
                runner.run(args)
            with open(os.path.join(directory, "run_config.json"), encoding="utf-8") as stream:
                metadata = json.load(stream)
        # Compare exact TM calls across independent runs by actor ID.
        def normalized(value):
            if isinstance(value, mock.Mock) and value in actors.values():
                return ("actor", value.id)
            if isinstance(value, (list, tuple)):
                return tuple(normalized(v) for v in value)
            return value
        tm_calls = [(call[0], normalized(call.args), call.kwargs) for call in tm.method_calls]
        return actors, world, monitor, events, metadata, tm_calls

    def test_prediction_is_post_tick_and_does_not_change_tm_control_dispatch(self):
        _, _, disabled_monitor, disabled_events, disabled_metadata, disabled_calls = self.run_baseline(False)
        actors, world, monitor, events, metadata, enabled_calls = self.run_baseline(True)
        self.assertEqual(disabled_events, ["tick", "metrics"] * 3)
        self.assertEqual(events, ["tick", "metrics", "prediction"] * 3)
        self.assertEqual(enabled_calls, disabled_calls)
        self.assertEqual(metadata["actor_authority"], disabled_metadata["actor_authority"])
        self.assertEqual(metadata["carla_behavior_variant"], disabled_metadata["carla_behavior_variant"])
        self.assertEqual(metadata["termination_reason"], "duration_reached")
        disabled_monitor.observe.assert_not_called()
        monitor.close.assert_called_once()
        for call in monitor.observe.call_args_list:
            self.assertEqual(set(call.args[2]), {"moving"})
        for actor in actors.values():
            actor.destroy.assert_called_once()
        self.assertEqual(world.apply_settings.call_count, 2)

    def test_prediction_failure_does_not_abort_simulation_or_cleanup(self):
        actors, world, monitor, events, metadata, _ = self.run_baseline(True, fail_prediction=True)
        self.assertEqual(events.count("tick"), 3)
        self.assertEqual(events.count("metrics"), 3)
        self.assertEqual(events.count("prediction"), 1)
        self.assertEqual(metadata["termination_reason"], "duration_reached")
        self.assertIsNone(metadata["error"])
        self.assertIn("prediction failed", json.dumps(metadata["prediction_risk"]))
        monitor.close.assert_called_once()
        for actor in actors.values():
            actor.destroy.assert_called_once()
        self.assertEqual(world.apply_settings.call_count, 2)


if __name__ == "__main__":
    unittest.main()
