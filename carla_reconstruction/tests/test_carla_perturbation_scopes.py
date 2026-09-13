"""Verify perturbation isolation at generation and actual TM API dispatch."""

import copy
import json
import os
import sys
import tempfile
import types
import unittest
from unittest import mock

sys.modules.setdefault("carla", types.ModuleType("carla"))

from closed_loop.safety_variants import (
    carla_behavior_for_track, validate_carla_variant)
from closed_loop.tracks import ActorTrack, TrackPoint, TrackBundle
from tools import generate_carla_safety_variants as generator
from runtime import run_tm_closed_loop as runner


def track(actor_id, positions, start=0.0, category="vehicle.car"):
    return ActorTrack(actor_id, category, [
        TrackPoint(i, start + i * 0.5, x, 0.0, 0.0)
        for i, x in enumerate(positions)
    ], is_ego=actor_id == "ego")


def bundle():
    actors = [track("moving", [0, 5]), track("later", [30, 35], start=0.1),
              track("parked", [50, 50]), track("single", [70]),
              track("walker", [10, 12], category="human.pedestrian.adult")]
    return TrackBundle("test", 0.5, track("ego", [0, 5]),
                       {actor.actor_id: actor for actor in actors})


MANIFEST = {"scene": "test", "map": {"runtime_name": "TestMap"}}


def generate(target, population=None, profile="aggressive"):
    args = generator.build_parser().parse_args([
        "--manifest", "manifest.json", "--target", target, "--count", "3", "--profile", profile])
    written = {}

    def capture(path, value):
        written[os.path.basename(path)] = copy.deepcopy(value)

    with mock.patch.object(generator, "load_manifest_tracks",
                           return_value=(MANIFEST, population or bundle())), \
            mock.patch.object(generator.os, "makedirs"), \
            mock.patch.object(generator, "_validate_output_scope"), \
            mock.patch.object(generator, "prune_stale_numbered_variants", return_value=[]), \
            mock.patch("builtins.print"), \
            mock.patch.object(generator, "write_json", side_effect=capture):
        generator.run(args)
    return written


def expected_speed(behavior, recorded_kmh=36.0):
    return min(max(3.0, recorded_kmh * behavior["desired_speed_scale"],
                   behavior.get("target_speed_floor_kmh", 0.0)),
               behavior.get("target_speed_ceiling_kmh", 0.0) or float("inf"))


class CarlaScopeGenerationTests(unittest.TestCase):
    def test_two_families_have_identical_baselines_and_isolated_samples(self):
        ego, surrounding = generate("ego"), generate("surrounding")
        for family in (ego, surrounding):
            self.assertEqual(family["baseline.json"]["ego_mode"], "tm")
            self.assertEqual(family, generate(
                family["index.json"]["perturbation_target"]))
            self.assertEqual(family["baseline.json"]["selection"][
                "retained_vehicle_track_ids"], ["later", "moving", "parked", "single"])
        for actor_id in ("ego", "moving", "later"):
            baseline = carla_behavior_for_track(ego["baseline.json"], actor_id)
            self.assertEqual(baseline, carla_behavior_for_track(
                surrounding["baseline.json"], actor_id))
            changed_ego = carla_behavior_for_track(ego["variant_000.json"], actor_id)
            changed_surrounding = carla_behavior_for_track(
                surrounding["variant_000.json"], actor_id)
            if actor_id == "ego":
                self.assertNotEqual(changed_ego, baseline)
                self.assertEqual(changed_surrounding, baseline)
            else:
                self.assertEqual(changed_ego, baseline)
                self.assertNotEqual(changed_surrounding, baseline)
        for actor_id in ("parked", "single", "walker"):
            for family in (ego, surrounding):
                self.assertIsNone(carla_behavior_for_track(
                    family["variant_000.json"], actor_id))
        self.assertIn(os.path.join("carla_safety_variants", "ego"),
                      ego["index.json"]["baseline_config"])
        self.assertIn(os.path.join("carla_safety_variants", "surrounding"),
                      surrounding["index.json"]["baseline_config"])

    def test_ego_case_does_not_require_moving_surrounding_vehicles(self):
        population = bundle()
        population.actors = {}
        doc = generate("ego", population)["variant_000.json"]
        self.assertEqual(doc["selection"]["eligible_track_ids"], ["ego"])
        self.assertEqual(doc["selection"]["retained_vehicle_track_ids"], [])
        with self.assertRaisesRegex(ValueError, "no moving surrounding"):
            generate("surrounding", population)

    def test_rejects_cross_scope_targets_and_invalid_baseline(self):
        for target, bad_ids in (("ego", ["moving"]), ("surrounding", ["ego"])):
            doc = generate(target)["variant_000.json"]
            doc["selection"]["eligible_track_ids"] = bad_ids
            with self.assertRaises(ValueError):
                validate_carla_variant(doc)
        doc = generate("ego")["variant_000.json"]
        doc["baseline_behavior"]["leading_distance_m"] = -1
        with self.assertRaisesRegex(ValueError, "baseline_behavior"):
            validate_carla_variant(doc)

    def test_rejects_output_reuse_across_targets_before_writes(self):
        ego = generate("ego")["baseline.json"]
        surrounding = generate("surrounding")["baseline.json"]
        with mock.patch.object(generator.os.path, "isfile", return_value=True), \
                mock.patch.object(generator, "load_json", return_value=ego):
            generator._validate_output_scope("ego-folder", ego)
            with self.assertRaisesRegex(ValueError, "separate --output"):
                generator._validate_output_scope("ego-folder", surrounding)

    def test_version_one_keeps_existing_surrounding_only_semantics(self):
        doc = generate("surrounding")["variant_000.json"]
        doc["schema_version"] = 1
        for key in ("ego_mode", "baseline_behavior"):
            del doc[key]
        del doc["selection"]["moving_surrounding_track_ids"]
        doc = validate_carla_variant(doc)
        self.assertIsNone(carla_behavior_for_track(doc, "ego"))
        self.assertEqual(carla_behavior_for_track(doc, "moving"), doc["behavior"])
        self.assertEqual(runner._resolve_ego_mode(None, doc), "replay")
        self.assertEqual(runner._resolve_ego_mode("tm", doc), "tm")


class CarlaScopeRuntimeTests(unittest.TestCase):
    def test_incompatible_ego_modes_fail_before_connecting_to_carla(self):
        for target in ("ego", "surrounding"):
            doc = generate(target)["variant_000.json"]
            for mode in ("replay", "external"):
                args = runner.build_parser().parse_args([
                    "--variant-config", "variant.json", "--ego-mode", mode])
                with mock.patch.object(runner, "load_carla_variant", return_value=doc), \
                        mock.patch.object(runner.carla, "Client", create=True) as client:
                    with self.assertRaisesRegex(ValueError, "requires --ego-mode tm"):
                        runner.run(args)
                    client.assert_not_called()

    def test_changed_population_rejected_for_both_scopes_before_connecting(self):
        for target in ("ego", "surrounding"):
            doc = generate(target)["variant_000.json"]
            population = bundle()
            population.actors["moving"] = track("moving", [0, 0])
            args = runner.build_parser().parse_args([
                "--variant-config", "variant.json"])
            with mock.patch.object(runner, "load_carla_variant", return_value=doc), \
                    mock.patch.object(runner, "load_manifest_tracks",
                                      return_value=(MANIFEST, population)), \
                    mock.patch.object(runner.carla, "Client", create=True) as client:
                with self.assertRaisesRegex(ValueError, "preserve every moving"):
                    runner.run(args)
                client.assert_not_called()

    def _run(self, document, fail_configuration=False):
        population = bundle()
        actors = {actor_id: mock.Mock(
            id=i + 1, is_alive=True, type_id="vehicle.test")
            for i, actor_id in enumerate(["ego"] + list(population.actors))}
        world = mock.Mock()
        world.get_settings.side_effect = [types.SimpleNamespace(), types.SimpleNamespace()]
        tm = mock.Mock()
        if fail_configuration:
            tm.set_desired_speed.side_effect = RuntimeError("TM rejected speed")
        client = mock.Mock()
        client.load_world.return_value = world
        client.get_trafficmanager.return_value = tm
        metrics = mock.Mock()
        metrics.write.return_value = ("metrics.csv", "summary.json")
        spawn_calls = {}

        def spawn(_world, actor_track, _projector, role_name, physics=True):
            spawn_calls[actor_track.actor_id] = physics
            return actors[actor_track.actor_id], 0.0

        with tempfile.TemporaryDirectory() as directory:
            args = runner.build_parser().parse_args([
                "--variant-config", "variant.json", "--duration", "0.15",
                "--replay-pedestrians", "--output", directory,
                # Saved profiles must govern non-target actors too.
                "--leading-distance", "9", "--auto-lane-change"])
            with mock.patch.object(runner, "load_carla_variant", return_value=document), \
                    mock.patch.object(runner, "load_manifest_tracks",
                                      return_value=(MANIFEST, population)), \
                    mock.patch.object(runner.carla, "Client", return_value=client, create=True), \
                    mock.patch.object(runner.carla, "Location", types.SimpleNamespace, create=True), \
                    mock.patch.object(runner, "GroundProjector"), \
                    mock.patch.object(runner, "spawn_track_actor", side_effect=spawn), \
                    mock.patch.object(runner, "CollisionMonitor"), \
                    mock.patch.object(runner, "SafetyMetrics", return_value=metrics), \
                    mock.patch.object(runner, "actor_state", return_value={}), \
                    mock.patch.object(runner, "chase_transform"), \
                    mock.patch.object(runner, "place_from_point") as place:
                if fail_configuration:
                    with self.assertRaisesRegex(RuntimeError, "TM rejected speed"):
                        runner.run(args)
                else:
                    runner.run(args)
                replayed_ids = {c.args[0].id for c in place.call_args_list}
            with open(os.path.join(directory, "run_config.json"), encoding="utf-8") as stream:
                metadata = json.load(stream)
        return actors, tm, spawn_calls, metadata, world, replayed_ids

    def test_only_requested_group_receives_perturbed_tm_commands(self):
        for target in ("ego", "surrounding"):
            with self.subTest(target=target):
                doc = generate(target)["variant_000.json"]
                actors, tm, spawn_calls, metadata, world, replayed = self._run(doc)
                self.assertEqual(metadata["ego_mode"], "tm")
                self.assertEqual(world.tick.call_count, 4)
                selected = {"ego"} if target == "ego" else {"moving", "later"}
                audit = metadata["carla_behavior_variant"]
                self.assertEqual(set(audit["applied_track_ids"]), selected)
                self.assertEqual(audit["unapplied_track_ids"], [])
                self.assertEqual(set(audit["all_tm_actor_settings"]), {"ego", "moving", "later"})
                for actor_id in ("ego", "moving", "later"):
                    expected = carla_behavior_for_track(doc, actor_id)
                    actor = actors[actor_id]
                    self.assertTrue(spawn_calls[actor_id])
                    tm.set_desired_speed.assert_any_call(actor, expected_speed(expected))
                    tm.distance_to_leading_vehicle.assert_any_call(actor, expected["leading_distance_m"])
                    tm.auto_lane_change.assert_any_call(actor, expected["auto_lane_change"])
                    tm.random_left_lanechange_percentage.assert_any_call(
                        actor, expected["random_left_lane_change_percentage"])
                    tm.random_right_lanechange_percentage.assert_any_call(
                        actor, expected["random_right_lane_change_percentage"])
                    tm.keep_right_rule_percentage.assert_any_call(actor, expected["keep_right_rule_percentage"])
                    for name in ("ignore_vehicles_percentage", "ignore_walkers_percentage",
                                 "ignore_lights_percentage", "ignore_signs_percentage"):
                        getattr(tm, name).assert_any_call(actor, expected.get(name, 0.0))
                    self.assertEqual(audit["all_tm_actor_settings"][actor_id]["behavior"]["desired_speed_kmh"],
                                     expected_speed(expected))
                    if actor_id not in selected:
                        self.assertEqual(expected["ignore_vehicles_percentage"], 0.0)
                    self.assertEqual(metadata["actor_authority"][actor_id], "carla_tm")
                self.assertEqual(tm.set_desired_speed.call_count, 3)
                self.assertAlmostEqual(audit["all_tm_actor_settings"]["later"]["applied_at_s"], 0.1)
                for actor_id in ("parked", "single", "walker"):
                    self.assertFalse(spawn_calls[actor_id])
                self.assertEqual(replayed, {actors["walker"].id})

    def test_configuration_failure_is_reported_and_ego_cleaned_up(self):
        doc = generate("ego")["variant_000.json"]
        actors, tm, _, metadata, world, _ = self._run(doc, fail_configuration=True)
        self.assertEqual(metadata["termination_reason"], "error")
        self.assertEqual(metadata["error"], "TM rejected speed")
        self.assertEqual(metadata["carla_behavior_variant"]["unapplied_track_ids"], ["ego"])
        actors["ego"].destroy.assert_called_once()
        tm.set_synchronous_mode.assert_called_with(False)
        self.assertEqual(world.apply_settings.call_count, 2)


if __name__ == "__main__":
    unittest.main()
