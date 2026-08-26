import argparse
import copy
import os
import random
import types
import unittest
from unittest import mock

from closed_loop.safety_variants import (
    CARLA_PIPELINE, SUMO_PIPELINE, latin_hypercube,
    prune_stale_numbered_variants, validate_carla_variant,
    validate_sumo_variant)
from closed_loop.tracks import ActorTrack, TrackPoint
from tools import generate_carla_safety_variants, generate_sumo_safety_variants


def _sumo_baseline(tau, gap, accel, decel):
    return {
        "car_following": {
            "tau_s": tau,
            "min_gap_m": gap,
            "accel_mps2": accel,
            "decel_mps2": decel,
            "apparent_decel_mps2": decel,
            "emergency_decel_mps2": 9.0,
        },
        "lane_changing": {
            "lc_strategic": 1.0,
            "lc_cooperative": 0.6,
            "lc_speed_gain": 0.8,
            "lc_keep_right": 0.5,
            "lc_assertive": 1.0,
        },
    }


class SamplingTests(unittest.TestCase):
    def test_latin_hypercube_is_deterministic_and_stratified(self):
        bounds = [("x", (0.0, 1.0)), ("y", (10.0, 20.0))]
        left = latin_hypercube(8, bounds, random.Random(7))
        right = latin_hypercube(8, bounds, random.Random(7))

        self.assertEqual(left, right)
        self.assertEqual(len(left), 8)
        self.assertEqual(
            sorted(int(row["x"] * 8) for row in left), list(range(8)))
        self.assertTrue(all(10.0 <= row["y"] <= 20.0 for row in left))

    def test_prunes_only_obsolete_numbered_generator_outputs(self):
        with mock.patch("closed_loop.safety_variants.os.path.isdir",
                        return_value=True), \
                mock.patch("closed_loop.safety_variants.os.listdir",
                           return_value=[
                               "variant_000.json", "variant_004.json",
                               "notes.json", "variant_manual.json"]), \
                mock.patch("closed_loop.safety_variants.os.remove") as remove:
            removed = prune_stale_numbered_variants("variants", 2)

        self.assertEqual(removed, ["variant_004.json"])
        remove.assert_called_once_with(os.path.join(
            "variants", "variant_004.json"))


class SumoSafetyVariantTests(unittest.TestCase):
    def _base(self, critical=False, legacy_skip=False):
        rows = [
            {
                "id": "car-track",
                "sumo_id": "nusc_car",
                "sumo_type_id": "nusc_car_car-track",
                "sumo_max_speed_mps": None,
                "sumo_behavior_baseline": _sumo_baseline(
                    1.2, 2.5, 2.6, 4.5),
            },
            {
                "id": "bike-track",
                "sumo_id": "nusc_bike",
                "sumo_type_id": "nusc_bicycle_bike-track",
                "sumo_max_speed_mps": None,
                "sumo_behavior_baseline": _sumo_baseline(
                    1.0, 1.0, 1.2, 3.0),
            },
        ]
        report = {
            "included": rows,
            "carla_static": [{"id": "parked"}],
            "moving_speed": {"policy": "unbounded"},
            "skipped": ([{"id": "legacy", "reason": "CARLA authority"}]
                        if legacy_skip else []),
        }
        config = {
            "schema_version": 1,
            "manifest": "scene_manifest.json",
            "seed": 103,
            "sumo": {"route_report": "route_report.json"},
            "critical_actors": ([{
                "track_id": "legacy", "authority": "carla_reference",
            }] if critical else []),
            "static_actors": [{"track_id": "parked"}],
            "background": {
                "authority": "sumo",
                "track_ids": ["car-track", "bike-track"],
                "moving_speed": {"policy": "unbounded"},
            },
        }
        return config, report

    def _args(self, config, output):
        defaults = generate_sumo_safety_variants.DEFAULT_RANGES
        return argparse.Namespace(
            config=config, output=output, count=4, seed=19,
            tau_factor=defaults["tau_factor"],
            min_gap_factor=defaults["min_gap_factor"],
            accel_factor=defaults["accel_factor"],
            decel_factor=defaults["decel_factor"],
            lc_strategic=defaults["lc_strategic"],
            lc_cooperative=defaults["lc_cooperative"],
            lc_speed_gain=defaults["lc_speed_gain"],
            lc_keep_right=defaults["lc_keep_right"],
            lc_assertive=defaults["lc_assertive"],
        )

    def test_generates_matched_autonomous_baseline_and_all_mover_variants(self):
        base, report = self._base()
        written = {}

        def fake_load(path):
            return copy.deepcopy(
                report if os.path.basename(path) == "route_report.json" else base)

        def capture(path, value):
            written[os.path.basename(path)] = copy.deepcopy(value)

        with mock.patch.object(
                generate_sumo_safety_variants, "load_json",
                side_effect=fake_load), \
                mock.patch.object(
                    generate_sumo_safety_variants.os.path, "isfile",
                    return_value=True), \
                mock.patch.object(
                    generate_sumo_safety_variants.os, "makedirs"), \
                mock.patch.object(
                    generate_sumo_safety_variants,
                    "_route_type_definitions",
                    return_value=(
                        "routes.rou.xml",
                        {
                            "nusc_car_car-track": {},
                            "nusc_bicycle_bike-track": {},
                        },
                        {
                            "nusc_car": "nusc_car_car-track",
                            "nusc_bike": "nusc_bicycle_bike-track",
                        })), \
                mock.patch.object(
                    generate_sumo_safety_variants, "write_json",
                    side_effect=capture):
            index = generate_sumo_safety_variants.run(
                self._args("hybrid_config.json", "variants"))

        self.assertEqual(index["pipeline"], SUMO_PIPELINE)
        self.assertEqual(len(index["variants"]), 4)
        baseline = written["baseline.json"]
        self.assertEqual(
            baseline["background"]["moving_speed"]["policy"],
            "unbounded")
        self.assertEqual(
            set(baseline["sumo_behavior_variant"]["vehicles"]),
            {"nusc_car", "nusc_bike"})
        self.assertEqual(baseline["critical_actors"], [])
        validate_sumo_variant(
            baseline["sumo_behavior_variant"],
            {"nusc_car", "nusc_bike"})

        variant = written["variant_000.json"]
        section = variant["sumo_behavior_variant"]
        factor = section["sampled_profile"]["tau_factor"]
        self.assertAlmostEqual(
            section["vehicles"]["nusc_car"]["car_following"]["tau_s"],
            1.2 * factor, places=5)
        self.assertAlmostEqual(
            section["vehicles"]["nusc_bike"]["car_following"]["tau_s"],
            1.0 * factor, places=5)
        self.assertEqual(
            variant["scenario"]["effective_moving_speed_policy"],
            "unbounded")
        self.assertTrue(os.path.isabs(baseline["manifest"]))
        self.assertTrue(os.path.isabs(baseline["sumo"]["route_report"]))

    def test_rejects_recorded_speed_base_with_hidden_vtype_caps(self):
        base, _report = self._base()
        base["background"]["moving_speed"]["policy"] = "recorded_profile"
        args = self._args("hybrid_config.json", "variants")
        with mock.patch.object(
                generate_sumo_safety_variants, "load_json",
                return_value=base):
            with self.assertRaisesRegex(
                    ValueError, "--moving-speed-policy unbounded"):
                generate_sumo_safety_variants.run(args)

    def test_rejects_json_relabel_when_route_report_still_has_speed_caps(self):
        base, report = self._base()
        report["moving_speed"]["policy"] = "recorded_profile"
        report["included"][0]["sumo_max_speed_mps"] = 12.0
        args = self._args("hybrid_config.json", "variants")
        with mock.patch.object(
                generate_sumo_safety_variants, "load_json",
                side_effect=[base, report]), \
                mock.patch.object(
                    generate_sumo_safety_variants.os.path, "isfile",
                    return_value=True):
            with self.assertRaisesRegex(
                    ValueError, "changing only hybrid_config.json"):
                generate_sumo_safety_variants.run(args)

    def test_rejects_legacy_critical_actor_partition(self):
        base, _report = self._base(critical=True)
        args = self._args("hybrid_config.json", "variants")
        with mock.patch.object(
                generate_sumo_safety_variants, "load_json",
                return_value=base):
            with self.assertRaisesRegex(ValueError, "without --critical-actor"):
                generate_sumo_safety_variants.run(args)

    def test_rejects_route_report_that_still_excludes_carla_authority(self):
        base, report = self._base(legacy_skip=True)
        args = self._args("hybrid_config.json", "variants")
        with mock.patch.object(
                generate_sumo_safety_variants, "load_json",
                side_effect=[base, report]), \
                mock.patch.object(
                    generate_sumo_safety_variants.os.path, "isfile",
                    return_value=True):
            with self.assertRaisesRegex(ValueError, "route report still excludes"):
                generate_sumo_safety_variants.run(args)

    def test_variant_requires_one_to_one_source_track_mapping(self):
        spec = {
            "source_track_id": "car-track",
            "sumo_type_id": "nusc_car",
            **_sumo_baseline(1.2, 2.5, 2.6, 4.5),
        }
        document = {
            "schema_version": 1,
            "pipeline": SUMO_PIPELINE,
            "variant_id": "bad_mapping",
            "scope": {
                "policy": "all_moving_background",
                "sumo_vehicle_ids": ["nusc_a", "nusc_b"],
                "source_track_ids": ["car-track", "bike-track"],
            },
            "vehicles": {
                "nusc_a": copy.deepcopy(spec),
                "nusc_b": copy.deepcopy(spec),
            },
        }
        with self.assertRaisesRegex(ValueError, "one-to-one"):
            validate_sumo_variant(document)


class CarlaSafetyVariantTests(unittest.TestCase):
    @staticmethod
    def _track(actor_id, points):
        return ActorTrack(actor_id, "vehicle.car", [
            TrackPoint(index, float(index), x, 0.0, 0.0)
            for index, x in enumerate(points)
        ])

    def test_generates_all_moving_scope_and_retains_static_population(self):
        moving = self._track("moving", [0.0, 5.0, 10.0])
        static = self._track("static", [0.0, 0.2, -0.1])
        one_point = self._track("one-point", [3.0])
        bundle = types.SimpleNamespace(vehicle_tracks={
            track.actor_id: track for track in (moving, static, one_point)
        })
        manifest = {"scene": "test-scene"}
        written = {}

        def capture(path, value):
            written[os.path.basename(path)] = copy.deepcopy(value)

        with mock.patch.object(
                generate_carla_safety_variants, "load_manifest_tracks",
                return_value=(manifest, bundle)), \
                mock.patch.object(
                    generate_carla_safety_variants.os, "makedirs"), \
                mock.patch.object(
                    generate_carla_safety_variants, "write_json",
                    side_effect=capture):
            args = argparse.Namespace(
                manifest="manifest.json",
                output="variants",
                count=3, seed=7, simulation_seed=103,
                minimum_track_distance=2.0,
                minimum_two_point_speed=1.0,
                desired_speed_scale=(0.75, 1.35),
                leading_distance=(0.5, 4.0),
                random_left_lane_change=(0.0, 30.0),
                random_right_lane_change=(0.0, 30.0),
                keep_right=(0.0, 30.0),
                baseline_leading_distance=2.5,
            )

            index = generate_carla_safety_variants.run(args)

            self.assertEqual(index["pipeline"], CARLA_PIPELINE)
            self.assertEqual(
                index["selection"]["eligible_track_ids"], ["moving"])
            self.assertEqual(
                index["selection"]["retained_vehicle_track_ids"],
                ["moving", "one-point", "static"])
            self.assertEqual(
                index["population"]["recorded_static_count"], 1)
            self.assertEqual(
                index["population"]["single_observation_static_count"], 1)
            baseline = written["baseline.json"]
            self.assertFalse(baseline["behavior"]["auto_lane_change"])
            self.assertEqual(len(index["variants"]), 3)
            variant = validate_carla_variant(
                written["variant_000.json"])
            self.assertTrue(variant["behavior"]["auto_lane_change"])
            self.assertEqual(
                variant["selection"]["eligible_track_ids"], ["moving"])
            self.assertEqual(
                variant["selection"]["retained_vehicle_track_ids"],
                ["moving", "one-point", "static"])

    def test_disallows_lane_change_percentages_when_auto_change_is_off(self):
        document = {
            "schema_version": 1,
            "pipeline": CARLA_PIPELINE,
            "variant_id": "bad",
            "manifest": "manifest.json",
            "simulation_seed": 103,
            "selection": {
                "scope": "all_moving_surrounding",
                "minimum_track_distance_m": 2.0,
                "minimum_two_point_speed_mps": 1.0,
                "eligible_track_ids": ["moving"],
                "retained_vehicle_track_ids": ["moving"],
            },
            "behavior": {
                "desired_speed_scale": 1.0,
                "leading_distance_m": 2.5,
                "auto_lane_change": False,
                "random_left_lane_change_percentage": 10.0,
                "random_right_lane_change_percentage": 0.0,
                "keep_right_rule_percentage": 0.0,
            },
        }
        with self.assertRaisesRegex(ValueError, "must be zero"):
            validate_carla_variant(copy.deepcopy(document))

    def test_rejects_out_of_range_traffic_manager_seed(self):
        document = {
            "schema_version": 1,
            "pipeline": CARLA_PIPELINE,
            "variant_id": "bad_seed",
            "manifest": "manifest.json",
            "simulation_seed": -1,
            "selection": {
                "scope": "all_moving_surrounding",
                "minimum_track_distance_m": 2.0,
                "minimum_two_point_speed_mps": 1.0,
                "eligible_track_ids": ["moving"],
                "retained_vehicle_track_ids": ["moving"],
            },
            "behavior": {
                "desired_speed_scale": 1.0,
                "leading_distance_m": 2.5,
                "auto_lane_change": False,
                "random_left_lane_change_percentage": 0.0,
                "random_right_lane_change_percentage": 0.0,
                "keep_right_rule_percentage": 0.0,
            },
        }
        with self.assertRaisesRegex(ValueError, "inside 0"):
            validate_carla_variant(document)


if __name__ == "__main__":
    unittest.main()
