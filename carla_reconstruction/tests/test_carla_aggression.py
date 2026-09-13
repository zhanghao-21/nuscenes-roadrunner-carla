"""Stronger simulation-only TM profiles, compatibility and scope isolation."""

import copy
import unittest
from unittest import mock

from closed_loop.safety_variants import (
    CARLA_AGGRESSION_LIMITS, carla_behavior_for_track, validate_carla_variant)
from tests import test_carla_perturbation_scopes as scopes
from tools import generate_carla_safety_variants as generator


class CarlaAggressionTests(unittest.TestCase):
    def sampling(self, *options):
        args = generator.build_parser().parse_args(["--manifest", "manifest.json"] + list(options))
        return generator.resolve_sampling(args)

    def test_default_is_aggressive_and_separate_from_legacy_output(self):
        doc = scopes.generate("surrounding")
        self.assertEqual(doc["index.json"]["aggression_profile"], "aggressive")
        self.assertIn("aggressive", doc["index.json"]["baseline_config"])
        self.assertEqual(doc["variant_000.json"]["schema_version"], 3)
        self.assertEqual(dict(self.sampling()[1])["desired_speed_scale"], (1.4, 2.6))

    def test_strong_profiles_sample_each_driver_within_recorded_bounds(self):
        for profile in ("aggressive", "stress"):
            family = scopes.generate("surrounding", profile=profile)
            ranges = family["index.json"]["parameter_ranges"]
            actors = family["variant_000.json"]["actor_behaviors"]
            self.assertNotEqual(actors["moving"], actors["later"])
            for index in range(3):
                for behavior in family["variant_%03d.json" % index]["actor_behaviors"].values():
                    for key, (lower, upper) in ranges.items():
                        self.assertGreaterEqual(behavior[key], lower)
                        self.assertLessEqual(behavior[key], upper)

    def test_per_track_samples_are_stable_when_another_track_is_added(self):
        profile, bounds, ceiling = self.sampling()
        first = generator.sampled_actor_behaviors(20, bounds, 103, ["moving", "later"], profile, ceiling)
        second = generator.sampled_actor_behaviors(20, bounds, 103, ["new", "later", "moving"], profile, ceiling)
        self.assertEqual([x["moving"] for x in first], [x["moving"] for x in second])
        changed = generator.sampled_actor_behaviors(20, bounds, 104, ["moving"], profile, ceiling)
        self.assertNotEqual([x["moving"] for x in first], [x["moving"] for x in changed])

    def test_per_driver_latin_hypercube_strata_remain_covered(self):
        profile, bounds, ceiling = self.sampling()
        rows = generator.sampled_actor_behaviors(20, bounds, 103, ["ego", "other"], profile, ceiling)
        for actor_id in ("ego", "other"):
            for key, (lower, upper) in bounds:
                if lower == upper:
                    continue
                bins = {min(19, int((r[actor_id][key] - lower) / (upper - lower) * 20)) for r in rows}
                self.assertEqual(bins, set(range(20)))

    def test_stress_explicitly_ignores_vehicle_and_rule_hazards(self):
        doc = scopes.generate("ego", profile="stress")["variant_000.json"]
        behavior = doc["actor_behaviors"]["ego"]
        for key in ("ignore_vehicles_percentage", "ignore_lights_percentage", "ignore_signs_percentage"):
            self.assertEqual(behavior[key], 100)
        self.assertEqual(behavior["target_speed_ceiling_kmh"], 100)

    def test_mild_preserves_previous_ranges_and_homogeneous_behavior(self):
        family = scopes.generate("surrounding", profile="mild")
        self.assertEqual(family["index.json"]["parameter_ranges"]["desired_speed_scale"], (0.75, 1.35))
        behavior = family["variant_000.json"]["actor_behaviors"]
        self.assertEqual(behavior["moving"], behavior["later"])
        self.assertTrue(all(behavior["moving"][key] == 0 for key in CARLA_AGGRESSION_LIMITS))

    def test_baselines_identical_across_profiles_and_scopes(self):
        baseline = scopes.generate("ego", profile="mild")["baseline.json"]["behavior"]
        for target in ("ego", "surrounding"):
            for profile in ("mild", "aggressive", "stress"):
                self.assertEqual(scopes.generate(target, profile=profile)["baseline.json"]["behavior"], baseline)

    def test_explicit_range_overrides_take_precedence(self):
        profile, bounds, cap = self.sampling("--profile", "stress", "--ignore-vehicles", "0,0",
                                            "--target-speed-floor", "20,30", "--maximum-speed-kmh", "40")
        self.assertEqual(profile, "stress")
        self.assertEqual(dict(bounds)["ignore_vehicles_percentage"], (0, 0))
        self.assertEqual(cap, 40)

    def test_bad_ranges_rejected_before_generation(self):
        for options in (("--ignore-vehicles=-1,100",), ("--ignore-lights", "0,101"),
                        ("--target-speed-floor", "80,90", "--maximum-speed-kmh", "70"),
                        ("--maximum-speed-kmh", "nan"), ("--desired-speed-scale", "1,9")):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.sampling(*options)

    def test_old_version_two_has_zero_bypass_and_keeps_both_tm_scopes(self):
        for target in ("ego", "surrounding"):
            doc = scopes.generate(target, profile="mild")["variant_000.json"]
            doc["schema_version"] = 2
            del doc["actor_behaviors"]
            for profile in (doc["behavior"], doc["baseline_behavior"]):
                for key in CARLA_AGGRESSION_LIMITS:
                    del profile[key]
            normalized = validate_carla_variant(doc)
            self.assertEqual(normalized["ego_mode"], "tm")
            for actor_id in ("ego", "moving", "later"):
                resolved = carla_behavior_for_track(normalized, actor_id)
                self.assertTrue(all(resolved[key] == 0 for key in CARLA_AGGRESSION_LIMITS))

    def test_cross_target_actor_profile_and_missing_actor_are_rejected(self):
        doc = scopes.generate("ego")["variant_000.json"]
        for wrong in ({}, {"moving": doc["behavior"]}, {"ego": doc["behavior"], "parked": doc["behavior"]}):
            altered = copy.deepcopy(doc)
            altered["actor_behaviors"] = wrong
            with self.assertRaisesRegex(ValueError, "exactly the selected"):
                validate_carla_variant(altered)

    def test_cannot_silently_make_untargeted_baseline_aggressive(self):
        doc = scopes.generate("ego")["variant_000.json"]
        doc["baseline_behavior"]["ignore_vehicles_percentage"] = 99
        with self.assertRaisesRegex(ValueError, "preserve normal"):
            validate_carla_variant(doc)

    def test_no_unsupported_settings_or_invalid_actor_values(self):
        for key, value in (("ignore_vehicles_percentage", 101), ("ignore_vehicles_percentage", True),
                           ("target_speed_floor_kmh", float("nan")), ("ignore_walkers_percentage", 100)):
            doc = scopes.generate("ego")["variant_000.json"]
            doc["actor_behaviors"]["ego"][key] = value
            with self.assertRaises(ValueError):
                validate_carla_variant(doc)

    def test_profile_output_cannot_overwrite_another_profile(self):
        aggressive = scopes.generate("ego")["baseline.json"]
        stress = scopes.generate("ego", profile="stress")["baseline.json"]
        with mock.patch.object(generator.os.path, "isfile", return_value=True), \
                mock.patch.object(generator, "load_json", return_value=aggressive):
            with self.assertRaisesRegex(ValueError, "different scene, target, profile"):
                generator._validate_output_scope("old", stress)

    def test_low_recorded_speed_uses_variant_floor_without_affecting_baseline(self):
        from carla_reconstruction.closed_loop import carla_runtime
        behavior = scopes.generate("ego")["variant_000.json"]["actor_behaviors"]["ego"]
        track = scopes.track("ego", [0, .1])
        manager, actor = mock.Mock(), mock.Mock()
        with mock.patch.object(carla_runtime, "carla_path", return_value=[]):
            result = carla_runtime.configure_tm_actor(manager, actor, track, 8000, behavior_variant=behavior)
            normal = carla_runtime.configure_tm_actor(mock.Mock(), actor, track, 8000)
        self.assertEqual(result["desired_speed_kmh"], behavior["target_speed_floor_kmh"])
        self.assertEqual(normal["desired_speed_kmh"], 3.0)
        self.assertEqual(normal["ignore_vehicles_percentage"], 0.0)

    def test_high_recorded_speed_is_capped_and_pedestrian_checks_remain_on(self):
        from carla_reconstruction.closed_loop import carla_runtime
        behavior = scopes.generate("ego", profile="stress")["variant_000.json"]["actor_behaviors"]["ego"]
        manager, actor = mock.Mock(), mock.Mock()
        with mock.patch.object(carla_runtime, "carla_path", return_value=[]):
            result = carla_runtime.configure_tm_actor(manager, actor, scopes.track("ego", [0, 100]),
                                                      8000, behavior_variant=behavior)
        self.assertEqual(result["desired_speed_kmh"], 100.0)
        manager.ignore_walkers_percentage.assert_called_once_with(actor, 0.0)
        manager.ignore_vehicles_percentage.assert_called_once_with(actor, 100.0)
        actor.set_simulate_physics.assert_not_called()


if __name__ == "__main__":
    unittest.main()
