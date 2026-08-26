import argparse
import copy
import unittest
from unittest import mock

from tools import generate_safety_variants


class SafetyVariantTimingTests(unittest.TestCase):
    def test_ranges_and_events_fit_finite_critical_track(self):
        start_range, duration_range = (
            generate_safety_variants.bounded_brake_ranges(
                5.5, (4.0, 12.0), (0.5, 2.5)))

        self.assertEqual(start_range, (4.0, 5.0))
        self.assertEqual(duration_range, (0.5, 2.5))
        self.assertEqual(
            generate_safety_variants.fit_brake_event(4.75, 2.0, 5.5),
            (4.75, 0.75))

    def test_generated_braking_events_end_before_track_control_ends(self):
        base = {
            "manifest": "unused-by-mock.json",
            "critical_actors": [{
                "track_id": "critical",
                "authority": "carla_reference",
            }],
        }
        args = argparse.Namespace(
            config="hybrid_config.json",
            critical_actor="critical",
            count=20,
            seed=103,
            brake_start=(4.0, 12.0),
            brake_duration=(0.5, 2.5),
            brake_intensity=(0.4, 1.0),
            speed_scale=(0.8, 1.2),
            time_headway=(0.8, 2.0),
            output="variants",
        )
        window = {
            "start_time_s": 0.0,
            "end_time_s": 5.5,
            "controllable_duration_s": 5.5,
        }
        dumped = []

        def capture_dump(value, _stream, **_kwargs):
            dumped.append(copy.deepcopy(value))

        with mock.patch.object(
                generate_safety_variants, "load_json", return_value=base), \
                mock.patch.object(
                    generate_safety_variants, "critical_track_window",
                    return_value=window), \
                mock.patch.object(generate_safety_variants.os, "makedirs"), \
                mock.patch("builtins.open", mock.mock_open()), \
                mock.patch.object(
                    generate_safety_variants.json, "dump",
                    side_effect=capture_dump):
            generate_safety_variants.run(args)

        variants, index = dumped[:-1], dumped[-1]
        self.assertEqual(len(variants), 20)
        self.assertEqual(index["critical_track_window"], window)
        self.assertEqual(index["effective_brake_start_range_s"],
                         [4.0, 5.0])
        for variant, row in zip(variants, index["variants"]):
            self.assertGreaterEqual(row["brake_duration_s"], 0.5)
            self.assertLessEqual(row["brake_end_s"], 5.5 + 1.0e-9)
            event = variant["critical_actors"][0]["events"][0]
            self.assertAlmostEqual(
                event["start"] + event["duration"], row["brake_end_s"])
            self.assertLessEqual(
                event["start"] + event["duration"], 5.5 + 1.0e-9)


if __name__ == "__main__":
    unittest.main()
