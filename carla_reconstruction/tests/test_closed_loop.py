import math
import os
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

from closed_loop.hybrid_authority import (
    actor_spawn_policy, configured_carla_actor_specs)
from closed_loop.metrics import SafetyMetrics, closing_ttc
from closed_loop.sumo_routes import (
    extend_route_to_terminal, load_network, plan_track_route,
    schedule_departure_times, write_sumo_scenario)
from closed_loop.tracks import (
    ActorTrack, TrackPoint, carla_pose, classify_vehicle_motion, point_at,
    recorded_speed_at, tracks_from_meta)


FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


class TrackTests(unittest.TestCase):
    def setUp(self):
        self.bundle = tracks_from_meta({
            "scene": "scene-test",
            "frame_dt": 0.5,
            "ego_trajectory_local": [
                {"x": 0.0, "y": 1.0, "yaw": math.radians(170.0)},
                {"x": 5.0, "y": 1.0, "yaw": math.radians(-170.0)},
            ],
            "agent_frames": [
                [],
                [{"id": "car-1", "category": "vehicle.car", "x": 2.0,
                  "y": 0.0, "yaw": 0.0, "wlh": [2.0, 4.0, 1.5]}],
            ],
        })

    def test_tracks_preserve_actor_start_time(self):
        self.assertEqual(self.bundle.actors["car-1"].start_time, 0.5)
        self.assertTrue(self.bundle.actors["car-1"].is_vehicle)

    def test_pose_interpolation_wraps_yaw_and_converts_frame(self):
        point = point_at(self.bundle.ego, 0.25)
        self.assertAlmostEqual(abs(math.degrees(point.yaw)), 180.0)
        x, y, yaw = carla_pose(point)
        self.assertAlmostEqual(x, 2.5)
        self.assertAlmostEqual(y, -1.0)
        self.assertAlmostEqual(abs(yaw), 180.0)

    def test_vehicle_motion_classifier_uses_spatial_extent_boundary(self):
        def track(actor_id, coordinates, category="vehicle.car"):
            return ActorTrack(actor_id, category, [
                TrackPoint(index, float(index), x, y, 0.0)
                for index, (x, y) in enumerate(coordinates)
            ])

        jitter = track(
            "annotation-jitter",
            [(0.0, 0.0), (1.1, 0.0), (0.0, 0.0), (1.1, 0.0)])
        self.assertGreater(jitter.distance, 2.0)
        self.assertLess(jitter.motion_extent, 2.0)
        self.assertEqual(
            classify_vehicle_motion(jitter),
            "static")
        self.assertEqual(
            classify_vehicle_motion(
                track("below-boundary", [(0.0, 0.0), (1.999, 0.0)])),
            "static")
        self.assertEqual(
            classify_vehicle_motion(
                track("boundary", [(0.0, 0.0), (2.0, 0.0)])),
            "moving")
        self.assertIsNone(
            classify_vehicle_motion(track("one-point", [(0.0, 0.0)])))
        self.assertIsNone(classify_vehicle_motion(
            track("walker", [(0.0, 0.0), (3.0, 0.0)],
                  category="human.pedestrian.adult")))
        with self.assertRaisesRegex(ValueError, "non-negative"):
            classify_vehicle_motion(
                track("invalid", [(0.0, 0.0), (3.0, 0.0)]), -1.0)

    def test_vehicle_motion_classifier_recognizes_short_fast_track(self):
        short_fast = ActorTrack("short-fast", "vehicle.car", [
            TrackPoint(0, 19.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 19.5, 0.75, 0.0, 0.0),
        ])

        self.assertLess(short_fast.motion_extent, 2.0)
        self.assertGreater(short_fast.mean_speed, 1.0)
        self.assertEqual(classify_vehicle_motion(short_fast), "moving")

    def test_vehicle_motion_classifier_validates_short_speed(self):
        track = ActorTrack("short", "vehicle.car", [
            TrackPoint(0, 0.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 0.5, 0.75, 0.0, 0.0),
        ])

        for invalid in (-0.1, float("nan"), float("inf"), True, "fast"):
            with self.subTest(invalid=invalid), \
                    self.assertRaisesRegex(ValueError, "non-negative"):
                classify_vehicle_motion(
                    track, minimum_two_point_speed=invalid)

    def test_recorded_speed_at_uses_active_segment(self):
        track = ActorTrack("speed-profile", "vehicle.car", [
            TrackPoint(0, 1.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 2.0, 3.0, 4.0, 0.0),
            TrackPoint(2, 4.0, 3.0, 10.0, 0.0),
        ])

        self.assertAlmostEqual(recorded_speed_at(track, 1.0), 5.0)
        self.assertAlmostEqual(recorded_speed_at(track, 1.75), 5.0)
        self.assertAlmostEqual(recorded_speed_at(track, 2.0), 3.0)
        self.assertAlmostEqual(recorded_speed_at(track, 3.5), 3.0)

    def test_recorded_speed_at_requires_an_active_segment(self):
        track = ActorTrack("speed-profile", "vehicle.car", [
            TrackPoint(0, 1.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 2.0, 3.0, 4.0, 0.0),
        ])
        one_point = ActorTrack("one-point", "vehicle.car", [
            TrackPoint(0, 1.0, 0.0, 0.0, 0.0),
        ])

        self.assertIsNone(recorded_speed_at(track, 0.999))
        self.assertIsNone(recorded_speed_at(track, 2.0))
        self.assertIsNone(recorded_speed_at(track, 2.001))
        self.assertIsNone(recorded_speed_at(one_point, 1.0))


class SumoRouteTests(unittest.TestCase):
    def test_track_is_matched_and_connected_across_edges(self):
        network_path = os.path.join(FIXTURES, "sumo.net.xml")
        network = load_network(network_path)
        bundle = tracks_from_meta({
            "scene": "scene-route",
            "frame_dt": 1.0,
            "ego_trajectory_local": [
                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                {"x": 1.0, "y": 0.0, "yaw": 0.0},
            ],
            "agent_frames": [
                [{"id": "car", "category": "vehicle.car", "x": 2.0,
                  "y": 0.0, "yaw": 0.0}],
                [{"id": "car", "category": "vehicle.car", "x": 15.0,
                  "y": 0.0, "yaw": 0.0}],
            ],
        })
        track = bundle.actors["car"]
        plan, error = plan_track_route(network, track, maximum_snap_distance=2.0)
        self.assertIsNone(error)
        self.assertEqual(plan["edges"], ["a", "b"])
        self.assertAlmostEqual(plan["depart_pos"], 2.0)

    def test_track_departure_uses_first_point_close_enough_to_lane(self):
        network = load_network(os.path.join(FIXTURES, "sumo.net.xml"))
        track = ActorTrack("car", "vehicle.car", [
            TrackPoint(0, 0.0, -5.0, 5.0, 0.0),
            TrackPoint(1, 1.0, 2.0, 0.0, 0.0),
            TrackPoint(2, 2.0, 15.0, 0.0, 0.0),
        ])

        plan, error = plan_track_route(
            network, track, maximum_snap_distance=2.0)

        self.assertIsNone(error)
        self.assertEqual(plan["first_matched_frame"], 1)
        self.assertEqual(plan["first_matched_time"], 1.0)
        self.assertAlmostEqual(plan["depart_pos"], 2.0)
        self.assertEqual(plan["edges"], ["a", "b"])

    def test_route_match_rejects_opposite_lane_heading(self):
        network = load_network(os.path.join(FIXTURES, "sumo.net.xml"))
        track = ActorTrack("westbound", "vehicle.car", [
            TrackPoint(0, 0.0, 8.0, 0.0, math.pi),
            TrackPoint(1, 1.0, 2.0, 0.0, math.pi),
        ])

        plan, error = plan_track_route(
            network, track, maximum_snap_distance=0.1)

        self.assertIsNone(plan)
        self.assertIn("no direction- and vehicle-class-compatible", error)

    def test_route_match_rejects_backward_progress_on_same_edge(self):
        network = load_network(os.path.join(FIXTURES, "sumo.net.xml"))
        track = ActorTrack("backward", "vehicle.car", [
            TrackPoint(0, 0.0, 8.0, 0.0, 0.0),
            TrackPoint(1, 1.0, 2.0, 0.0, 0.0),
        ])

        plan, error = plan_track_route(
            network, track, maximum_snap_distance=0.1)

        self.assertIsNone(plan)
        self.assertIn("fewer than 2 direction-consistent", error)

    def test_departure_schedule_preserves_batches_and_caps_only_large_gaps(self):
        schedule = schedule_departure_times(
            [10.0, 0.5, 4.5, 0.5], start_delay=0.2, maximum_gap=1.0)

        self.assertEqual(schedule, {0.5: 0.7, 4.5: 1.7, 10.0: 2.7})
        self.assertEqual(
            schedule_departure_times(
                [10.0, 0.5, 4.5], start_delay=2.0, maximum_gap=None),
            {0.5: 2.5, 4.5: 6.5, 10.0: 12.0})

        with self.assertRaisesRegex(ValueError, "non-negative"):
            schedule_departure_times([0.0], start_delay=-0.1)
        with self.assertRaisesRegex(ValueError, "positive"):
            schedule_departure_times([0.0], maximum_gap=0.0)
        with self.assertRaisesRegex(ValueError, "non-negative"):
            schedule_departure_times([float("nan")])

    def test_writer_outputs_chronological_batches_and_schedule_options(self):
        network_path = os.path.join(FIXTURES, "sumo.net.xml")

        def moving_track(actor_id, matched_time, matched_x):
            return ActorTrack(actor_id, "vehicle.car", [
                TrackPoint(0, 0.0, -5.0, 5.0, 0.0),
                TrackPoint(1, matched_time, matched_x, 0.0, 0.0),
                TrackPoint(2, matched_time + 1.0, 15.0, 0.0, 0.0),
            ])

        tracks = {
            "late-z": moving_track("late-z", 5.0, 2.0),
            "batch-b": moving_track("batch-b", 1.0, 3.0),
            "middle": moving_track("middle", 3.0, 4.0),
            "batch-a": moving_track("batch-a", 1.0, 5.0),
        }
        network = load_network(network_path)
        written_roots = []

        def capture_tree(tree, *args, **kwargs):
            written_roots.append(tree.getroot())

        with mock.patch("closed_loop.sumo_routes.load_network",
                        return_value=network), \
                mock.patch("closed_loop.sumo_routes.os.makedirs"), \
                mock.patch.object(ET.ElementTree, "write",
                                  autospec=True, side_effect=capture_tree), \
                mock.patch("builtins.open", mock.mock_open()):
            _, _, report = write_sumo_scenario(
                network_path, tracks, "unused-output", "scene-schedule",
                maximum_snap_distance=2.0,
                moving_start_delay=0.25,
                maximum_departure_gap=1.0,
                eager_insert=True,
                end_time=10.0)

        route_root, config_root = written_roots
        vehicles = route_root.findall("vehicle")
        self.assertEqual(
            [vehicle.get("id") for vehicle in vehicles],
            ["nusc_batch-a", "nusc_batch-b", "nusc_middle", "nusc_late-z"])
        self.assertEqual(
            [vehicle.get("depart") for vehicle in vehicles],
            ["1.250", "1.250", "2.250", "3.250"])
        self.assertEqual(
            [item["first_matched_time"] for item in report["included"]],
            [1.0, 1.0, 3.0, 5.0])
        self.assertEqual(
            report["moving_departure_schedule"], {
                "start_delay_s": 0.25,
                "maximum_departure_gap_s": 1.0,
                "eager_insert": True,
            })
        self.assertEqual(
            config_root.find("./processing/eager-insert").get("value"),
            "true")
        self.assertEqual(config_root.find("./time/end").get("value"),
                         "10.250")

    def test_writer_keeps_static_vehicles_out_of_sumo_routes(self):
        network_path = os.path.join(FIXTURES, "sumo.net.xml")
        bundle = tracks_from_meta({
            "scene": "scene-authority",
            "frame_dt": 1.0,
            "ego_trajectory_local": [
                {"x": 0.0, "y": 0.0, "yaw": 0.0},
                {"x": 1.0, "y": 0.0, "yaw": 0.0},
            ],
            "agent_frames": [
                [
                    {"id": "moving", "category": "vehicle.car",
                     "x": 2.0, "y": 0.0, "yaw": 0.0},
                    {"id": "parked", "category": "vehicle.car",
                     "x": 2.0, "y": 0.0, "yaw": 0.0},
                    {"id": "critical", "category": "vehicle.car",
                     "x": 2.0, "y": 0.0, "yaw": 0.0},
                    {"id": "one-point", "category": "vehicle.car",
                     "x": 2.0, "y": 0.0, "yaw": 0.0},
                    {"id": "walker", "category": "human.pedestrian.adult",
                     "x": 2.0, "y": 0.0, "yaw": 0.0},
                ],
                [
                    {"id": "moving", "category": "vehicle.car",
                     "x": 15.0, "y": 0.0, "yaw": 0.0},
                    {"id": "parked", "category": "vehicle.car",
                     "x": 2.5, "y": 0.0, "yaw": 0.0},
                    {"id": "critical", "category": "vehicle.car",
                     "x": 15.0, "y": 0.0, "yaw": 0.0},
                ],
            ],
        })

        network = load_network(network_path)
        written_roots = []

        def capture_tree(tree, *args, **kwargs):
            written_roots.append(tree.getroot())

        with mock.patch("closed_loop.sumo_routes.load_network",
                        return_value=network), \
                mock.patch("closed_loop.sumo_routes.os.makedirs"), \
                mock.patch.object(ET.ElementTree, "write",
                                  autospec=True, side_effect=capture_tree), \
                mock.patch("builtins.open", mock.mock_open()):
            _, _, report = write_sumo_scenario(
                network_path, bundle.actors, "unused-output", bundle.scene,
                maximum_snap_distance=2.0,
                excluded_actor_ids={"critical"},
                minimum_track_distance=2.0)

        root = written_roots[0]

        self.assertEqual(
            [vehicle.get("id") for vehicle in root.findall("vehicle")],
            ["nusc_moving"])
        self.assertEqual(
            [item["id"] for item in report["included"]], ["moving"])
        self.assertEqual(
            [item["id"] for item in report["carla_static"]],
            ["parked", "one-point"])
        self.assertEqual(report["carla_static"][0]["authority"], "carla_static")
        self.assertEqual(
            report["carla_static"][1]["motion_classification"],
            "single_observation_static")
        self.assertIn("sumo_behavior_baseline", report["included"][0])
        self.assertEqual(
            report["included"][0]["sumo_behavior_baseline"]
            ["lane_changing"]["lc_assertive"], 1.0)
        self.assertEqual(report["minimum_track_distance_m"], 2.0)
        skipped = {item["id"]: item["reason"] for item in report["skipped"]}
        self.assertEqual(skipped["critical"], "CARLA authority")
        self.assertNotIn("one-point", skipped)
        self.assertEqual(skipped["walker"], "not a vehicle")


class SumoRouteContinuationTests(unittest.TestCase):
    @staticmethod
    def _load_inline_network(contents):
        parsed = ET.ElementTree(ET.fromstring(contents))
        with mock.patch("closed_loop.sumo_routes.ET.parse",
                        return_value=parsed):
            return load_network("inline.net.xml")

    def _continuation_network(self):
        return self._load_inline_network("""<?xml version="1.0" encoding="UTF-8"?>
<net version="1.19">
  <location netOffset="0,0" convBoundary="0,0,100,100"
            origBoundary="0,0,100,100" projParameter="!"/>
  <edge id="prefix"><lane id="prefix_0" index="0" speed="13.9" length="10"
      shape="10,50 20,50"/></edge>
  <edge id="left"><lane id="left_0" index="0" speed="13.9" length="22"
      shape="20,50 30,70"/></edge>
  <edge id="straight"><lane id="straight_0" index="0" speed="13.9" length="75"
      shape="20,50 95,50"/></edge>
  <edge id="right"><lane id="right_0" index="0" speed="13.9" length="57"
      shape="20,50 50,95"/></edge>
  <edge id="bus_only"><lane id="bus_only_0" index="0" speed="13.9" length="75"
      allow="bus" shape="20,50 95,60"/></edge>
  <edge id="turnaround"><lane id="turnaround_0" index="0" speed="13.9" length="15"
      shape="20,50 5,50"/></edge>

  <edge id="u_start"><lane id="u_start_0" index="0" speed="13.9" length="10"
      shape="20,20 30,20"/></edge>
  <edge id="u_sink"><lane id="u_sink_0" index="0" speed="13.9" length="25"
      shape="30,20 5,20"/></edge>

  <edge id="class_start"><lane id="class_start_0" index="0" speed="13.9" length="10"
      shape="20,30 30,30"/></edge>
  <edge id="class_bus_sink"><lane id="class_bus_sink_0" index="0" speed="13.9" length="65"
      allow="bus" shape="30,30 95,30"/></edge>

  <edge id="interior_path_start"><lane id="interior_path_start_0" index="0"
      speed="13.9" length="10" shape="20,80 30,80"/></edge>
  <edge id="interior_path_sink"><lane id="interior_path_sink_0" index="0"
      speed="13.9" length="20" shape="30,80 50,80"/></edge>

  <edge id="mixed_start"><lane id="mixed_start_0" index="0" speed="13.9"
      length="10" shape="20,60 30,60"/></edge>
  <edge id="mixed_interior_sink"><lane id="mixed_interior_sink_0" index="0"
      speed="13.9" length="20" shape="30,60 50,60"/></edge>
  <edge id="mixed_cycle_a"><lane id="mixed_cycle_a_0" index="0" speed="13.9"
      length="10" shape="30,65 40,65"/></edge>
  <edge id="mixed_cycle_b"><lane id="mixed_cycle_b_0" index="0" speed="13.9"
      length="10" shape="40,65 30,65"/></edge>

  <edge id="cycle_a"><lane id="cycle_a_0" index="0" speed="13.9" length="10"
      shape="20,10 30,10"/></edge>
  <edge id="cycle_b"><lane id="cycle_b_0" index="0" speed="13.9" length="10"
      shape="30,10 40,10"/></edge>
  <edge id="cycle_exit"><lane id="cycle_exit_0" index="0" speed="13.9" length="55"
      shape="40,10 95,10"/></edge>

  <edge id="loop_a"><lane id="loop_a_0" index="0" speed="13.9" length="10"
      shape="40,40 50,40"/></edge>
  <edge id="loop_b"><lane id="loop_b_0" index="0" speed="13.9" length="10"
      shape="50,40 40,40"/></edge>

  <connection from="prefix" to="left" fromLane="0" toLane="0" dir="l"/>
  <connection from="prefix" to="straight" fromLane="0" toLane="0" dir="s"/>
  <connection from="prefix" to="right" fromLane="0" toLane="0" dir="r"/>
  <connection from="prefix" to="bus_only" fromLane="0" toLane="0" dir="r"/>
  <connection from="prefix" to="turnaround" fromLane="0" toLane="0" dir="t"/>
  <connection from="u_start" to="u_sink" fromLane="0" toLane="0" dir="t"/>
  <connection from="class_start" to="class_bus_sink" fromLane="0" toLane="0" dir="s"/>
  <connection from="interior_path_start" to="interior_path_sink"
      fromLane="0" toLane="0" dir="s"/>
  <connection from="mixed_start" to="mixed_interior_sink"
      fromLane="0" toLane="0" dir="s"/>
  <connection from="mixed_start" to="mixed_cycle_a"
      fromLane="0" toLane="0" dir="s"/>
  <connection from="mixed_cycle_a" to="mixed_cycle_b"
      fromLane="0" toLane="0" dir="s"/>
  <connection from="mixed_cycle_b" to="mixed_cycle_a"
      fromLane="0" toLane="0" dir="s"/>
  <connection from="cycle_a" to="cycle_b" fromLane="0" toLane="0" dir="s"/>
  <connection from="cycle_b" to="cycle_a" fromLane="0" toLane="0" dir="t"/>
  <connection from="cycle_b" to="cycle_exit" fromLane="0" toLane="0" dir="s"/>
  <connection from="loop_a" to="loop_b" fromLane="0" toLane="0" dir="s"/>
  <connection from="loop_b" to="loop_a" fromLane="0" toLane="0" dir="s"/>
</net>
""")

    def test_terminal_extension_is_deterministic_and_preserves_prefix(self):
        network = self._continuation_network()
        recorded = ["prefix"]

        first, first_report = extend_route_to_terminal(
            network, recorded, "car", "actor-1", seed=103)
        second, second_report = extend_route_to_terminal(
            network, recorded, "car", "actor-1", seed=103)

        self.assertEqual(first, second)
        self.assertEqual(first_report, second_report)
        self.assertEqual(recorded, ["prefix"])
        self.assertEqual((recorded + first)[:len(recorded)], recorded)
        self.assertIn(first, (["straight"], ["right"]))
        self.assertEqual(first_report["status"],
                         "extended_to_boundary_terminal")
        self.assertNotIn("left", first)
        self.assertNotIn("bus_only", first)
        self.assertNotIn("turnaround", first)

        unchanged, terminal_report = extend_route_to_terminal(
            network, ["straight"], "car", "actor-1", seed=103,
            minimum_cycle_distance_m=1000.0)
        self.assertEqual(unchanged, [])
        self.assertEqual(terminal_report["status"], "boundary_terminal")
        self.assertEqual(terminal_report["terminal_edge"], "straight")

    def test_extension_rejects_class_restricted_interior_sink(self):
        network = self._continuation_network()

        with self.assertRaisesRegex(
                RuntimeError, "no valid boundary-or-cycle continuation"):
            extend_route_to_terminal(
                network, ["class_start"], "car", "passenger", seed=7)
        bus_tail, _ = extend_route_to_terminal(
            network, ["class_start"], "bus", "bus", seed=7)
        self.assertEqual(bus_tail, ["class_bus_sink"])

    def test_extension_rejects_acyclic_path_to_interior_sink(self):
        network = self._continuation_network()

        with self.assertRaisesRegex(
                RuntimeError, "no valid boundary-or-cycle continuation"):
            extend_route_to_terminal(
                network, ["interior_path_start"], "car",
                "interior-path", seed=7)

    def test_extension_prefers_closed_cycle_over_interior_sink(self):
        network = self._continuation_network()

        first_tail, first_report = extend_route_to_terminal(
            network, ["mixed_start"], "car", "mixed", seed=0,
            maximum_edges=4, minimum_cycle_distance_m=60.0)
        repeated_tail, repeated_report = extend_route_to_terminal(
            network, ["mixed_start"], "car", "mixed", seed=0,
            maximum_edges=4, minimum_cycle_distance_m=60.0)

        self.assertEqual(first_tail, repeated_tail)
        self.assertEqual(first_report, repeated_report)
        self.assertNotIn("mixed_interior_sink", first_tail)
        self.assertEqual(first_tail[0], "mixed_cycle_a")
        self.assertGreaterEqual(first_tail.count("mixed_cycle_a"), 2)
        self.assertGreaterEqual(first_tail.count("mixed_cycle_b"), 2)
        self.assertEqual(first_report["status"], "extended_bounded_cycle")
        self.assertTrue(first_report["horizon_covered"])
        self.assertGreaterEqual(
            first_report["planned_cycle_distance_m"], 60.0)

    def test_extension_filters_uturns_and_preserves_cycles(self):
        network = self._continuation_network()

        with self.assertRaisesRegex(
                RuntimeError, "no valid boundary-or-cycle continuation"):
            extend_route_to_terminal(
                network, ["u_start"], "car", "car", seed=7)
        with_uturn, _ = extend_route_to_terminal(
            network, ["u_start"], "car", "car", seed=7,
            allow_uturns=True)
        self.assertEqual(with_uturn, ["u_sink"])

        cycle_tail, cycle_report = extend_route_to_terminal(
            network, ["cycle_a"], "car", "cycle", seed=7)
        self.assertEqual(cycle_tail, ["cycle_b", "cycle_exit"])
        self.assertEqual(cycle_report["terminal_edge"], "cycle_exit")
        loop_tail, loop_report = extend_route_to_terminal(
            network, ["loop_a"], "car", "loop", seed=7,
            maximum_edges=5)
        repeated_tail, repeated_report = extend_route_to_terminal(
            network, ["loop_a"], "car", "loop", seed=7,
            maximum_edges=5)
        self.assertEqual(loop_tail,
                         ["loop_b", "loop_a", "loop_b", "loop_a", "loop_b"])
        self.assertEqual(loop_tail, repeated_tail)
        self.assertEqual(loop_report, repeated_report)
        self.assertEqual(loop_report["status"], "extended_bounded_cycle")
        self.assertEqual(loop_report["terminal_edge"], "loop_b")
        self.assertEqual(loop_report["directions"], ["s"] * 5)

    def test_cycle_extension_exceeds_legacy_limit_to_cover_distance(self):
        network = self._continuation_network()

        tail, report = extend_route_to_terminal(
            network, ["loop_a"], "car", "long-running-loop", seed=7,
            maximum_edges=64, minimum_cycle_distance_m=650.0)
        repeated_tail, repeated_report = extend_route_to_terminal(
            network, ["loop_a"], "car", "long-running-loop", seed=7,
            maximum_edges=64, minimum_cycle_distance_m=650.0)

        self.assertEqual(len(tail), 65)
        self.assertGreater(len(tail), 64)
        self.assertEqual(tail, repeated_tail)
        self.assertEqual(report, repeated_report)
        self.assertEqual(report["status"], "extended_bounded_cycle")
        self.assertAlmostEqual(report["minimum_cycle_distance_m"], 650.0)
        self.assertGreaterEqual(report["planned_cycle_distance_m"], 650.0)
        self.assertTrue(report["horizon_covered"])

    def test_cycle_fallback_preserves_a_terminal_beyond_search_depth(self):
        network = self._continuation_network()

        tail, report = extend_route_to_terminal(
            network, ["cycle_a"], "car", "late-terminal", seed=7,
            maximum_edges=1, minimum_cycle_distance_m=1000.0)

        self.assertEqual(tail, ["cycle_b", "cycle_exit"])
        self.assertEqual(report["status"],
                         "extended_to_boundary_terminal")
        self.assertEqual(report["terminal_edge"], "cycle_exit")

    def test_cycle_construction_safety_limit_fails_explicitly(self):
        network = self._continuation_network()

        with mock.patch(
                "closed_loop.sumo_routes.MAXIMUM_CYCLE_CONSTRUCTION_EDGES",
                5), self.assertRaisesRegex(
                    RuntimeError, "safety limit was exhausted"):
            extend_route_to_terminal(
                network, ["loop_a"], "car", "guarded-loop", seed=7,
                maximum_edges=4, minimum_cycle_distance_m=100.0)

    def test_writer_sizes_cycle_for_remaining_simulation_horizon(self):
        network = self._continuation_network()
        track = ActorTrack("looping-car", "vehicle.car", [
            TrackPoint(0, 0.0, 42.0, 40.0, 0.0),
            TrackPoint(1, 1.0, 48.0, 40.0, 0.0),
        ])
        written_roots = []

        def capture_tree(tree, *args, **kwargs):
            written_roots.append(tree.getroot())

        with mock.patch("closed_loop.sumo_routes.load_network",
                        return_value=network), \
                mock.patch("closed_loop.sumo_routes.os.makedirs"), \
                mock.patch.object(ET.ElementTree, "write", autospec=True,
                                  side_effect=capture_tree), \
                mock.patch("builtins.open", mock.mock_open()):
            _, _, report = write_sumo_scenario(
                "inline.net.xml", {track.actor_id: track}, "unused-output",
                "long-loop", maximum_snap_distance=0.5, end_time=20.0,
                moving_route_continuation="terminal",
                maximum_route_continuation_edges=4,
                moving_speed_policy="recorded_profile")

        included = report["included"][0]
        continuation = included["continuation"]
        self.assertTrue(report["moving_route_continuation"][
            "runtime_extend_at_route_end"])
        self.assertEqual(
            report["moving_route_continuation"][
                "runtime_selection_policy"],
            "preserve_prepared_route_then_guarded_persistent_stop_recovery")
        self.assertEqual(
            report["moving_route_continuation"][
                "tail_extension_selection_policy"],
            "seeded_random_viable_outgoing_before_route_end")
        self.assertEqual(
            report["moving_route_continuation"][
                "persistent_stop_recovery"], {
                    "minimum_duration_s": 1.0,
                    "stop_speed_mps": 0.1,
                    "resume_speed_mps": 0.3,
                    "progress_reset_distance_m": 0.5,
                    "close_leader_distance_m": 15.0,
                    "traffic_signal_guard_distance_m": 15.0,
                    "downstream_blocker_minimum_duration_s": 1.0,
                    "maximum_suffix_recovery_attempts": 3,
                })
        self.assertGreater(len(included["continuation_edges"]), 4)
        self.assertAlmostEqual(continuation["simulation_horizon_s"], 20.0)
        self.assertAlmostEqual(continuation["speed_bound_mps"], 6.0)
        self.assertGreaterEqual(
            continuation["planned_cycle_distance_m"],
            continuation["minimum_cycle_distance_m"])
        self.assertTrue(continuation["horizon_covered"])
        route_edges = written_roots[0].find("route").get("edges").split()
        self.assertEqual(route_edges, included["edges"])
        self.assertEqual(
            written_roots[1].find("./time/end").get("value"), "20.000")

    def test_internal_via_observation_restores_normal_edge_transition(self):
        network = self._load_inline_network("""<?xml version="1.0" encoding="UTF-8"?>
<net version="1.19">
  <location netOffset="0,0" convBoundary="0,0,20,10"
            origBoundary="0,0,20,10" projParameter="!"/>
  <edge id="from"><lane id="from_0" index="0" speed="13.9" length="10"
      shape="0,0 10,0"/></edge>
  <edge id=":junction" function="internal"><lane id=":junction_0" index="0"
      speed="8" length="10" shape="10,0 10,10"/></edge>
  <edge id="to"><lane id="to_0" index="0" speed="13.9" length="10"
      shape="10,10 20,10"/></edge>
  <connection from="from" to="to" fromLane="0" toLane="0"
      via=":junction_0" dir="l"/>
</net>
""")
        track = ActorTrack("turning", "vehicle.car", [
            TrackPoint(0, 0.0, 10.0, 3.0, math.pi / 2.0),
            TrackPoint(1, 1.0, 10.0, 7.0, math.pi / 2.0),
        ])

        plan, error = plan_track_route(
            network, track, maximum_snap_distance=0.5)

        self.assertIsNone(error)
        self.assertEqual(plan["recorded_edges"], ["from", "to"])
        self.assertEqual(plan["first_match_kind"], "internal_connection")
        self.assertEqual(plan["internal_transitions"], [{
            "from": "from",
            "to": "to",
            "direction": "l",
            "internal_lane": ":junction_0",
            "first_evidence_frame": 0,
            "first_evidence_time": 0.0,
        }])
        self.assertAlmostEqual(plan["depart_pos"], 9.95)

    def test_chained_internal_via_matches_late_lane_and_full_permissions(self):
        def chained_network(second_lane_permissions=""):
            return self._load_inline_network(("""<?xml version="1.0" encoding="UTF-8"?>
<net version="1.19">
  <location netOffset="0,0" convBoundary="0,0,24,2"
            origBoundary="0,0,24,2" projParameter="!"/>
  <edge id="from"><lane id="from_0" index="0" speed="13.9" length="10"
      shape="0,0 10,0"/></edge>
  <edge id=":turn_0" function="internal"><lane id=":turn_0_0" index="0"
      speed="8" length="2.8" shape="10,0 12,2"/></edge>
  <edge id=":turn_1" function="internal"><lane id=":turn_1_0" index="0"
      speed="8" length="2.8" %s shape="12,2 14,0"/></edge>
  <edge id="to"><lane id="to_0" index="0" speed="13.9" length="10"
      shape="14,0 24,0"/></edge>
  <connection from="from" to="to" fromLane="0" toLane="0"
      via=":turn_0_0" dir="r"/>
  <connection from=":turn_0" to="to" fromLane="0" toLane="0"
      via=":turn_1_0" dir="s"/>
  <connection from=":turn_1" to="to" fromLane="0" toLane="0" dir="s"/>
</net>
""") % second_lane_permissions)

        network = chained_network()
        late_lane_track = ActorTrack("late-lane", "vehicle.car", [
            TrackPoint(0, 0.0, 12.5, 1.5, -math.pi / 4.0),
            TrackPoint(1, 1.0, 13.5, 0.5, -math.pi / 4.0),
        ])

        plan, error = plan_track_route(
            network, late_lane_track, maximum_snap_distance=0.05)

        self.assertIsNone(error)
        self.assertEqual(plan["recorded_edges"], ["from", "to"])
        self.assertEqual(plan["first_match_kind"], "internal_connection")
        self.assertEqual(
            plan["internal_transitions"][0]["internal_lane"],
            ":turn_1_0")

        restricted = chained_network('allow="bus"')
        first_lane_points = [
            TrackPoint(0, 0.0, 10.5, 0.5, math.pi / 4.0),
            TrackPoint(1, 1.0, 11.5, 1.5, math.pi / 4.0),
        ]
        passenger_plan, passenger_error = plan_track_route(
            restricted,
            ActorTrack("passenger", "vehicle.car", first_lane_points),
            maximum_snap_distance=0.05)
        bus_plan, bus_error = plan_track_route(
            restricted,
            ActorTrack("bus", "vehicle.bus", first_lane_points),
            maximum_snap_distance=0.05)

        self.assertIsNone(passenger_plan)
        self.assertIn(
            "no direction- and vehicle-class-compatible", passenger_error)
        self.assertIsNone(bus_error)
        self.assertEqual(bus_plan["recorded_edges"], ["from", "to"])
        self.assertEqual(
            bus_plan["internal_transitions"][0]["internal_lane"],
            ":turn_0_0")

    def test_recorded_uturn_connection_is_retained(self):
        network = self._load_inline_network("""<?xml version="1.0" encoding="UTF-8"?>
<net version="1.19">
  <location netOffset="0,0" convBoundary="0,0,10,2"
            origBoundary="0,0,10,2" projParameter="!"/>
  <edge id="approach"><lane id="approach_0" index="0" speed="13.9"
      length="10" shape="0,0 10,0"/></edge>
  <edge id="return"><lane id="return_0" index="0" speed="13.9"
      length="10" shape="10,1 0,1"/></edge>
  <connection from="approach" to="return" fromLane="0" toLane="0"
      dir="t"/>
</net>
""")
        track = ActorTrack("recorded-uturn", "vehicle.car", [
            TrackPoint(0, 0.0, 8.0, 0.0, 0.0),
            TrackPoint(1, 1.0, 8.0, 1.0, math.pi),
        ])

        plan, error = plan_track_route(
            network, track, maximum_snap_distance=0.1)

        self.assertIsNone(error)
        self.assertEqual(plan["recorded_edges"], ["approach", "return"])

    def test_recorded_mean_vtypes_and_sumo_seed_are_written(self):
        network_path = os.path.join(FIXTURES, "sumo.net.xml")
        network = load_network(network_path)
        tracks = {
            "slow-car": ActorTrack("slow-car", "vehicle.car", [
                TrackPoint(0, 0.0, 2.0, 0.0, 0.0),
                TrackPoint(1, 2.0, 8.0, 0.0, 0.0),
            ]),
            "fast-car": ActorTrack("fast-car", "vehicle.car", [
                TrackPoint(0, 0.0, 2.0, 0.0, 0.0),
                TrackPoint(1, 2.0, 14.0, 0.0, 0.0),
            ]),
        }
        written_roots = []

        def capture_tree(tree, *args, **kwargs):
            written_roots.append(tree.getroot())

        with mock.patch("closed_loop.sumo_routes.load_network",
                        return_value=network), \
                mock.patch("closed_loop.sumo_routes.os.makedirs"), \
                mock.patch.object(ET.ElementTree, "write", autospec=True,
                                  side_effect=capture_tree), \
                mock.patch("builtins.open", mock.mock_open()):
            _, _, report = write_sumo_scenario(
                network_path, tracks, "unused-output", "scene-speeds",
                maximum_snap_distance=2.0,
                moving_speed_policy="recorded_mean",
                sumo_seed=103,
                end_time=5.0)

        route_root, config_root = written_roots
        actor_types = {
            element.get("id"): element
            for element in route_root.findall("vType")
            if element.get("id") in {
                "nusc_car_slow-car", "nusc_car_fast-car"}
        }
        self.assertEqual(actor_types["nusc_car_slow-car"].get("maxSpeed"),
                         "3.000")
        self.assertEqual(actor_types["nusc_car_fast-car"].get("maxSpeed"),
                         "6.000")
        self.assertEqual(actor_types["nusc_car_slow-car"].get("speedFactor"),
                         "1.0")
        self.assertEqual(actor_types["nusc_car_slow-car"].get("speedDev"),
                         "0.0")
        vehicle_types = {
            element.get("id"): element.get("type")
            for element in route_root.findall("vehicle")
        }
        self.assertEqual(vehicle_types, {
            "nusc_fast-car": "nusc_car_fast-car",
            "nusc_slow-car": "nusc_car_slow-car",
        })
        self.assertEqual(config_root.find("./random_number/seed").get("value"),
                         "103")
        self.assertEqual(
            {item["id"]: item["sumo_max_speed_mps"]
             for item in report["included"]},
            {"fast-car": 6.0, "slow-car": 3.0})

    def test_recorded_profile_uses_peak_speed_and_reports_initial_pose(self):
        network_path = os.path.join(FIXTURES, "sumo.net.xml")
        network = load_network(network_path)
        track = ActorTrack("profile-car", "vehicle.car", [
            TrackPoint(0, 0.0, 2.0, 0.0, 0.0),
            TrackPoint(1, 1.0, 8.0, 0.0, 0.0),
            TrackPoint(2, 3.0, 10.0, 0.0, 0.0),
        ])
        self.assertAlmostEqual(track.mean_speed, 8.0 / 3.0)
        self.assertAlmostEqual(track.peak_speed, 6.0)
        written_roots = []

        def capture_tree(tree, *args, **kwargs):
            written_roots.append(tree.getroot())

        with mock.patch("closed_loop.sumo_routes.load_network",
                        return_value=network), \
                mock.patch("closed_loop.sumo_routes.os.makedirs"), \
                mock.patch.object(ET.ElementTree, "write", autospec=True,
                                  side_effect=capture_tree), \
                mock.patch("builtins.open", mock.mock_open()):
            _, _, report = write_sumo_scenario(
                network_path, {track.actor_id: track}, "unused-output",
                "scene-profile", maximum_snap_distance=2.0,
                moving_initial_pose="recorded",
                moving_speed_policy="recorded_profile",
                end_time=5.0)

        route_root = written_roots[0]
        actor_type = route_root.find("./vType[@id='nusc_car_profile-car']")
        self.assertIsNotNone(actor_type)
        self.assertEqual(actor_type.get("maxSpeed"), "6.000")
        self.assertNotEqual(
            float(actor_type.get("maxSpeed")), track.mean_speed)

        self.assertEqual(report["moving_speed"]["policy"],
                         "recorded_profile")
        self.assertEqual(
            report["moving_speed"]["terminal_stop_policy"],
            "release_to_sumo")
        self.assertEqual(
            report["moving_speed"]["terminal_stop_maximum_extent_m"],
            1.0)
        self.assertEqual(
            report["moving_speed"]["terminal_stop_minimum_duration_s"],
            2.0)
        self.assertEqual(report["moving_initialization"], {
            "position_policy": "recorded",
        })
        included = report["included"][0]
        self.assertAlmostEqual(included["peak_speed_mps"], 6.0)
        self.assertAlmostEqual(included["sumo_max_speed_mps"], 6.0)
        self.assertEqual(included["initial_sumo_pose"], {
            "x": 12.0,
            "y": 20.0,
            "angle_degrees": 90.0,
            "reference_point": "vehicle_center",
        })
        self.assertAlmostEqual(included["sumo_vehicle_length_m"], 4.7)


class HybridAuthorityTests(unittest.TestCase):
    def test_explicit_authority_precedes_automatic_static_authority(self):
        specs = configured_carla_actor_specs({
            "critical_actors": [
                {"track_id": "critical", "authority": "carla_reference"},
            ],
            "static_actors": [
                {"track_id": "parked"},
                {"track_id": "critical", "authority": "carla_static"},
            ],
        }, {"track_id": "ego", "authority": "carla_replay"})

        self.assertEqual(
            [spec["track_id"] for spec in specs],
            ["ego", "critical", "parked"])
        self.assertEqual(specs[1]["authority"], "carla_reference")
        self.assertEqual(specs[2]["authority"], "carla_static")
        self.assertEqual(actor_spawn_policy("ego", "carla_replay"),
                         ("hero", False))
        self.assertEqual(actor_spawn_policy("parked", "carla_static"),
                         ("static_recorded", False))
        self.assertEqual(actor_spawn_policy("critical", "carla_reference"),
                         ("critical_actor", True))

    def test_legacy_config_without_static_actors_still_works(self):
        specs = configured_carla_actor_specs(
            {"critical_actors": []},
            {"track_id": "ego", "authority": "carla_reference"})
        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0]["track_id"], "ego")

    def test_static_actor_authority_and_duplicates_are_validated(self):
        with self.assertRaisesRegex(ValueError, "authority must be carla_static"):
            configured_carla_actor_specs({
                "static_actors": [
                    {"track_id": "parked", "authority": "carla_reference"},
                ],
            }, {"track_id": "ego"})
        with self.assertRaisesRegex(ValueError, "duplicate.*parked"):
            configured_carla_actor_specs({
                "static_actors": [
                    {"track_id": "parked"},
                    {"track_id": "parked"},
                ],
            }, {"track_id": "ego"})
        with self.assertRaisesRegex(ValueError, "require track_id"):
            configured_carla_actor_specs({
                "static_actors": [{"track_id": None}],
            }, {"track_id": "ego"})


class MetricTests(unittest.TestCase):
    def test_closing_ttc_and_summary(self):
        self.assertAlmostEqual(
            closing_ttc((0, 0), (10, 0), (20, 0), (0, 0)), 2.0)
        self.assertTrue(math.isinf(
            closing_ttc((0, 0), (0, 0), (20, 0), (10, 0))))
        metrics = SafetyMetrics()
        metrics.update(0.0,
                       {"position": (0, 0), "velocity": (10, 0), "radius": 1.0},
                       {"npc": {"position": (20, 0), "velocity": (0, 0),
                                "radius": 1.0}})
        self.assertAlmostEqual(metrics.summary()["minimum_distance_m"], 20.0)
        self.assertAlmostEqual(metrics.summary()["minimum_ttc_s"], 1.8)


if __name__ == "__main__":
    unittest.main()
