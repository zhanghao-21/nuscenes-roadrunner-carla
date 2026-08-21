import math
import os
import unittest

from closed_loop.metrics import SafetyMetrics, closing_ttc
from closed_loop.sumo_routes import load_network, plan_track_route
from closed_loop.tracks import carla_pose, point_at, tracks_from_meta


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
