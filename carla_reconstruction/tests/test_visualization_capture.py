"""Offline capture checks using sensor and map doubles, without CARLA."""

import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from carla_reconstruction.visualization import capture


def measurement(frame, label=""):
    return SimpleNamespace(frame=frame, label=label)


class SelectionTests(unittest.TestCase):
    def test_default_selects_every_source_frame(self):
        self.assertEqual(capture.selected_indices(4), [0, 1, 2, 3])

    def test_subset_keeps_original_indices_not_renumbered(self):
        self.assertEqual(capture.selected_indices(12, start=2, stride=3, limit=2), [2, 5])
        self.assertEqual(capture.selected_indices(12, start=2, stride=3, limit=99), [2, 5, 8, 11])
        self.assertEqual(capture.selected_indices(12, start=11, stride=100), [11])

    def test_invalid_counts_offsets_strides_and_limits_are_rejected(self):
        for parameters in ((0,), (-1,), (4, -1), (4, 4), (4, 0, 0),
                           (4, 0, -1), (4, 0, 1, 0), (4, 0, 1, -1)):
            with self.subTest(parameters=parameters):
                with self.assertRaisesRegex(ValueError, "start-frame"):
                    capture.selected_indices(*parameters)


class SensorFrameBufferTests(unittest.TestCase):
    def setUp(self):
        self.buffer = capture.SensorFrameBuffer(["camera", "lidar"])

    def publish(self, frame, names=("camera", "lidar")):
        for name in names:
            self.buffer.callback(name)(measurement(frame, name))

    def test_empty_sensor_set_and_unknown_sensor_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "at least one"):
            capture.SensorFrameBuffer([])
        with self.assertRaisesRegex(ValueError, "unknown sensor"):
            self.buffer.callback("unregistered")

    def test_arrival_order_does_not_mix_different_sensor_frames(self):
        self.publish(12, ("camera",))
        self.publish(11, ("lidar",))
        self.buffer.drain()
        self.assertIsNone(self.buffer.pop_aligned(10))
        self.publish(12, ("lidar",))
        self.buffer.drain()
        frame, items = self.buffer.pop_aligned(10)
        self.assertEqual(frame, 12)
        self.assertEqual({item.frame for item in items.values()}, {12})
        self.assertEqual(set(items), {"camera", "lidar"})

    def test_complete_frame_before_new_pose_threshold_is_not_used(self):
        self.publish(40)
        self.publish(41, ("camera",))
        self.buffer.drain()
        self.assertIsNone(self.buffer.pop_aligned(41))
        self.publish(41, ("lidar",))
        self.buffer.drain()
        self.assertEqual(self.buffer.pop_aligned(41)[0], 41)
        self.assertEqual(self.buffer.pending, {})

    def test_newest_complete_frame_selected_and_future_partial_retained(self):
        self.publish(10)
        self.publish(11)
        self.publish(12, ("camera",))
        self.buffer.drain()
        self.assertEqual(self.buffer.pop_aligned(10)[0], 11)
        self.assertEqual(set(self.buffer.pending), {12})
        self.assertEqual(set(self.buffer.pending[12]), {"camera"})

    def test_late_callbacks_remain_stale_for_next_keyframe(self):
        self.publish(10)
        self.buffer.drain()
        self.assertEqual(self.buffer.pop_aligned(10)[0], 10)
        self.publish(10)
        self.publish(11)
        self.buffer.drain()
        self.assertIsNone(self.buffer.pop_aligned(12))

    def test_pending_frames_are_bounded_if_sensor_stops_publishing(self):
        for frame in range(100):
            self.publish(frame, ("camera",))
        self.buffer.drain()
        self.assertEqual(sorted(self.buffer.pending), list(range(68, 100)))
        self.assertIsNone(self.buffer.pop_aligned(68))

    def test_empty_queue_drain_is_a_no_op(self):
        self.buffer.drain()
        self.assertEqual(self.buffer.pending, {})

    def test_capture_warms_up_and_returns_only_current_pose_measurements(self):
        buffer = self.buffer

        class World:
            frame = 100

            def tick(self):
                self.frame += 1
                for name in buffer.names:
                    buffer.callback(name)(measurement(self.frame))
                return self.frame

        world = World()
        self.publish(90)
        frame, items = buffer.capture(world, warmup_ticks=3, timeout=1.0)
        self.assertEqual(frame, 103)
        self.assertEqual(world.frame, 103)
        self.assertEqual({item.frame for item in items.values()}, {103})

    def test_delayed_sensor_can_complete_prior_tick_without_index_shift(self):
        buffer = self.buffer

        class World:
            frame = 100

            def tick(self):
                self.frame += 1
                buffer.callback("camera")(measurement(self.frame))
                # GPU/sensor delays need not be identical. The match must use
                # CARLA frame IDs, not callback order or latest world frame.
                buffer.callback("lidar")(measurement(self.frame - 1))
                return self.frame

        world = World()
        frame, items = buffer.capture(world, warmup_ticks=1, timeout=1.0)
        self.assertEqual(world.frame, 102)
        self.assertEqual(frame, 101)
        self.assertEqual({item.frame for item in items.values()}, {101})

    def test_timeout_reports_missing_alignment_instead_of_returning_partial(self):
        world = SimpleNamespace(tick=lambda: 50)
        self.buffer.pending[50] = {"camera": measurement(50)}
        # Deterministic simulated deadline; never wait on a real sensor.
        with patch.object(self.buffer, "drain"), patch.object(
                capture.time, "monotonic", side_effect=[0.0, 0.0, 1.0]):
            with self.assertRaisesRegex(TimeoutError, r"timed out.*50.*camera"):
                self.buffer.capture(world, warmup_ticks=1, timeout=0.5)


def waypoint(road, lane, section, station, x, y, width=4.0,
             right=(0.0, 1.0), lane_type="Driving"):
    return SimpleNamespace(
        road_id=road, lane_id=lane, section_id=section, s=station,
        lane_width=width, lane_type=lane_type,
        transform=SimpleNamespace(
            location=SimpleNamespace(x=x, y=y),
            get_right_vector=lambda: SimpleNamespace(x=right[0], y=right[1])),
    )


class MapGeometryTests(unittest.TestCase):
    def map(self, points, topology=()):
        return SimpleNamespace(generate_waypoints=lambda step: points,
                               get_topology=lambda: topology)

    def test_lane_width_station_order_and_carla_y_flip(self):
        points = [waypoint(2, -1, 0, 10, 10, 20),
                  waypoint(2, -1, 0, 0, 0, 20)]
        environment = {"buildings": [{"x": 3, "y": 4}]}
        result = capture.map_geometry(self.map(points), environment)
        self.assertEqual(result["coordinate_frame"], "opendrive_patch_local_metres")
        self.assertEqual(result["environment"], environment)
        self.assertEqual(result["roads"], [{
            "road_id": 2, "lane_id": -1, "section_id": 0,
            "polygon": [[0, -18], [10, -18], [10, -22], [0, -22]],
        }])

    def test_curved_topology_endpoint_and_local_heading_are_used(self):
        start = waypoint(3, 1, 0, 0, 0, 0, right=(0, -1))
        end = waypoint(3, 1, 0, 5, 4, 4, right=(1, 0))
        result = capture.map_geometry(self.map([start], [(start, end)]), {})
        self.assertEqual(result["roads"][0]["polygon"],
                         [[0, -2], [2, -4], [6, -4], [0, 2]])

    def test_lane_sections_stay_separate_and_nondriving_lanes_excluded(self):
        points = []
        for section in (1, 0):
            points.extend([waypoint(2, -1, section, 0, 10 * section, 0),
                           waypoint(2, -1, section, 1, 10 * section + 1, 0)])
        points.extend([waypoint(1, -2, 0, 0, 0, 0, lane_type="Sidewalk"),
                       waypoint(1, -2, 0, 1, 1, 0, lane_type="Sidewalk")])
        # A single isolated sample is not enough to form a surface polygon.
        points.append(waypoint(3, -1, 0, 0, 0, 0))
        result = capture.map_geometry(self.map(points), {})
        self.assertEqual([(road["road_id"], road["section_id"]) for road in result["roads"]],
                         [(2, 0), (2, 1)])

    def test_empty_or_nondrivable_maps_are_rejected(self):
        for points in ([], [waypoint(1, -1, 0, 0, 0, 0)],
                       [waypoint(1, -1, 0, 0, 0, 0, lane_type="Sidewalk")]):
            with self.subTest(points=points):
                with self.assertRaisesRegex(ValueError, "no driving-lane geometry"):
                    capture.map_geometry(self.map(points), {})


class CaptureMetadataTests(unittest.TestCase):
    def test_json_index_and_hash_are_written_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "index.json"
            capture.write_json(path, {"complete": False, "frames": []})
            capture.write_json(path, {"complete": True, "frames": [7]})
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")),
                             {"complete": True, "frames": [7]})
            self.assertFalse(path.with_suffix(".json.tmp").exists())
            self.assertEqual(capture.file_sha256(path),
                             hashlib.sha256(path.read_bytes()).hexdigest())

    def test_nonfinite_json_does_not_overwrite_valid_index(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "index.json"
            capture.write_json(path, {"complete": False})
            original = path.read_bytes()
            with self.assertRaises(ValueError):
                capture.write_json(path, {"bad": float("nan")})
            self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
