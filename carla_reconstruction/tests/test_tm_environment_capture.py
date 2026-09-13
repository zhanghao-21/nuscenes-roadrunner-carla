"""Regression checks for lane placement and continuous, frame-aligned capture."""

import argparse
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np
from PIL import Image

from carla_reconstruction.closed_loop.environment_scenarios import (
    add_environment_arguments, validate_environment_arguments, waypoint_ahead)
from carla_reconstruction.visualization.capture import SensorFrameBuffer
from carla_reconstruction.visualization.scene_data import CAMERA_CHANNELS
from carla_reconstruction.visualization.sim_panels import build_sim_panels
from carla_reconstruction.visualization.tm_capture import (
    TMSensorCapture, add_capture_arguments, validate_capture_arguments)


class LanePlacementTests(unittest.TestCase):
    def test_follows_curved_lane_instead_of_straight_heading_offset(self):
        class Waypoint:
            def __init__(self, s):
                self.s = s
            def next(self, distance):
                return [Waypoint(self.s + distance)]
        self.assertAlmostEqual(waypoint_ahead(Waypoint(2), 5.5).s, 7.5)

    def test_refuses_an_ambiguous_fork(self):
        with self.assertRaisesRegex(ValueError, "fork"):
            waypoint_ahead(NS(next=lambda step: [NS(), NS()]), 12)

    def test_rejects_invalid_options_before_world_access(self):
        parser = argparse.ArgumentParser()
        add_environment_arguments(parser)
        for distance in ("nan", "inf", "-1", "4"):
            args = parser.parse_args(["--obstacle-distance", distance])
            with self.assertRaises(ValueError):
                validate_environment_arguments(args)


class ContinuousCaptureTests(unittest.TestCase):
    def test_delayed_sensors_keep_original_tick_state_and_chronological_order(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = object.__new__(TMSensorCapture)
            capture.root = Path(directory)
            (capture.root / "lidar").mkdir()
            capture.buffer = SensorFrameBuffer(["CAM_FRONT", "LIDAR_TOP"])
            capture.audit = {"frames": []}
            capture.requested = {
                10: {"relative_time_s": 0.5, "ego_carla_xyz": [10, 0, 0]},
                20: {"relative_time_s": 1.0, "ego_carla_xyz": [20, 0, 0]},
            }

            def send(frame, channel):
                value = NS(frame=frame, timestamp=frame * .05,
                           raw_data=np.ones((3, 4), dtype=np.float32).tobytes(),
                           transform=NS(get_matrix=lambda: np.eye(4).tolist()),
                           save_to_disk=lambda path: None)
                capture.buffer.callback(channel)(value)

            send(10, "LIDAR_TOP")
            send(20, "LIDAR_TOP")
            send(20, "CAM_FRONT")
            capture.flush()
            self.assertEqual(capture.audit["frames"], [])
            send(10, "CAM_FRONT")
            capture.flush()
            self.assertEqual([r["ego_carla_xyz"][0] for r in capture.audit["frames"]], [10, 20])
            self.assertEqual([r["sensor_frame_ids"]["CAM_FRONT"] for r in capture.audit["frames"]], [10, 20])
            self.assertFalse(capture.requested)

    def test_capture_requires_rendering_and_new_outputs(self):
        parser = argparse.ArgumentParser()
        add_capture_arguments(parser)
        args = parser.parse_args(["--capture-sensors"])
        args.step_length, args.no_rendering = .05, True
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "rendering"):
                validate_capture_arguments(args, directory, 20)
            args.no_rendering = False
            root = Path(directory) / "capture"
            root.mkdir()
            (root / "existing.txt").write_text("keep")
            with self.assertRaisesRegex(ValueError, "not empty"):
                validate_capture_arguments(args, directory, 20)


class SimPanelTests(unittest.TestCase):
    def fixture(self, root):
        cameras = {}
        for channel in CAMERA_CHANNELS:
            path = root / (channel + ".png")
            Image.new("RGB", (64, 36), (100, 120, 140)).save(path)
            cameras[channel] = path.name
        np.save(root / "lidar.npy", np.array([[2., 3., 1., .8]], dtype=np.float32))
        capture = {"schema_version": 1, "mode": "traffic_manager_sensor_capture",
                   "complete": True, "scene": "test", "frame_dt_s": .5,
                   "scenario": {"weather": "rain", "obstacle": None}, "frames": []}
        for i in range(2):
            capture["frames"].append({
                "index": i, "carla_frame": 10 + i, "relative_time_s": .5 + i * .5,
                "timestamp_us": 500000 + i * 500000, "world_timestamp_s": 4 + i * .5,
                "sensor_frame_ids": {key: 10 + i for key in list(CAMERA_CHANNELS) + ["LIDAR_TOP"]},
                "sensor_timestamps_s": {key: 4 + i * .5 for key in list(CAMERA_CHANNELS) + ["LIDAR_TOP"]},
                "cameras": cameras, "lidar_file": "lidar.npy", "ego_carla_xyz": [0, 0, 0],
                "lidar_sensor_world_matrix": np.eye(4).tolist(), "ego_speed_kmh": 12,
            })
        (root / "capture_index.json").write_text(json.dumps(capture))
        return capture

    def test_builds_sim_only_png_and_timed_gif_without_dataset(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.fixture(root)
            result = build_sim_panels(root, root / "panels", cell_width=160)
            self.assertEqual(result["layout"], "sim_six_cameras_one_lidar")
            self.assertEqual(result["gif_duration_ms"], [500, 500])
            with Image.open(root / "panels" / "simulation.gif") as gif:
                self.assertEqual(gif.n_frames, 2)
            self.assertTrue((root / "panels" / "figure.png").is_file())

    def test_mismatched_lidar_frame_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = self.fixture(root)
            capture["frames"][0]["sensor_frame_ids"]["LIDAR_TOP"] = 9
            (root / "capture_index.json").write_text(json.dumps(capture))
            with self.assertRaisesRegex(ValueError, "share"):
                build_sim_panels(root, root / "panels")
            self.assertFalse((root / "panels").exists())

    def test_same_frame_with_a_camera_timestamp_from_next_tick_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = self.fixture(root)
            capture["frames"][0]["sensor_timestamps_s"]["CAM_BACK_RIGHT"] += .05
            (root / "capture_index.json").write_text(json.dumps(capture))
            with self.assertRaisesRegex(ValueError, "timestamps"):
                build_sim_panels(root, root / "panels")
            self.assertFalse((root / "panels").exists())


if __name__ == "__main__":
    unittest.main()
