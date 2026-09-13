"""Synthetic offline checks: no running CARLA server or nuScenes download needed."""

import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image, ImageSequence

from carla_reconstruction.visualization import panels
from carla_reconstruction.visualization.scene_data import CAMERA_CHANNELS


class VisualizationPanelTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.scene, self.capture, self.index = self.fixture("test-town_scene-0001")

    def fixture(self, name):
        root = self.root / name
        root.mkdir()
        manifest = root / "scene_manifest.json"
        meta = root / "scene_meta.json"
        manifest.write_text(json.dumps({"scene": name, "source": {"meta": str(meta)}}), encoding="utf-8")
        meta.write_text(json.dumps({"name": name}), encoding="utf-8")
        capture = root / "capture"
        capture.mkdir()
        frames = []
        rows = []
        for index in range(3):
            cameras = {}
            sim_cameras = {}
            for camera_index, channel in enumerate(CAMERA_CHANNELS):
                real_path = root / ("real_%s_%d.jpg" % (channel, index))
                sim_path = capture / ("sim_%s_%d.png" % (channel, index))
                Image.new("RGB", (80, 45), (30 * camera_index, 40 * index, 60)).save(real_path)
                Image.new("RGB", (80, 45), (30 * camera_index, 40 * index, 180)).save(sim_path)
                cameras[channel] = str(real_path)
                sim_cameras[channel] = sim_path.name
            raw_path = capture / ("lidar_%d.npy" % index)
            np.save(raw_path, np.array([[1, 2, 3, 0.8], [4, -2, 1, 0.5]], dtype=np.float32))
            common = {"index": index, "sample_token": "%s-sample-%d" % (name, index),
                      "timestamp_us": 1000000 + 500000 * index, "relative_time_s": 0.5 * index}
            frames.append(dict(common, cameras=cameras, lidar={"path": "synthetic.bin"}))
            rows.append(dict(common, cameras=sim_cameras, lidar_file=raw_path.name,
                             lidar_sensor_world_matrix=np.eye(4).tolist(), ego_carla_xyz=[index, 0, 0],
                             ego_map_yaw_rad=0, actors=[{"track_id": "ego", "category": "vehicle.car", "is_ego": True,
                                                       "x": index, "y": 0, "yaw_rad": 0, "length": 4, "width": 2}]))
        geometry = {"coordinate_frame": "opendrive_patch_local_metres",
                    "roads": [{"road_id": 1, "lane_id": -1, "section_id": 0,
                               "polygon": [[-40, -4], [40, -4], [40, 4], [-40, 4]]}],
                    "environment": {"buildings": [{"x": 8, "y": 12, "half_x": 4, "half_y": 3, "yaw_rad": 0.3}],
                                    "trees": [{"x": -6, "y": 10, "radius": 1.5}],
                                    "nuscenes_static_objects": [{"category": "movable_object.trafficcone", "x": 3, "y": 5,
                                                                 "length": 0.4, "width": 0.3, "yaw_rad": 0},
                                                                {"category": "movable_object.barrier", "x": 6, "y": 5,
                                                                 "length": 2, "width": 0.5, "yaw_rad": 0.1}],
                                    "traffic_signs": [{"x": 10, "y": 3}],
                                    "traffic_lights": [{"x": 12, "y": 3}]}}
        (capture / "map_geometry.json").write_text(json.dumps(geometry), encoding="utf-8")
        sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
        index = {"schema_version": 1, "scene": name, "manifest_path": str(manifest),
                 "manifest_sha256": sha(manifest), "meta_sha256": sha(meta),
                 "mode": "recorded_reconstruction_replay", "complete": True,
                 "frame_dt_s": 0.5, "map_geometry_file": "map_geometry.json", "frames": rows,
                 "capture_parameters": {"requested_indices": [0, 1, 2]}}
        self.write_index(capture, index)
        scene = {"scene": name, "manifest_path": str(manifest), "meta_path": str(meta),
                 "manifest": json.loads(manifest.read_text(encoding="utf-8")), "frames": frames}
        return scene, capture, index

    def write_index(self, capture=None, index=None):
        capture = capture or self.capture
        index = index or self.index
        (capture / "capture_index.json").write_text(json.dumps(index), encoding="utf-8")

    def test_builds_both_scenes_independently(self):
        other_scene, other_capture, _ = self.fixture("test-town_scene-0002")
        for scene, capture in ((self.scene, self.capture), (other_scene, other_capture)):
            output = self.root / (scene["scene"] + "_panels")
            result = panels.build_scene(scene, capture, output, cell_width=64)
            self.assertEqual(result["scene"], scene["scene"])
            self.assertEqual(len(result["frames"]), 3)
            self.assertEqual(result["gif_duration_ms"], [500, 500, 500])
            with Image.open(output / "aligned.gif") as gif:
                self.assertEqual(gif.n_frames, 3)
                self.assertEqual(gif.info["loop"], 0)
                durations = [frame.info["duration"] for frame in ImageSequence.Iterator(gif)]
                self.assertEqual(durations, [500, 500, 500])
            with Image.open(output / "frames" / "000.png") as image:
                self.assertEqual(image.size, (352, 152))

    def test_full_includes_matching_lidar_and_has_wider_layout(self):
        output = self.root / "full"
        with patch.object(panels, "real_lidar_points", return_value=np.array([[0, 0, 0], [1, 2, 1]])):
            result = panels.build_scene(self.scene, self.capture, output, full=True, cell_width=64)
        self.assertEqual(result["layout"], "aligned_full")
        with Image.open(output / "frames" / "000.png") as image:
            self.assertEqual(image.size, (432, 152))

    def test_token_mismatch_does_not_create_output(self):
        self.index["frames"][1]["sample_token"] = "wrong-sample"
        self.write_index()
        output = self.root / "should_not_exist"
        with self.assertRaisesRegex(ValueError, "sample_token mismatch"):
            panels.build_scene(self.scene, self.capture, output)
        self.assertFalse(output.exists())

    def test_timestamp_and_relative_time_must_match(self):
        original = copy.deepcopy(self.index)
        for key in ("timestamp_us", "relative_time_s"):
            self.index = copy.deepcopy(original)
            self.index["frames"][0][key] += 1
            self.write_index()
            with self.assertRaisesRegex(ValueError, key + " mismatch"):
                panels.validate_capture(self.scene, self.capture)

    def test_scene_and_hash_match_not_directory_order(self):
        other, _, _ = self.fixture("test-town_scene-0002")
        with self.assertRaisesRegex(ValueError, "Capture scene"):
            panels.validate_capture(other, self.capture)
        self.index["manifest_sha256"] = "bad"
        self.write_index()
        with self.assertRaisesRegex(ValueError, "manifest_sha256"):
            panels.validate_capture(self.scene, self.capture)

    def test_different_dataset_root_is_valid_if_manifest_identity_matches(self):
        new_root = self.root / "relocated_dataset"
        new_root.mkdir()
        for frame in self.scene["frames"]:
            for channel, filename in frame["cameras"].items():
                path = Path(filename)
                target = new_root / path.name
                target.write_bytes(path.read_bytes())
                frame["cameras"][channel] = str(target)
        self.assertEqual(len(panels.validate_capture(self.scene, self.capture)[2]), 3)

    def test_missing_index_is_a_clear_error(self):
        (self.capture / "capture_index.json").unlink()
        with self.assertRaisesRegex(FileNotFoundError, "Capture index not found"):
            panels.validate_capture(self.scene, self.capture)

    def test_missing_sim_camera_is_not_silently_dropped(self):
        name = self.index["frames"][1]["cameras"][CAMERA_CHANNELS[0]]
        (self.capture / name).unlink()
        with self.assertRaisesRegex(FileNotFoundError, "Missing indexed capture asset"):
            panels.validate_capture(self.scene, self.capture, allow_partial=True)

    def test_missing_camera_key_is_a_clear_error(self):
        del self.index["frames"][0]["cameras"][CAMERA_CHANNELS[0]]
        self.write_index()
        with self.assertRaisesRegex(ValueError, "missing camera"):
            panels.validate_capture(self.scene, self.capture)

    def test_corrupt_real_image_is_rejected_before_outputs(self):
        Path(self.scene["frames"][1]["cameras"][CAMERA_CHANNELS[0]]).write_bytes(b"broken image")
        with self.assertRaisesRegex(ValueError, "Unreadable camera image"):
            panels.validate_capture(self.scene, self.capture)

    def test_parent_directory_escape_is_rejected(self):
        self.index["frames"][0]["cameras"][CAMERA_CHANNELS[0]] = "../scene_manifest.json"
        self.write_index()
        with self.assertRaisesRegex(ValueError, "escapes capture directory"):
            panels.validate_capture(self.scene, self.capture)

    def test_absolute_capture_path_is_rejected(self):
        self.index["frames"][0]["cameras"][CAMERA_CHANNELS[0]] = str(self.capture / "anything.png")
        self.write_index()
        with self.assertRaisesRegex(ValueError, "relative paths"):
            panels.validate_capture(self.scene, self.capture)

    def test_duplicate_indexes_are_rejected(self):
        self.index["frames"][1]["index"] = 0
        self.write_index()
        with self.assertRaisesRegex(ValueError, "unique and strictly increasing"):
            panels.validate_capture(self.scene, self.capture)

    def test_incomplete_needs_explicit_allow_partial_and_preserves_gap(self):
        self.index["complete"] = False
        del self.index["frames"][1]
        self.write_index()
        with self.assertRaisesRegex(ValueError, "incomplete"):
            panels.validate_capture(self.scene, self.capture)
        output = self.root / "partial"
        result = panels.build_scene(self.scene, self.capture, output, cell_width=64, allow_partial=True)
        self.assertEqual([frame["index"] for frame in result["frames"]], [0, 2])
        self.assertEqual(result["gif_duration_ms"], [1000, 1000])
        self.assertTrue(result["partial_capture"])
        self.assertTrue((output / "frames" / "002.png").exists())
        self.assertFalse((output / "frames" / "001.png").exists())

    def test_declared_stride_subset_is_a_complete_capture(self):
        del self.index["frames"][1]
        self.index["capture_parameters"]["requested_indices"] = [0, 2]
        self.write_index()
        self.assertEqual(len(panels.validate_capture(self.scene, self.capture)[2]), 2)

    def test_complete_missing_index_is_rejected(self):
        del self.index["frames"][1]
        self.write_index()
        with self.assertRaisesRegex(ValueError, "missing source frame indexes"):
            panels.validate_capture(self.scene, self.capture)

    def test_nonempty_output_is_preserved(self):
        output = self.root / "existing"
        output.mkdir()
        original = output / "keep.txt"
        original.write_text("user data", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "not empty"):
            panels.build_scene(self.scene, self.capture, output)
        self.assertEqual(original.read_text(encoding="utf-8"), "user data")

    def test_invalid_geometry_or_missing_geometry_are_rejected(self):
        path = self.capture / "map_geometry.json"
        path.write_text(json.dumps({"coordinate_frame": "wrong", "roads": []}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "coordinate frame"):
            panels.validate_capture(self.scene, self.capture)
        path.unlink()
        with self.assertRaisesRegex(FileNotFoundError, "Missing indexed"):
            panels.validate_capture(self.scene, self.capture)

    def test_full_lidar_must_exist_and_be_nx4(self):
        path = self.capture / self.index["frames"][0]["lidar_file"]
        np.save(path, np.ones((2, 3)))
        with self.assertRaisesRegex(ValueError, "Nx4"):
            panels.validate_capture(self.scene, self.capture, full=True)
        path.unlink()
        with self.assertRaisesRegex(FileNotFoundError, "Missing indexed"):
            panels.validate_capture(self.scene, self.capture, full=True)

    def test_dimensions_fps_and_window_validation_precedes_output(self):
        for options in ({"cell_width": 4}, {"window_m": 0}, {"fps": 0}):
            output = self.root / "invalid"
            with self.assertRaises(ValueError):
                panels.build_scene(self.scene, self.capture, output, **options)
            self.assertFalse(output.exists())

    def test_north_up_projection_and_actor_heading(self):
        self.assertEqual(panels._project((0, 10), (0, 0), 100, 100), (50, 40))
        self.assertEqual(panels._project((10, 0), (0, 0), 100, 100), (60, 50))
        self.assertEqual(panels._box(0, 0, 4, 2, 0)[0], (2, 1))

    def test_static_prop_validation_precedes_output(self):
        path = self.capture / "map_geometry.json"
        geometry = json.loads(path.read_text(encoding="utf-8"))
        geometry["environment"]["nuscenes_static_objects"][0]["width"] = -1
        path.write_text(json.dumps(geometry), encoding="utf-8")
        output = self.root / "bad_props"
        with self.assertRaisesRegex(ValueError, "static prop dimensions"):
            panels.build_scene(self.scene, self.capture, output)
        self.assertFalse(output.exists())

    def test_gif_delays_are_quantized_to_supported_centiseconds(self):
        self.assertEqual(panels.frame_durations([{}, {}], fps=3), [330, 330])


if __name__ == "__main__":
    unittest.main()
