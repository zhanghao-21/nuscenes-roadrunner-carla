import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from visualization.scene_data import (
    CAMERA_CHANNELS, CAMERA_GRID, carla_lidar_points, load_scene,
    quaternion_matrix, read_manifest_selection, real_lidar_points,
)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


class VisualizationDataTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.dataset = self.root / "dataset"
        self.tables = self.dataset / "v1.0-mini"
        self.generated = self.root / "generated"
        self.manifest_path = self.generated / "test-map_scene-0001" / "scene_manifest.json"
        self.meta_path = self.manifest_path.parent / "trajectory.json"
        self.meta = {
            "scene": "scene-0001", "map": "test-map", "origin": {"x": 90, "y": 180},
            "ego_trajectory_local": [{"x": 10, "y": 20, "yaw": 0},
                                     {"x": 11, "y": 22, "yaw": 0.2}],
            "agent_frames": [[], []],
        }
        self.manifest = {
            "scene": "test-map_scene-0001", "nuscenes_scene": "scene-0001", "nuscenes_map": "test-map",
            "coordinate_frame": {"name": "opendrive_patch_local_metres", "unreal_flip_y": True},
            "source": {"nuscenes_dataroot": str(self.dataset), "nuscenes_version": "v1.0-mini"},
            "trajectories": {"meta": "trajectory.json", "ego_keyframes": 2, "agent_keyframes": 2},
        }
        self.raw = {
            "scene": [{"token": "scene-token", "name": "scene-0001", "first_sample_token": "a",
                       "last_sample_token": "b", "nbr_samples": 2, "log_token": "log-token"}],
            "sample": [{"token": "a", "scene_token": "scene-token", "timestamp": 1_000_000,
                        "prev": "", "next": "b"},
                       {"token": "b", "scene_token": "scene-token", "timestamp": 1_501_234,
                        "prev": "a", "next": ""}],
            "sample_data": [], "calibrated_sensor": [], "sensor": [],
            "ego_pose": [
                {"token": "pose-a", "translation": [100, 200, 1], "rotation": [1, 0, 0, 0]},
                {"token": "pose-b", "translation": [101, 202, 2],
                 "rotation": [math.cos(0.1), 0, 0, math.sin(0.1)]},
            ],
            "log": [{"token": "log-token", "location": "test-map"}],
        }
        for number, channel in enumerate(CAMERA_CHANNELS + ("LIDAR_TOP",)):
            self.raw["sensor"].append({"token": "sensor-" + channel, "channel": channel,
                                       "modality": "lidar" if channel == "LIDAR_TOP" else "camera"})
            self.raw["calibrated_sensor"].append({"token": "cal-" + channel,
                "sensor_token": "sensor-" + channel, "translation": [1, 2, 3],
                "rotation": [1, 0, 0, 0]})
            for index, token in enumerate(("a", "b")):
                # File names do not contain a camera channel: calibration links
                # must determine channels, not a convenient filename pattern.
                filename = "samples/input-{}-{}.{}".format(number, index, "bin" if channel == "LIDAR_TOP" else "jpg")
                self.raw["sample_data"].append({
                    "token": token + "-" + channel, "sample_token": token, "is_key_frame": True,
                    "calibrated_sensor_token": "cal-" + channel, "ego_pose_token": "pose-" + token,
                    "filename": filename, "timestamp": self.raw["sample"][index]["timestamp"] - number * 100,
                })
                path = self.dataset / filename
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(np.asarray([[2, 3, 4, 1, 0]], dtype=np.float32).tobytes()
                                 if channel == "LIDAR_TOP" else b"image placeholder")
        self.save()

    def save(self):
        write_json(self.manifest_path, self.manifest)
        write_json(self.meta_path, self.meta)
        for name, records in self.raw.items():
            write_json(self.tables / (name + ".json"), records)

    def test_loads_sensor_links_exact_order_and_original_timestamps(self):
        self.raw["sample_data"].reverse()
        self.raw["sample"].reverse()
        self.save()
        scene = load_scene(self.manifest_path)
        self.assertEqual(scene["scene"], "test-map_scene-0001")
        self.assertEqual(scene["meta_path"], str(self.meta_path.resolve()))
        self.assertEqual(len(scene["manifest_sha256"]), 64)
        self.assertEqual(len(scene["meta_sha256"]), 64)
        self.assertEqual([frame["sample_token"] for frame in scene["frames"]], ["a", "b"])
        self.assertEqual([frame["index"] for frame in scene["frames"]], [0, 1])
        frame = scene["frames"][1]
        self.assertAlmostEqual(frame["relative_time_s"], 0.501234)
        self.assertEqual(frame["timestamp_us"], 1_501_234)
        self.assertEqual(frame["camera_sample_data"]["CAM_FRONT"]["timestamp_us"], 1_501_134)
        self.assertEqual(frame["lidar"]["timestamp_us"], 1_500_634)
        self.assertEqual(set(frame["cameras"]), set(CAMERA_CHANNELS))
        self.assertTrue(all(Path(path).is_absolute() for path in frame["cameras"].values()))
        self.assertEqual(CAMERA_GRID[0], ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT"))

    def test_dataroot_override_and_table_directory_fallback(self):
        self.manifest["source"]["nuscenes_dataroot"] = "nonexistent"
        self.save()
        scene = load_scene(self.manifest_path, dataroot=self.tables)
        self.assertEqual(scene["dataroot"], str(self.dataset.resolve()))

    def test_relative_source_dataroot_is_relative_to_manifest(self):
        self.manifest["source"]["nuscenes_dataroot"] = "../../dataset"
        self.save()
        self.assertEqual(load_scene(self.manifest_path)["dataroot"], str(self.dataset.resolve()))

    def test_missing_tables_explain_dataroot(self):
        with self.assertRaisesRegex(FileNotFoundError, "--dataroot"):
            load_scene(self.manifest_path, dataroot=self.root / "not-there")

    def test_meta_identity_mismatch_rejected(self):
        for field, value in (("scene", "scene-0002"), ("map", "another-map")):
            with self.subTest(field=field):
                original = self.meta[field]
                self.meta[field] = value
                self.save()
                with self.assertRaisesRegex(ValueError, "identity"):
                    load_scene(self.manifest_path)
                self.meta[field] = original

    def test_missing_or_unsupported_coordinate_frame_rejected(self):
        for coordinate_frame in (None, {}, {"name": "global", "unreal_flip_y": True},
                                 {"name": "opendrive_patch_local_metres", "unreal_flip_y": False},
                                 {"name": "opendrive_patch_local_metres", "unreal_flip_y": "true"}):
            with self.subTest(coordinate_frame=coordinate_frame):
                self.manifest["coordinate_frame"] = coordinate_frame
                self.save()
                with self.assertRaisesRegex(ValueError, "coordinate_frame.name"):
                    load_scene(self.manifest_path)

    def test_raw_map_mismatch_rejected(self):
        self.raw["log"][0]["location"] = "different-map"
        self.save()
        with self.assertRaisesRegex(ValueError, "log location"):
            load_scene(self.manifest_path)

    def test_wrong_combined_scene_name_rejected(self):
        self.manifest["scene"] = "different-map_scene-0001"
        self.save()
        with self.assertRaisesRegex(ValueError, "Manifest scene"):
            load_scene(self.manifest_path)

    def test_sample_chain_cycle_rejected(self):
        self.raw["sample"][1]["next"] = "a"
        self.save()
        with self.assertRaisesRegex(ValueError, "Cycle"):
            load_scene(self.manifest_path)

    def test_missing_sample_chain_link_rejected(self):
        self.raw["sample"][0]["next"] = "missing"
        self.save()
        with self.assertRaisesRegex(ValueError, "missing sample token"):
            load_scene(self.manifest_path)

    def test_nonmonotonic_timestamps_rejected(self):
        self.raw["sample"][1]["timestamp"] = 999999
        self.save()
        with self.assertRaisesRegex(ValueError, "Nonmonotonic"):
            load_scene(self.manifest_path)

    def test_sample_from_another_scene_rejected(self):
        self.raw["sample"][1]["scene_token"] = "another-scene"
        self.save()
        with self.assertRaisesRegex(ValueError, "different scene"):
            load_scene(self.manifest_path)

    def test_broken_sample_prev_link_rejected(self):
        self.raw["sample"][1]["prev"] = "wrong"
        self.save()
        with self.assertRaisesRegex(ValueError, "Broken prev"):
            load_scene(self.manifest_path)

    def test_declared_sample_count_rejected(self):
        self.raw["scene"][0]["nbr_samples"] = 3
        self.save()
        with self.assertRaisesRegex(ValueError, "sample-chain length"):
            load_scene(self.manifest_path)

    def test_metadata_trajectory_count_rejected_without_truncating(self):
        self.meta["ego_trajectory_local"].pop()
        self.save()
        with self.assertRaisesRegex(ValueError, "silently truncate"):
            load_scene(self.manifest_path)

    def test_agent_frame_count_rejected(self):
        self.meta["agent_frames"].pop()
        self.save()
        with self.assertRaisesRegex(ValueError, "agent-frame count"):
            load_scene(self.manifest_path)

    def test_missing_agent_frames_rejected(self):
        del self.meta["agent_frames"]
        self.save()
        with self.assertRaisesRegex(ValueError, "must include agent_frames"):
            load_scene(self.manifest_path)

    def test_agent_frames_must_be_lists(self):
        self.meta["agent_frames"] = {"0": [], "1": []}
        self.save()
        with self.assertRaisesRegex(ValueError, "must include agent_frames"):
            load_scene(self.manifest_path)
        self.meta["agent_frames"] = [{}, []]
        self.save()
        with self.assertRaisesRegex(ValueError, "must be a list of annotations"):
            load_scene(self.manifest_path)

    def test_duplicate_actor_ids_in_same_frame_rejected(self):
        actor = {"id": "vehicle-a", "x": 10, "y": 20, "yaw": 0}
        self.meta["agent_frames"] = [[actor, dict(actor, x=11)], []]
        self.save()
        with self.assertRaisesRegex(ValueError, "duplicate actor id 'vehicle-a'"):
            load_scene(self.manifest_path)
        self.meta["agent_frames"] = [[actor], [dict(actor, x=11)]]
        self.save()
        self.assertEqual(len(load_scene(self.manifest_path)["frames"]), 2)

    def test_agent_annotations_require_id_and_finite_pose(self):
        for annotation in ({"x": 1, "y": 2, "yaw": 0}, {"id": "", "x": 1, "y": 2, "yaw": 0},
                           {"id": "a", "x": float("nan"), "y": 2, "yaw": 0},
                           {"id": "a", "x": 1, "y": float("inf"), "yaw": 0},
                           {"id": "a", "x": 1, "y": 2, "yaw": None},
                           {"id": "a", "x": 1, "y": 2}, {"id": "a", "x": "bad", "y": 2, "yaw": 0}):
            with self.subTest(annotation=annotation):
                self.meta["agent_frames"] = [[annotation], []]
                self.save()
                with self.assertRaisesRegex(ValueError, "without an id|finite x, y and yaw"):
                    load_scene(self.manifest_path)

    def test_declared_manifest_frame_count_rejected(self):
        self.manifest["trajectories"]["ego_keyframes"] = 9
        self.save()
        with self.assertRaisesRegex(ValueError, "ego_keyframes"):
            load_scene(self.manifest_path)

    def test_ego_position_mismatch_rejected(self):
        self.meta["ego_trajectory_local"][0]["x"] += 1
        self.save()
        with self.assertRaisesRegex(ValueError, "Frame 0 .* ego pose disagrees"):
            load_scene(self.manifest_path)

    def test_ego_yaw_mismatch_rejected(self):
        self.meta["ego_trajectory_local"][1]["yaw"] += 0.1
        self.save()
        with self.assertRaisesRegex(ValueError, "yaw tolerance"):
            load_scene(self.manifest_path)

    def test_ego_yaw_wrap_is_accepted(self):
        self.meta["ego_trajectory_local"][1]["yaw"] += 2 * math.pi
        self.save()
        self.assertEqual(len(load_scene(self.manifest_path)["frames"]), 2)

    def test_missing_sensor_keyframe_is_actionable(self):
        self.raw["sample_data"] = [sd for sd in self.raw["sample_data"] if sd["token"] != "a-CAM_FRONT"]
        self.save()
        with self.assertRaisesRegex(FileNotFoundError, "missing keyframe sensor data for CAM_FRONT"):
            load_scene(self.manifest_path)

    def test_sweep_does_not_replace_missing_keyframe(self):
        self.raw["sample_data"][0]["is_key_frame"] = False
        self.save()
        with self.assertRaisesRegex(FileNotFoundError, "missing keyframe sensor"):
            load_scene(self.manifest_path)

    def test_missing_camera_file_is_actionable(self):
        (self.dataset / self.raw["sample_data"][0]["filename"]).unlink()
        with self.assertRaisesRegex(FileNotFoundError, "sensor file missing:.*--dataroot"):
            load_scene(self.manifest_path)

    def test_missing_lidar_file_is_actionable(self):
        (self.dataset / self.raw["sample_data"][-1]["filename"]).unlink()
        with self.assertRaisesRegex(FileNotFoundError, "LIDAR_TOP.*sensor file missing"):
            load_scene(self.manifest_path)

    def test_duplicate_keyframe_channel_rejected(self):
        duplicate = copy.deepcopy(self.raw["sample_data"][0])
        duplicate["token"] = "duplicate"
        self.raw["sample_data"].append(duplicate)
        self.save()
        with self.assertRaisesRegex(ValueError, "Duplicate keyframe"):
            load_scene(self.manifest_path)

    def test_duplicate_table_token_rejected(self):
        self.raw["ego_pose"].append(copy.deepcopy(self.raw["ego_pose"][0]))
        self.save()
        with self.assertRaisesRegex(ValueError, "duplicate token"):
            load_scene(self.manifest_path)

    def test_missing_calibration_link_rejected(self):
        self.raw["sample_data"][0]["calibrated_sensor_token"] = "missing"
        self.save()
        with self.assertRaisesRegex(ValueError, "missing calibrated_sensor token"):
            load_scene(self.manifest_path)

    def test_unsafe_sensor_paths_rejected_cross_platform(self):
        for value in ("../outside.jpg", "samples/../../outside.jpg", "samples\\..\\escape.jpg",
                      "/absolute/image.jpg", "C:\\absolute\\image.jpg", "C:relative.jpg"):
            with self.subTest(filename=value):
                self.raw["sample_data"][0]["filename"] = value
                self.save()
                with self.assertRaisesRegex(ValueError, "unsafe dataset filename"):
                    load_scene(self.manifest_path)

    def test_unsafe_version_rejected(self):
        for version in ("..", "../other", "x\\other", "C:other"):
            with self.subTest(version=version):
                with self.assertRaisesRegex(ValueError, "version must be"):
                    load_scene(self.manifest_path, version=version)

    def test_manifest_selection_full_short_and_all(self):
        self.assertEqual(read_manifest_selection(manifest=self.manifest_path), [self.manifest_path])
        self.assertEqual(read_manifest_selection(scene="test-map_scene-0001", generated_root=self.generated), [self.manifest_path])
        self.assertEqual(read_manifest_selection(scene="scene-0001", generated_root=self.generated), [self.manifest_path])
        self.assertEqual(read_manifest_selection(scene="0001", generated_root=self.generated), [self.manifest_path])
        # Nested manifests are not reconstruction scenes.
        write_json(self.generated / "closed_loop" / "nested" / "scene_manifest.json", self.manifest)
        self.assertEqual(read_manifest_selection(scene="all", generated_root=self.generated), [self.manifest_path])

    def test_manifest_selection_ambiguity_missing_and_mutual_exclusion(self):
        duplicate_path = self.generated / "other-map_scene-0001" / "scene_manifest.json"
        duplicate = dict(self.manifest, scene="other-map_scene-0001", nuscenes_map="other-map")
        write_json(duplicate_path, duplicate)
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            read_manifest_selection(scene="scene-0001", generated_root=self.generated)
        self.assertEqual(read_manifest_selection(scene="test-map_scene-0001", generated_root=self.generated), [self.manifest_path])
        with self.assertRaises(FileNotFoundError):
            read_manifest_selection(scene="not-there", generated_root=self.generated)
        with self.assertRaises(ValueError):
            read_manifest_selection(manifest=self.manifest_path, scene="all")
        with self.assertRaises(ValueError):
            read_manifest_selection(generated_root=self.generated)
        with self.assertRaises(FileNotFoundError):
            read_manifest_selection(scene="all", generated_root=self.root / "empty")

    def test_real_lidar_applies_sensor_and_ego_full_3d_rotations(self):
        frame = load_scene(self.manifest_path)["frames"][0]
        frame["lidar"]["calibrated_sensor"]["rotation"] = [math.sqrt(0.5), math.sqrt(0.5), 0, 0]
        frame["lidar"]["ego_pose"]["rotation"] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
        frame["lidar"]["ego_pose"]["translation"] = [1234, 9876, 19]
        np.testing.assert_allclose(real_lidar_points(frame), [[2, 3, 6]], atol=1e-10)

    def test_corrupt_lidar_byte_count_rejected(self):
        frame = load_scene(self.manifest_path)["frames"][0]
        Path(frame["lidar"]["path"]).write_bytes(b"a" * 21)
        with self.assertRaisesRegex(ValueError, "N x 5"):
            real_lidar_points(frame)


class VisualizationTransformTests(unittest.TestCase):
    def test_quaternion_normalizes_and_rotates(self):
        matrix = quaternion_matrix([2, 0, 0, 2])
        np.testing.assert_allclose(matrix @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        np.testing.assert_allclose(matrix @ matrix.T, np.eye(3), atol=1e-12)

    def test_quaternion_rejects_invalid_values(self):
        for quaternion in ([0, 0, 0, 0], [1, 2, 3], [1, float("nan"), 0, 0], [float("inf"), 0, 0, 0]):
            with self.subTest(quaternion=quaternion):
                with self.assertRaises(ValueError):
                    quaternion_matrix(quaternion)

    def test_carla_lidar_world_transform_mount_offset_and_handedness(self):
        matrix = np.eye(4)
        matrix[:3, :3] = quaternion_matrix([math.sqrt(0.5), 0, 0, math.sqrt(0.5)])
        # Ego at (10,20,3); mount translated +1 world X, +2 world Y,+2Z.
        matrix[:3, 3] = [11, 22, 5]
        raw = np.asarray([[2, 0, 1, 0.8], [0, 1, -2, 0.1]])
        np.testing.assert_allclose(carla_lidar_points(raw, matrix, [10, 20, 3]),
                                   [[1, -4, 3], [0, -2, 0]], atol=1e-12)
        np.testing.assert_array_equal(raw, [[2, 0, 1, 0.8], [0, 1, -2, 0.1]])

    def test_carla_lidar_empty_and_invalid_shapes(self):
        self.assertEqual(carla_lidar_points(np.empty((0, 4)), np.eye(4), [0, 0, 0]).shape, (0, 3))
        with self.assertRaisesRegex(ValueError, "N x 4"):
            carla_lidar_points(np.zeros((1, 3)), np.eye(4), [0, 0, 0])
        with self.assertRaisesRegex(ValueError, "4 x 4"):
            carla_lidar_points(np.zeros((1, 4)), np.eye(3), [0, 0, 0])
        with self.assertRaisesRegex(ValueError, "three finite"):
            carla_lidar_points(np.zeros((1, 4)), np.eye(4), [0, float("nan"), 0])
        malformed = np.eye(4)
        malformed[3, 0] = 1
        with self.assertRaisesRegex(ValueError, "affine"):
            carla_lidar_points(np.zeros((1, 4)), malformed, [0, 0, 0])


if __name__ == "__main__":
    unittest.main()
