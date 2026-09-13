import copy
import unittest

from tools.prepare_0757_marking_override import DIVIDERS, make_override


class Boston0757OverrideTests(unittest.TestCase):
    def setUp(self):
        self.manifest = {"scene": "boston-seaport_scene-0757", "nuscenes_map": "boston-seaport",
                         "map": {"asset_name": "Nusc_boston_seaport_0757"}}
        self.source = {
            "node": [{"token": "a", "x": 1, "y": 2}, {"token": "b", "x": 3, "y": 4}],
            "line": [{"token": "line", "node_tokens": ["a", "b"]}],
            "road_divider": [{"token": token, "line_token": "line"} for token in DIVIDERS] +
                            [{"token": "untouched", "line_token": "line"}],
            "lane_divider": [{"token": "lane", "lane_divider_segments": [
                {"node_token": "a", "segment_type": "NIL"}]}],
        }

    def test_only_five_untyped_records_change_and_source_is_immutable(self):
        before = copy.deepcopy(self.source)
        result = make_override(self.manifest, self.source)
        self.assertEqual(self.source, before)
        for key in ("node", "line", "lane_divider"):
            self.assertEqual(result[key], before[key])
        for record in result["road_divider"]:
            if record["token"] in DIVIDERS:
                self.assertEqual(record["road_divider_segments"],
                                 [{"node_token": "a", "segment_type": "DOUBLE_SOLID_YELLOW"}])
            else:
                self.assertNotIn("road_divider_segments", record)
        self.assertFalse(result["scene_visual_override"]["style_verified_against_camera_imagery"])

    def test_rejects_other_scenes(self):
        self.manifest["scene"] = "boston-seaport_scene-0103"
        with self.assertRaisesRegex(ValueError, "restricted"):
            make_override(self.manifest, self.source)

    def test_does_not_override_new_source_annotations(self):
        self.source["road_divider"][0]["road_divider_segments"] = [
            {"node_token": "a", "segment_type": "NIL"}]
        with self.assertRaisesRegex(ValueError, "existing source"):
            make_override(self.manifest, self.source)


if __name__ == "__main__":
    unittest.main()
