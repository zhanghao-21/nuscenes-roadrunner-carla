import copy
import math
import unittest

from tools.prepare_0103_marking_override import WHITE_DIVIDERS, YELLOW_DIVIDERS, make_override
from nusc_carla.nuscenes_markings import build_nuscenes_marking_mesh


class Boston0103OverrideTests(unittest.TestCase):
    def setUp(self):
        self.manifest = {"scene": "boston-seaport_scene-0103", "nuscenes_map": "boston-seaport",
                         "map": {"asset_name": "Nusc_boston_seaport_0103"}}
        self.source = {key: [] for key in ("node", "line", "polygon", "lane", "lane_divider", "road_divider")}
        self.source["arcline_path_3"] = {}
        for i, token in enumerate(WHITE_DIVIDERS + YELLOW_DIVIDERS):
            y = 10 * i
            a, b, c, d = [token + suffix for suffix in ("a", "b", "c", "d")]
            for name, x, yy in ((a, 0, y), (b, 20, y), (c, 20, y + 3), (d, 0, y - 3)):
                self.source["node"].append({"token": name, "x": x, "y": yy})
            self.source["line"].append({"token": token, "node_tokens": [a, b]})
            white = token in WHITE_DIVIDERS
            layer = "lane_divider" if white else "road_divider"
            record = {"token": token, "line_token": token}
            if white:
                record["lane_divider_segments"] = [{"node_token": n, "segment_type": "NIL"} for n in (a, b)]
            self.source[layer].append(record)
            for side, vertices in enumerate(([a, b, c], [b, a, d])):
                lane = token + str(side)
                self.source["polygon"].append({"token": lane, "exterior_node_tokens": vertices})
                self.source["lane"].append({"token": lane, "polygon_token": lane})
                heading = 0 if white or side == 0 else math.pi
                self.source["arcline_path_3"][lane] = [
                    {"start_pose": [0, y, heading], "end_pose": [20, y, heading]}]
        self.source["lane_divider"].append({
            "token": "untouched", "line_token": WHITE_DIVIDERS[0],
            "lane_divider_segments": [{"node_token": WHITE_DIVIDERS[0] + "b", "segment_type": "NIL"}]})

    def test_scoped_changes_preserve_geometry_and_source(self):
        before = copy.deepcopy(self.source)
        result = make_override(self.manifest, self.source)
        self.assertEqual(self.source, before)
        for key in ("node", "line", "polygon", "lane", "arcline_path_3"):
            self.assertEqual(result[key], before[key])
        self.assertEqual(result["lane_divider"][-1], before["lane_divider"][-1])
        for record in result["lane_divider"][:-1]:
            self.assertEqual([a["segment_type"] for a in record["lane_divider_segments"]],
                             ["SINGLE_DASHED_WHITE", "NIL"])
        for record in result["road_divider"]:
            self.assertEqual(record["road_divider_segments"][0]["segment_type"], "DOUBLE_SOLID_YELLOW")
        self.assertEqual(len(result["scene_visual_override"]["changes"]), 15)
        self.assertFalse(result["scene_visual_override"]["style_verified_against_camera_imagery"])

    def test_rejects_wrong_scene_map_or_asset(self):
        for key, value in (("scene", "boston-seaport_scene-0757"), ("nuscenes_map", "singapore-hollandvillage"),
                           ("map", {"asset_name": "Nusc_boston_seaport_0757"})):
            manifest = dict(self.manifest, **{key: value})
            with self.assertRaisesRegex(ValueError, "restricted"):
                make_override(manifest, self.source)

    def test_refuses_to_replace_painted_lane_edges(self):
        self.source["lane_divider"][0]["lane_divider_segments"][0]["segment_type"] = "SINGLE_SOLID_WHITE"
        with self.assertRaisesRegex(ValueError, "source lane paint"):
            make_override(self.manifest, self.source)

    def test_refuses_to_replace_road_annotations(self):
        self.source["road_divider"][0]["road_divider_segments"] = [{"segment_type": "NIL"}]
        with self.assertRaisesRegex(ValueError, "source road paint"):
            make_override(self.manifest, self.source)

    def test_rejects_wrong_traffic_direction_for_each_color(self):
        for token, wrong_heading in ((WHITE_DIVIDERS[0], math.pi), (YELLOW_DIVIDERS[0], 0)):
            source = copy.deepcopy(self.source)
            source["arcline_path_3"][token + "1"][0]["start_pose"][2] = wrong_heading
            with self.assertRaisesRegex(ValueError, "direction"):
                make_override(self.manifest, source)

    def test_rejects_road_edge_or_connector_boundary(self):
        self.source["lane_connector"] = [self.source["lane"].pop(1)]
        with self.assertRaisesRegex(ValueError, "exactly two non-connector"):
            make_override(self.manifest, self.source)

    def test_rejects_cross_lane_end_caps(self):
        for side in ("0", "1"):
            arc = self.source["arcline_path_3"][WHITE_DIVIDERS[0] + side][0]
            arc["start_pose"][2] = arc["end_pose"][2] = math.pi / 2
        with self.assertRaisesRegex(ValueError, "longitudinal"):
            make_override(self.manifest, self.source)

    def test_rejects_missing_expected_record(self):
        self.source["lane_divider"].pop(0)
        with self.assertRaisesRegex(ValueError, "missing"):
            make_override(self.manifest, self.source)

    def test_new_mesh_preserves_existing_solid_geometry(self):
        self.source["node"].extend([{"token": "s0", "x": 0, "y": -5}, {"token": "s1", "x": 20, "y": -5}])
        self.source["line"].append({"token": "solid", "node_tokens": ["s0", "s1"]})
        self.source["lane_divider"].append({"token": "solid", "line_token": "solid", "lane_divider_segments": [
            {"node_token": "s0", "segment_type": "SINGLE_SOLID_WHITE"}]})
        bounds = (-1, -6, 21, 151)
        before = build_nuscenes_marking_mesh(self.source, (0, 0), bounds)
        after = build_nuscenes_marking_mesh(make_override(self.manifest, self.source), (0, 0), bounds)
        self.assertTrue(set(before.vertices).issubset(set(after.vertices)))
        self.assertEqual(before.vertices, [v for v in after.vertices if v[1] < -400])
        self.assertGreater(len(after.faces["white"]), len(before.faces["white"]))
        self.assertGreater(len(after.faces["yellow"]), 0)
        self.assertEqual(after.metadata["exact_duplicate_style_conflicts"], [])


if __name__ == "__main__":
    unittest.main()
