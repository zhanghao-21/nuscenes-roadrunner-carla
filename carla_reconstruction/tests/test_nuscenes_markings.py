import copy
import importlib.util
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import MagicMock, mock_open, patch

from nusc_carla.nuscenes_markings import (
    audit_road_alignment, build_nuscenes_marking_mesh, generate_nuscenes_marking_obj,
)
from tools.generate_lane_markings import build_parser, run


FIXTURE = Path(__file__).parent / "fixtures" / "markings.xodr"


def map_fixture(points, styles, layer="lane_divider"):
    return {
        "node": [{"token": str(i), "x": x, "y": y} for i, (x, y) in enumerate(points)],
        "line": [{"token": "line", "node_tokens": [str(i) for i in range(len(points))]}],
        layer: [{"token": "divider", "line_token": "line", layer + "_segments": [
            {"node_token": str(i), "segment_type": style} for i, style in enumerate(styles)]}],
    }


def mesh_for(data, **kwargs):
    return build_nuscenes_marking_mesh(data, (0, 0), (-100, -100, 100, 100), **kwargs)


class SourceDividerTests(unittest.TestCase):
    def test_surveyed_coordinates_and_origin_preserved_without_lane_width(self):
        data = map_fixture([(100, 204), (110, 204)], ["SINGLE_SOLID_WHITE"])
        original = copy.deepcopy(data)
        mesh = build_nuscenes_marking_mesh(data, (100, 200), (-1, -1, 20, 20))
        self.assertAlmostEqual(min(p[0] for p in mesh.vertices), 0)
        self.assertAlmostEqual(max(p[0] for p in mesh.vertices), 1000)
        self.assertAlmostEqual(sum(p[1] for p in mesh.vertices) / len(mesh.vertices), 400)
        self.assertTrue(all(p[2] == 2 for p in mesh.vertices))
        self.assertEqual(data, original)

    def test_double_dashes_remain_double_with_no_collapse(self):
        mesh = mesh_for(map_fixture([(0, 0), (20, 0)], ["DOUBLE_DASHED_WHITE"]))
        self.assertEqual(mesh.ribbons, 6)
        self.assertEqual(mesh.collapsed_double_broken_definitions, 0)
        ys = {round(p[1], 3) for p in mesh.vertices}
        self.assertEqual(ys, {-18.5, -5.5, 5.5, 18.5})

    def test_style_changes_and_nil_do_not_become_defaults(self):
        data = map_fixture([(0, 0), (5, 0), (10, 0), (15, 0)],
                           ["SINGLE_SOLID_WHITE", "NIL", "SINGLE_SOLID_YELLOW", "DOUBLE_SOLID_WHITE"])
        mesh = mesh_for(data)
        self.assertEqual(mesh.ribbons, 2)
        self.assertEqual(mesh.metadata["skipped_edges_by_style"], {"NIL": 1})
        white = {i for face in mesh.faces["white"] for i in face}
        yellow = {i for face in mesh.faces["yellow"] for i in face}
        self.assertLessEqual(max(mesh.vertices[i - 1][0] for i in white), 500)
        self.assertGreaterEqual(min(mesh.vertices[i - 1][0] for i in yellow), 1000)

    def test_missing_unknown_and_untyped_road_dividers_are_reported_not_painted(self):
        mesh = mesh_for(map_fixture([(0, 0), (5, 0)], [], "road_divider"))
        self.assertFalse(mesh.vertices)
        self.assertEqual(mesh.metadata["skipped_edges_by_style"], {"missing": 1})
        mesh = mesh_for(map_fixture([(0, 0), (5, 0)], ["UNKNOWN_YELLOW"]))
        self.assertFalse(mesh.vertices)
        self.assertEqual(mesh.metadata["skipped_edges_by_style"], {"UNKNOWN_YELLOW": 1})

    def test_patch_clips_crossing_edge_and_keeps_original_dash_phase(self):
        mesh = build_nuscenes_marking_mesh(
            map_fixture([(-10, 0), (20, 0)], ["SINGLE_DASHED_WHITE"]),
            (0, 0), (0, -5, 10, 5))
        xs = {round(p[0] / 100, 6) for p in mesh.vertices}
        self.assertEqual(mesh.ribbons, 2)
        self.assertEqual(min(xs), 0)
        self.assertEqual(max(xs), 10)
        self.assertTrue(all(x <= 2 or x >= 8 for x in xs))

    def test_dash_phase_does_not_restart_at_each_source_vertex(self):
        mesh = mesh_for(map_fixture([(0, 0), (5, 0), (20, 0)],
                                   ["SINGLE_DASHED_WHITE", "SINGLE_DASHED_WHITE"]))
        xs = {round(p[0] / 100, 6) for p in mesh.vertices}
        self.assertEqual(mesh.ribbons, 3)
        self.assertTrue(all(x <= 3 or 9 <= x <= 12 or x >= 18 for x in xs))

    def test_source_corners_retained_even_with_large_sampling_step(self):
        mesh = mesh_for(map_fixture([(0, 0), (5, 0), (5, 5)],
                                   ["SINGLE_SOLID_WHITE", "SINGLE_SOLID_WHITE"]), step=10)
        self.assertEqual(len(mesh.vertices), 6)
        corner = mesh.vertices[2:4]
        self.assertAlmostEqual(sum(p[0] for p in corner) / 2, 500)
        self.assertAlmostEqual(sum(p[1] for p in corner) / 2, 0)

    def test_exact_duplicates_removed_but_nearby_dividers_retained(self):
        data = map_fixture([(0, 0), (10, 0)], ["SINGLE_SOLID_WHITE"])
        duplicate = copy.deepcopy(data["lane_divider"][0])
        duplicate["token"] = "duplicate"
        data["lane_divider"].append(duplicate)
        data["node"] += [{"token": "2", "x": 0, "y": 0.1}, {"token": "3", "x": 10, "y": 0.1}]
        data["line"].append({"token": "nearby", "node_tokens": ["2", "3"]})
        data["lane_divider"].append({"token": "other", "line_token": "nearby",
                                     "lane_divider_segments": [{"node_token": "2", "segment_type": "SINGLE_SOLID_WHITE"}]})
        mesh = mesh_for(data)
        self.assertEqual(mesh.duplicate_base_segments, 1)
        self.assertEqual(mesh.ribbons, 2)

    def test_zigzag_is_not_flattened_to_straight_solid(self):
        mesh = mesh_for(map_fixture([(0, 0), (10, 0)], ["SINGLE_ZIGZAG_WHITE"]))
        self.assertLess(min(p[1] for p in mesh.vertices), -15)
        self.assertGreater(max(p[1] for p in mesh.vertices), 15)

    def test_invalid_dimensions_and_geometry_fail_before_writing(self):
        data = map_fixture([(0, 0), (10, 0)], ["SINGLE_SOLID_WHITE"])
        for kwargs in ({"dash_length": 0}, {"dash_gap": -1}, {"step": float("nan")},
                       {"double_separation": 0.1}, {"stripe_width": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                mesh_for(data, **kwargs)
        data["node"][0]["x"] = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            mesh_for(data)

    def test_alignment_reports_off_road_paint_without_snapping(self):
        mesh = mesh_for(map_fixture([(2, 8), (10, 8)], ["SINGLE_SOLID_WHITE"]))
        before = list(mesh.vertices)
        audit = audit_road_alignment(mesh, FIXTURE)
        self.assertEqual(audit["outside_fraction"], 1)
        self.assertEqual(mesh.vertices, before)
        aligned = mesh_for(map_fixture([(2, 0), (10, 0)], ["SINGLE_SOLID_WHITE"]))
        self.assertEqual(audit_road_alignment(aligned, FIXTURE)["outside_sample_count"], 0)


class DividerGeneratorIntegrationTests(unittest.TestCase):
    def setUp(self):
        generated = Path(__file__).parents[1] / "generated"
        generated.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="test_source_markings_", dir=str(generated))
        self.root = Path(self.temp.name)
        self.map_path = self.root / "map.json"
        self.map_path.write_text(json.dumps(map_fixture([(2, 1), (10, 1)], ["DOUBLE_DASHED_WHITE"])))
        self.meta = self.root / "meta.json"
        self.meta.write_text(json.dumps({"scene": "scene-test", "map": "map-test",
                                       "origin": {"x": 0, "y": 0},
                                       "patch": {"x_min": 0, "y_min": -10, "x_max": 20, "y_max": 10}}))
        self.manifest = self.root / "manifest.json"
        self.manifest.write_text(json.dumps({"scene": "map-test_scene-test",
            "nuscenes_scene": "scene-test", "nuscenes_map": "map-test",
            "source": {"meta": str(self.meta), "xodr": str(FIXTURE)}}))
        self.output = self.root / "marks.obj"

    def tearDown(self):
        self.temp.cleanup()

    def test_cli_default_uses_source_data_without_modifying_xodr(self):
        before = FIXTURE.read_bytes()
        args = build_parser().parse_args(["--manifest", str(self.manifest), "--output", str(self.output),
                                          "--map-json", str(self.map_path)])
        with redirect_stdout(io.StringIO()):
            stats = run(args)
        self.assertEqual(stats["source_kind"], "nuscenes")
        self.assertEqual(stats["double_broken_definitions_collapsed"], 0)
        self.assertEqual(stats["invented_boundary_markings"], 0)
        self.assertEqual(stats["road_alignment"]["outside_sample_count"], 0)
        self.assertLess(stats["expected_unreal_bounds_cm"]["max"][1], 0)
        self.assertNotIn("source_road_mark_definitions", stats)
        self.assertEqual(FIXTURE.read_bytes(), before)
        self.assertIn("Generated from nuscenes", self.output.read_text())
        self.assertEqual(json.loads(self.output.with_suffix(".json").read_text()), stats)

    def test_missing_map_does_not_silently_fall_back_or_overwrite(self):
        self.output.write_text("previous overlay")
        with self.assertRaisesRegex(ValueError, "map source missing"):
            generate_nuscenes_marking_obj(self.manifest, self.output)
        self.assertEqual(self.output.read_text(), "previous overlay")

    def test_explicit_legacy_source_remains_available(self):
        args = build_parser().parse_args(["--manifest", str(self.manifest), "--output", str(self.output),
                                          "--source", "opendrive"])
        with redirect_stdout(io.StringIO()):
            stats = run(args)
        self.assertEqual(stats["source_kind"], "opendrive")
        self.assertIn("source_road_mark_definitions", stats)

    def test_empty_typed_source_is_explicit_and_can_remove_stale_overlay(self):
        self.map_path.write_text(json.dumps(map_fixture([(2, 1), (10, 1)], [])))
        stats = generate_nuscenes_marking_obj(self.manifest, self.output, self.map_path)
        self.assertEqual(stats["vertices"], 0)
        self.assertEqual(stats["source_kind"], "nuscenes")
        self.assertNotIn("expected_unreal_bounds_cm", stats)

    def test_nonflat_map_rejected_before_replacing_artifacts(self):
        xodr = self.root / "nonflat.xodr"
        xodr.write_text(FIXTURE.read_text().replace("<lanes>",
            '<elevationProfile><elevation s="0" a="1" b="0" c="0" d="0"/></elevationProfile><lanes>'))
        manifest = json.loads(self.manifest.read_text())
        manifest["source"]["xodr"] = str(xodr)
        self.manifest.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(ValueError, "flat OpenDRIVE"):
            generate_nuscenes_marking_obj(self.manifest, self.output, self.map_path)
        self.assertFalse(self.output.exists())


class MarkingsOnlyEditorTests(unittest.TestCase):
    def setUp(self):
        self.unreal = MagicMock()
        path = Path(__file__).parents[1] / "unreal" / "build_environment.py"
        spec = importlib.util.spec_from_file_location("test_environment_build", str(path))
        self.build = importlib.util.module_from_spec(spec)
        with patch.dict("sys.modules", {"unreal": self.unreal}):
            spec.loader.exec_module(self.build)

    def test_only_owned_marking_actors_are_removed(self):
        actors = []
        for label in ("NSRC_LaneMarkings_OpenDRIVE", "NSRC_LaneMarkings_Source",
                      "NSRC_Bldg1", "NSRC_Static_1", "UserRoadMarking"):
            actor = MagicMock()
            actor.get_actor_label.return_value = label
            actors.append(actor)
        self.unreal.EditorLevelLibrary.get_all_level_actors.return_value = actors
        self.build.delete_previous(markings_only=True)
        self.assertEqual(self.unreal.EditorLevelLibrary.destroy_actor.call_args_list,
                         [unittest.mock.call(actors[0]), unittest.mock.call(actors[1])])

    def _run_main(self, changed=False, empty=False):
        b = self.build
        b.required_env = lambda key: {"NUSC_CARLA_MANIFEST": "manifest.json",
                                     "NUSC_CARLA_ASSET_CATALOG": "catalog.json",
                                     "NUSC_CARLA_SOURCE_LEVEL": "/Game/Base",
                                     "NUSC_CARLA_TARGET_LEVEL": "/Game/Decorated",
                                     "NUSC_CARLA_MARKINGS_OBJ": "marks.obj",
                                     "NUSC_CARLA_RESULT": "result.json"}[key]
        b.read_json = MagicMock(side_effect=[{"scene": "test"}, {},
                                            {"vertices": 0 if empty else 20, "source_kind": "nuscenes"}])
        for name in ("prepare_level", "delete_previous", "import_lane_marking_asset",
                     "add_lane_markings", "apply_road_surface_material", "add_buildings",
                     "add_trees", "add_signs", "add_nuscenes_static_objects", "add_traffic_lights"):
            setattr(b, name, MagicMock(return_value=0 if empty else 1))
        b.unrelated_actor_fingerprint = MagicMock(side_effect=[{"sha256": "same"},
                                                              {"sha256": "different" if changed else "same"}])
        with patch.dict(b.os.environ, {"NUSC_CARLA_MARKINGS_ONLY": "1"}), patch("builtins.open", mock_open()):
            b.main()

    def test_markings_only_main_does_not_rebuild_environment(self):
        self._run_main()
        self.build.delete_previous.assert_called_once_with(markings_only=True)
        for name in ("apply_road_surface_material", "add_buildings", "add_trees",
                     "add_signs", "add_nuscenes_static_objects", "add_traffic_lights"):
            getattr(self.build, name).assert_not_called()
        self.unreal.EditorLevelLibrary.save_current_level.assert_called_once()

    def test_empty_overlay_does_not_attempt_obj_import(self):
        self._run_main(empty=True)
        self.build.import_lane_marking_asset.assert_not_called()
        self.build.delete_previous.assert_called_once_with(markings_only=True)
        self.unreal.EditorLevelLibrary.save_current_level.assert_called_once()

    def test_unrelated_actor_change_aborts_before_level_save(self):
        with self.assertRaisesRegex(RuntimeError, "unrelated actor"):
            self._run_main(changed=True)
        self.unreal.EditorLevelLibrary.save_current_level.assert_not_called()


if __name__ == "__main__":
    unittest.main()
