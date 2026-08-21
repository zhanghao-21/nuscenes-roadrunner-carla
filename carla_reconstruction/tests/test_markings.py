import unittest
from pathlib import Path

from nusc_carla.markings import build_marking_mesh, generate_marking_obj


FIXTURE = Path(__file__).parent / "fixtures" / "markings.xodr"
DUPLICATE_FIXTURE = Path(__file__).parent / "fixtures" / "markings_duplicates.xodr"


class MarkingMeshTests(unittest.TestCase):
    def test_boundaries_styles_and_coordinate_flip(self):
        mesh = build_marking_mesh(FIXTURE, step=1.0)
        self.assertEqual(mesh.definitions, 2)
        self.assertGreater(len(mesh.faces["white"]), 0)
        self.assertGreater(len(mesh.faces["yellow"]), 0)
        ys = [vertex[1] for vertex in mesh.vertices]
        # OBJ output stays in OpenDRIVE axes: center boundary t=+2 is
        # Y=+200 cm and the right edge t=-2 is Y=-200 cm. Unreal applies the
        # Y inversion during import, exactly as it does for RoadRunner FBX.
        self.assertLess(min(ys), -190.0)
        self.assertGreater(max(ys), 190.0)
        white_indices = {index for face in mesh.faces["white"] for index in face}
        yellow_indices = {index for face in mesh.faces["yellow"] for index in face}
        self.assertTrue(all(mesh.vertices[index - 1][1] > 190.0
                            for index in white_indices))
        self.assertTrue(all(mesh.vertices[index - 1][1] < -180.0
                            for index in yellow_indices))

    def test_writes_obj_materials_and_stats(self):
        output = Path(__file__).parents[1] / "generated" / "test_markings.obj"
        siblings = [output, output.with_suffix(".mtl"), output.with_suffix(".json")]
        try:
            stats = generate_marking_obj(FIXTURE, output)
            text = output.read_text(encoding="ascii")
            self.assertIn("usemtl LaneMarkingWhite", text)
            self.assertIn("usemtl LaneMarkingYellow", text)
            self.assertGreater(stats["faces"], 0)
            self.assertTrue(output.with_suffix(".mtl").is_file())
            self.assertTrue(output.with_suffix(".json").is_file())
        finally:
            for path in siblings:
                if path.exists():
                    path.unlink()

    def test_shared_boundary_is_deduplicated_but_double_line_is_retained(self):
        mesh = build_marking_mesh(
            DUPLICATE_FIXTURE, step=1.0, preserve_double_broken=True)
        self.assertEqual(mesh.source_definitions, 3)
        self.assertEqual(mesh.definitions, 2)
        self.assertGreater(mesh.duplicate_base_segments, 0)
        # One broken line creates 3 dashes; the genuine broken-broken line
        # creates 6. The coincident second boundary creates none.
        self.assertEqual(mesh.ribbons, 9)

    def test_visual_default_collapses_double_dashed_to_one_stripe(self):
        mesh = build_marking_mesh(DUPLICATE_FIXTURE, step=1.0)
        self.assertEqual(mesh.definitions, 2)
        self.assertEqual(mesh.collapsed_double_broken_definitions, 1)
        # Both surviving definitions are visually single broken lines with
        # three dashes each.
        self.assertEqual(mesh.ribbons, 6)


if __name__ == "__main__":
    unittest.main()
