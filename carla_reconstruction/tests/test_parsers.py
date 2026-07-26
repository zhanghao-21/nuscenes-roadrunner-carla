import os
import unittest

from nusc_carla.nuscenes_static import extract_static_objects
from nusc_carla.osm import read_osm
from nusc_carla.xodr import read_xodr


FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


class ParserTests(unittest.TestCase):
    def test_xodr_line_and_signal(self):
        result = read_xodr(os.path.join(FIXTURES, "map.xodr"), sample_spacing=5)
        self.assertEqual(len(result["roads"]), 1)
        self.assertEqual(result["roads"][0]["width"], 4.0)
        self.assertAlmostEqual(result["signals"][0]["x"], 10.0)
        self.assertAlmostEqual(result["signals"][0]["y"], -3.0)

    def test_osm_supported_objects(self):
        result = read_osm(os.path.join(FIXTURES, "map.osm"),
                          "boston-seaport", {"x": 0.0, "y": 0.0})
        self.assertEqual(len(result["trees"]), 1)
        self.assertEqual(result["signs"][0]["kind"], "stop")
        self.assertEqual(len(result["green_polygons"]), 1)

    def test_nuscenes_static_objects_are_deduplicated(self):
        result = extract_static_objects(
            os.path.join(FIXTURES, "nuscenes"), "scene-test",
            {"x": 100.0, "y": 200.0},
            {"x_min": 100.0, "y_min": 200.0,
             "x_max": 150.0, "y_max": 250.0})
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["category"], "movable_object.trafficcone")
        self.assertAlmostEqual(result[0]["x"], 10.0)
        self.assertAlmostEqual(result[0]["y"], 20.0)


if __name__ == "__main__":
    unittest.main()
