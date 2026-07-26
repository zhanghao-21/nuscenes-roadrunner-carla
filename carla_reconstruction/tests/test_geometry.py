import unittest

from nusc_carla.geometry import (point_in_oriented_box, point_in_polygon,
                                 sample_polygon_grid, stable_seed)


class GeometryTests(unittest.TestCase):
    def test_point_in_polygon(self):
        square = [(0, 0), (10, 0), (10, 10), (0, 10)]
        self.assertTrue(point_in_polygon((5, 5), square))
        self.assertTrue(point_in_polygon((0, 5), square))
        self.assertFalse(point_in_polygon((12, 5), square))

    def test_oriented_box(self):
        box = {"x": 3.0, "y": 4.0, "half_x": 2.0, "half_y": 1.0,
               "yaw_rad": 0.0}
        self.assertTrue(point_in_oriented_box((4.9, 4.9), box))
        self.assertFalse(point_in_oriented_box((5.1, 5.1), box))

    def test_sampling_is_deterministic(self):
        polygon = [(0, 0), (30, 0), (30, 30), (0, 30)]
        self.assertEqual(sample_polygon_grid(polygon, 10, "a"),
                         sample_polygon_grid(polygon, 10, "a"))
        self.assertEqual(stable_seed("scene"), stable_seed("scene"))


if __name__ == "__main__":
    unittest.main()
