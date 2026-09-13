import unittest

from scene_overrides.refresh_0103_editor_routes import (
    display_points, match_cached_roads, missing_connectors, unique_actors)


class Scene0103EditorRouteTests(unittest.TestCase):
    def setUp(self):
        self.connectors = [{"road_id": 51, "start_cm": [100, 200], "end_cm": [900, 800]},
                           {"road_id": 57, "start_cm": [100, 200], "end_cm": [700, 50]}]

    def test_stale_editor_route_does_not_count_as_connector(self):
        routes = [{"routes_cm": [[[10, 20, 100], [100, 200, 100]]]}]
        self.assertEqual(missing_connectors(routes, self.connectors), [51, 57])

    def test_requires_both_branches_not_only_shared_entry(self):
        routes = [{"routes_cm": [[[100, 200, 100], [900, 800, 100]]]}]
        self.assertEqual(missing_connectors(routes, self.connectors), [57])

    def test_matching_endpoints_allow_native_sampling_epsilon_and_trigger_height(self):
        routes = [{"routes_cm": [[[100.1, 200, 100], [900, 799.9, 100]],
                                 [[100, 200, 500], [700, 50, 500]]]}]
        self.assertEqual(missing_connectors(routes, self.connectors), [])

    def test_wrong_direction_and_empty_splines_do_not_pass(self):
        routes = [{"routes_cm": [[], [[900, 800, 100], [100, 200, 100]]]}]
        self.assertEqual(missing_connectors(routes, self.connectors), [51, 57])

    def test_connector_can_have_short_bridge_before_and_after(self):
        routes = [{"routes_cm": [[[90, 190, 100], [100, 200, 100], [900, 800, 100], [901, 801, 100]]]}]
        self.assertEqual(missing_connectors(routes, self.connectors), [57])

    def test_cached_road_matching_refuses_ambiguous_starts(self):
        records = [{"routes_cm": [[[100, 200, 100], [200, 300, 100]]]}]
        starts = [{"road_id": 1, "start_cm": [100, 200]}]
        self.assertEqual(match_cached_roads(records, starts)[1], records[0]["routes_cm"][0])
        with self.assertRaisesRegex(ValueError, "Ambiguous"):
            match_cached_roads(records, starts + [{"road_id": 2, "start_cm": [100, 200]}])

    def test_bridges_preserve_existing_cached_geometry(self):
        connector = {"incoming_road": 1, "outgoing_road": 2, "points_cm": [[200, 0], [900, 0]]}
        cached = {1: [[0, 0, 100], [100, 0, 100]], 2: [[900.1, 0, 100], [1000, 0, 100]]}
        points = display_points(connector, cached, 100)
        self.assertEqual(points, [[100, 0, 100], [200, 0, 100], [900, 0, 100], [900.1, 0, 100]])
        self.assertEqual(cached[1], [[0, 0, 100], [100, 0, 100]])

    def test_refuses_long_invented_bridge(self):
        connector = {"incoming_road": 1, "outgoing_road": 2, "points_cm": [[500, 0], [900, 0]]}
        with self.assertRaisesRegex(ValueError, "gap"):
            display_points(connector, {1: [[0, 0, 100], [100, 0, 100]]}, 100)

    def test_duplicate_native_planner_references_are_counted_once(self):
        class Actor:
            def __init__(self, path):
                self.path = path

            def get_path_name(self):
                return self.path

        first, second = Actor("one"), Actor("two")
        self.assertEqual(unique_actors([first, None, second, first]), [first, second])


if __name__ == "__main__":
    unittest.main()
