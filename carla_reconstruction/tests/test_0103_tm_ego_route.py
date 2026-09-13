"""Guard the opt-in-by-scene ego route without weakening TM safety settings."""

import copy
import types
import unittest
from unittest import mock

from tests import test_carla_perturbation_scopes as scopes
from runtime import run_tm_closed_loop as runner
from scene_overrides import tm_ego_route_0103 as override


def fake_waypoint(road, lane=-1, distance=0.0, xyz=(0.0, 0.0, 0.0), junction=False):
    return types.SimpleNamespace(
        road_id=road, lane_id=lane, s=distance, lane_type="Driving",
        is_junction=junction,
        transform=types.SimpleNamespace(location=types.SimpleNamespace(
            x=xyz[0], y=xyz[1], z=xyz[2])),
        next=mock.Mock(return_value=[]))


class Scene0103EgoRouteValidationTests(unittest.TestCase):
    def setUp(self):
        self.track = scopes.track("ego", [0.0, 5.0])
        self.manifest = {"scene": override.SCENE_ID}
        self.anchors = [fake_waypoint(*spec, xyz=xyz) for spec, xyz in zip(
            override.ANCHOR_SPECS, override.EXPECTED_ANCHOR_XYZ)]
        self.nodes = [self.anchors[0], fake_waypoint(17, distance=156),
                      fake_waypoint(69, junction=True), self.anchors[1],
                      fake_waypoint(56, junction=True), self.anchors[2],
                      fake_waypoint(80, junction=True), self.anchors[3], self.anchors[4]]
        for left, right in zip(self.nodes, self.nodes[1:]):
            left.next.return_value = [right]
        mapping = dict(zip(override.ANCHOR_SPECS, self.anchors))
        self.carla_map = mock.Mock()
        self.carla_map.name = "/Game/" + override.EXPECTED_MAP_NAME
        self.carla_map.get_waypoint_xodr.side_effect = lambda *spec: mapping.get(spec)
        self.hash_patch = mock.patch.object(
            override, "EXPECTED_TRACK_SHA256", override._track_fingerprint(self.track))
        self.hash_patch.start()
        self.addCleanup(self.hash_patch.stop)

    def prepare(self, **kwargs):
        return override.prepare_ego_route(
            self.manifest, self.track, self.carla_map, "tm", **kwargs)

    def test_valid_corridor_returns_five_native_locations_and_unapplied_metadata(self):
        for map_name in (override.EXPECTED_MAP_NAME,
                         "/Game/" + override.EXPECTED_MAP_NAME):
            with self.subTest(map_name=map_name):
                self.carla_map.name = map_name
                result = self.prepare()
                self.assertIsInstance(result, override.PreparedEgoRoute)
                self.assertEqual(result.locations, tuple(
                    waypoint.transform.location for waypoint in self.anchors))
                self.assertEqual(result.metadata["road_sequence"], list(override.ROAD_SEQUENCE))
                self.assertEqual(result.metadata["override_id"], override.OVERRIDE_ID)
                self.assertFalse(result.metadata["applied"])
                self.assertEqual(len(result.metadata["anchors"]), 5)
                self.assertEqual(result.metadata["end_policy"],
                                 "native_tm_continuation_after_final_anchor")
        for waypoint in self.nodes[:-1]:
            waypoint.next.assert_called_with(1.0)

    def test_other_scenes_modes_and_explicit_disable_are_no_ops(self):
        # An unusable map/track also proves irrelevant requests return before inspection.
        for scene, mode, disabled in (("scene-other", "tm", False),
                                      (override.SCENE_ID, "replay", False),
                                      (override.SCENE_ID, "external", False),
                                      (override.SCENE_ID, "tm", True)):
            with self.subTest(scene=scene, mode=mode, disabled=disabled):
                self.assertIsNone(override.prepare_ego_route(
                    {"scene": scene}, None, None, mode, disabled=disabled))
        self.carla_map.get_waypoint_xodr.assert_not_called()

    def test_actual_loaded_map_must_match_exact_decorated_package(self):
        for name in ("OtherMap", override.EXPECTED_MAP_NAME.replace("_Decorated", ""),
                     "OtherPackage/" + override.EXPECTED_MAP_NAME):
            with self.subTest(name=name):
                self.carla_map.name = name
                with self.assertRaisesRegex(ValueError, "--disable-0103-ego-route"):
                    self.prepare()
        self.carla_map.get_waypoint_xodr.assert_not_called()

    def test_changed_ego_identity_or_route_is_rejected(self):
        for attribute, value in (("is_ego", False), ("actor_id", "replacement"),
                                 ("points", scopes.track("ego", [0, 6]).points)):
            with self.subTest(attribute=attribute):
                original = getattr(self.track, attribute)
                setattr(self.track, attribute, value)
                with self.assertRaisesRegex(ValueError, "recorded ego track differs"):
                    self.prepare()
                setattr(self.track, attribute, original)
        self.carla_map.get_waypoint_xodr.assert_not_called()

    def test_fingerprint_ignores_serialization_noise_but_detects_pose_changes(self):
        fingerprint = override._track_fingerprint(self.track)
        self.assertEqual(fingerprint, override._track_fingerprint(copy.deepcopy(self.track)))
        self.assertEqual(fingerprint, override._track_fingerprint(
            scopes.track("ego", [0, 5])))
        self.assertEqual(fingerprint, override._track_fingerprint(
            scopes.track("ego", [0.0, 5.0000001])))
        self.assertNotEqual(fingerprint, override._track_fingerprint(
            scopes.track("ego", [0.0, 5.1])))
        changed = copy.deepcopy(self.track)
        from dataclasses import replace
        changed.points[1] = replace(changed.points[1], yaw=0.1)
        self.assertNotEqual(fingerprint, override._track_fingerprint(changed))

    def test_missing_wrong_lane_junction_or_non_driving_anchor_is_rejected(self):
        self.carla_map.get_waypoint_xodr.side_effect = None
        for wrong in (None, fake_waypoint(99, distance=155),
                      fake_waypoint(17, lane=1, distance=155),
                      fake_waypoint(17, distance=155, junction=True),
                      fake_waypoint(17, distance=154),
                      fake_waypoint(17, distance=float("nan"))):
            with self.subTest(waypoint=wrong):
                self.carla_map.get_waypoint_xodr.return_value = wrong
                with self.assertRaisesRegex(ValueError, "incompatible non-junction anchor"):
                    self.prepare()
        self.anchors[0].lane_type = "Sidewalk"
        self.carla_map.get_waypoint_xodr.return_value = self.anchors[0]
        with self.assertRaisesRegex(ValueError, "incompatible non-junction anchor"):
            self.prepare()

    def test_shifted_and_nonfinite_geometry_are_rejected_with_small_tolerance(self):
        location = self.anchors[0].transform.location
        expected_x = location.x
        location.x = expected_x + 0.24
        self.assertIsNotNone(self.prepare())
        for value in (expected_x + 0.251, float("nan"), float("inf")):
            with self.subTest(value=value):
                location.x = value
                with self.assertRaisesRegex(ValueError, "anchor geometry changed"):
                    self.prepare()

    def test_corridor_rejects_wrong_connector_lane_or_driving_type(self):
        connector = self.nodes[4]
        for attribute, value in (("road_id", 53), ("lane_id", 1),
                                 ("lane_type", "Shoulder")):
            with self.subTest(attribute=attribute):
                original = getattr(connector, attribute)
                setattr(connector, attribute, value)
                with self.assertRaisesRegex(ValueError, "does not connect roads"):
                    self.prepare()
                setattr(connector, attribute, original)

    def test_disconnected_or_backward_same_road_path_is_rejected(self):
        first_successors = self.nodes[0].next.return_value
        for successors in ([], [self.nodes[0]], [fake_waypoint(17, distance=154)]):
            with self.subTest(successors=successors):
                self.nodes[0].next.return_value = successors
                with self.assertRaisesRegex(ValueError, "does not connect roads"):
                    self.prepare()
        self.nodes[0].next.return_value = first_successors
        self.nodes[-2].next.return_value = []  # Must reach the final anchor too.
        with self.assertRaisesRegex(ValueError, "does not connect roads"):
            self.prepare()

    def test_valid_branch_is_found_without_following_unrelated_turn(self):
        wrong_turn = fake_waypoint(53, junction=True)
        self.nodes[3].next.return_value.append(wrong_turn)
        self.assertIsNotNone(self.prepare())
        wrong_turn.next.assert_not_called()


class Scene0103EgoRouteRuntimeTests(unittest.TestCase):
    def prepared(self):
        return types.SimpleNamespace(
            locations=(object(), object()),
            metadata={"override_id": "scene0103-test", "applied": False,
                      "anchors": [], "road_sequence": [17, 69, 9, 56, 12, 80, 11],
                      "end_policy": "normal Traffic Manager continuation"})

    def test_disable_flag_defaults_false_and_can_be_requested(self):
        parser = runner.build_parser()
        self.assertFalse(parser.parse_args([]).disable_0103_ego_route)
        self.assertTrue(parser.parse_args(
            ["--disable-0103-ego-route"]).disable_0103_ego_route)

    def test_validation_failure_precedes_settings_changes_and_spawning(self):
        population = scopes.bundle()
        world, client = mock.Mock(), mock.Mock()
        client.load_world.return_value = world
        args = runner.build_parser().parse_args(["--manifest", "manifest.json"])
        with mock.patch.object(runner, "load_manifest_tracks", return_value=(
                scopes.MANIFEST, population)), \
                mock.patch.object(runner.carla, "Client", return_value=client, create=True), \
                mock.patch.object(runner, "prepare_ego_route", side_effect=ValueError(
                    "changed map --disable-0103-ego-route")) as prepare, \
                mock.patch.object(runner, "spawn_track_actor") as spawn:
            with self.assertRaisesRegex(ValueError, "changed map"):
                runner.run(args)
        world.get_map.assert_called_once()
        prepare.assert_called_once()
        world.get_settings.assert_not_called()
        world.apply_settings.assert_not_called()
        client.get_trafficmanager.assert_not_called()
        spawn.assert_not_called()

    def test_only_ego_path_changes_and_each_variant_keeps_its_behavior_and_safety(self):
        for target in ("ego", "surrounding"):
            with self.subTest(target=target):
                doc = scopes.generate(target)["variant_000.json"]
                route = self.prepared()
                original_doc = copy.deepcopy(doc)
                with mock.patch.object(runner, "prepare_ego_route", return_value=route):
                    actors, tm, _, metadata, _, _ = scopes.CarlaScopeRuntimeTests()._run(doc)
                self.assertEqual(doc, original_doc)
                replacement = mock.call(actors["ego"], list(route.locations), True)
                self.assertEqual(tm.set_path.call_args_list.count(replacement), 1)
                self.assertEqual(tm.set_path.call_count, 4)  # 3 normal + ego override.
                for actor_id in ("ego", "moving", "later"):
                    expected = scopes.carla_behavior_for_track(doc, actor_id)
                    tm.set_desired_speed.assert_any_call(
                        actors[actor_id], scopes.expected_speed(expected))
                    for method in ("ignore_vehicles_percentage", "ignore_walkers_percentage",
                                   "ignore_lights_percentage", "ignore_signs_percentage"):
                        getattr(tm, method).assert_any_call(actors[actor_id], expected.get(method, 0.0))
                self.assertEqual(tm.ignore_vehicles_percentage.call_count, 3)
                self.assertEqual(tm.ignore_lights_percentage.call_count, 3)
                self.assertTrue(metadata["scene_ego_route"]["applied"])
                self.assertEqual(metadata["scene_ego_route"]["override_id"], "scene0103-test")
                # The replacement follows all ordinary ego behavior setup.
                replacement_index = tm.method_calls.index(
                    mock.call.set_path(actors["ego"], list(route.locations), True))
                safety_index = tm.method_calls.index(
                    mock.call.ignore_signs_percentage(actors["ego"],
                        scopes.carla_behavior_for_track(doc, "ego")["ignore_signs_percentage"]))
                self.assertGreater(replacement_index, safety_index)

    def test_non_applicable_scene_preserves_all_original_paths(self):
        doc = scopes.generate("ego")["baseline.json"]
        with mock.patch.object(runner, "prepare_ego_route", return_value=None) as prepare:
            actors, tm, _, metadata, world, _ = scopes.CarlaScopeRuntimeTests()._run(doc)
        self.assertEqual(tm.set_path.call_count, 3)
        self.assertFalse(any(len(call.args) == 3 for call in tm.set_path.call_args_list))
        self.assertIn("scene_ego_route", metadata)
        self.assertEqual(prepare.call_args.args[0], scopes.MANIFEST)
        self.assertIs(prepare.call_args.args[2], world.get_map.return_value)

    def test_replay_mode_never_replaces_the_ego_path(self):
        doc = scopes.generate("ego")["baseline.json"]
        with mock.patch.object(runner, "_resolve_ego_mode", return_value="replay"), \
                mock.patch.object(runner, "prepare_ego_route", return_value=None) as prepare:
            actors, tm, _, metadata, _, replayed = scopes.CarlaScopeRuntimeTests()._run(doc)
        self.assertEqual(prepare.call_args.args[3], "replay")
        self.assertEqual(tm.set_path.call_count, 2)
        self.assertFalse(any(call.args[0] is actors["ego"]
                             for call in tm.set_path.call_args_list))
        self.assertIn(actors["ego"].id, replayed)
        self.assertIsNone(metadata["scene_ego_route"])

    def test_runtime_forwards_disable_flag_without_a_replacement(self):
        build_parser = runner.build_parser

        def disabled_parser():
            parser = build_parser()
            parser.set_defaults(disable_0103_ego_route=True)
            return parser

        doc = scopes.generate("ego")["baseline.json"]
        with mock.patch.object(runner, "build_parser", side_effect=disabled_parser), \
                mock.patch.object(runner, "prepare_ego_route", return_value=None) as prepare:
            _, tm, _, metadata, _, _ = scopes.CarlaScopeRuntimeTests()._run(doc)
        self.assertTrue(prepare.call_args.kwargs["disabled"])
        self.assertEqual(tm.set_path.call_count, 3)
        self.assertIsNone(metadata["scene_ego_route"])


if __name__ == "__main__":
    unittest.main()
