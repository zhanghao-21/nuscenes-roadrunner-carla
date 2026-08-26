import sys
import types
import unittest
from unittest import mock


from closed_loop.controllers import ReferencePathController, endpoint_progress


class _FakeControl:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class _FakeVehicle:
    def __init__(self, x, y=0.0, speed=0.0):
        self.id = 1
        self.is_alive = True
        self.bounding_box = types.SimpleNamespace(
            extent=types.SimpleNamespace(x=2.0, y=1.0))
        self._transform = types.SimpleNamespace(
            location=types.SimpleNamespace(x=x, y=y, z=0.0),
            rotation=types.SimpleNamespace(yaw=0.0))
        self._velocity = types.SimpleNamespace(x=speed, y=0.0, z=0.0)

    def get_transform(self):
        return self._transform

    def get_location(self):
        return self._transform.location

    def get_velocity(self):
        return self._velocity


class ReferenceControllerEndpointTests(unittest.TestCase):
    def setUp(self):
        self.controller = ReferencePathController(
            [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)],
            desired_speed=5.0)
        self.fake_carla = types.ModuleType("carla")
        self.fake_carla.VehicleControl = _FakeControl

    def _terminal_step(self, vehicle):
        with mock.patch.dict(sys.modules, {"carla": self.fake_carla}):
            return self.controller.run_step_to_endpoint(
                vehicle, [], elapsed=3.5)

    def assertStraightFullBrake(self, control):
        self.assertEqual(control.throttle, 0.0)
        self.assertEqual(control.steer, 0.0)
        self.assertEqual(control.brake, 1.0)

    def test_lagging_actor_continues_toward_finite_endpoint(self):
        control, held = self._terminal_step(
            _FakeVehicle(x=5.0, speed=0.0))

        self.assertFalse(held)
        self.assertGreater(control.throttle, 0.0)
        self.assertLess(control.brake, 1.0)

    def test_actor_within_terminal_tolerance_holds(self):
        control, held = self._terminal_step(
            _FakeVehicle(x=9.75, speed=2.0))

        self.assertTrue(held)
        self.assertStraightFullBrake(control)

    def test_actor_that_passed_endpoint_never_steers_back(self):
        control, held = self._terminal_step(
            _FakeVehicle(x=10.5, speed=1.0))

        self.assertTrue(held)
        self.assertStraightFullBrake(control)

    def test_curved_path_uses_arc_distance_remaining(self):
        progress = endpoint_progress(
            [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)],
            x=2.0, y=0.0)

        self.assertAlmostEqual(progress["remaining_distance"], 8.0)
        self.assertAlmostEqual(progress["endpoint_distance"], 34.0 ** 0.5)
        self.assertAlmostEqual(progress["signed_final_distance"], 5.0)
        self.assertEqual(progress["projected_segment"], 0)
        self.assertFalse(progress["on_final_segment"])

    def test_duplicate_terminal_points_use_last_nonzero_segment(self):
        progress = endpoint_progress(
            [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0),
             (10.0, 0.0), (10.0, 0.0)],
            x=8.0, y=0.0)

        self.assertAlmostEqual(progress["remaining_distance"], 2.0)
        self.assertAlmostEqual(progress["signed_final_distance"], 2.0)
        self.assertAlmostEqual(progress["endpoint_distance"], 2.0)
        self.assertEqual(progress["projected_segment"], 1)
        self.assertTrue(progress["on_final_segment"])

    def test_high_speed_approach_brakes_to_endpoint_speed_cap(self):
        control, held = self._terminal_step(
            _FakeVehicle(x=5.0, speed=8.0))

        self.assertFalse(held)
        self.assertEqual(control.throttle, 0.0)
        self.assertGreater(control.brake, 0.0)

    def test_lateral_offset_near_endpoint_projection_keeps_approaching(self):
        control, held = self._terminal_step(
            _FakeVehicle(x=9.75, y=3.0, speed=0.0))

        self.assertFalse(held)
        self.assertGreater(control.throttle, 0.0)

    def test_terminal_hold_remains_latched_after_backward_displacement(self):
        control, held = self._terminal_step(
            _FakeVehicle(x=9.75, speed=1.0))
        self.assertTrue(held)
        self.assertStraightFullBrake(control)

        # A collision or physics correction may push the actor away from its
        # parked endpoint. Once parked, it must not resume path pursuit.
        control, held = self._terminal_step(
            _FakeVehicle(x=5.0, speed=0.0))

        self.assertTrue(held)
        self.assertStraightFullBrake(control)


if __name__ == "__main__":
    unittest.main()
