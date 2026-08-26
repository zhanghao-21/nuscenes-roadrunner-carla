import sys
import types
import unittest
from unittest import mock


# The fidelity wrapper is testable without a CARLA server.  Supply only the
# import placeholder needed by the runtime module on machines without the
# CARLA Python package.
sys.modules.setdefault("carla", types.ModuleType("carla"))

from closed_loop.tracks import ActorTrack, TrackPoint  # noqa: E402
from runtime.run_sumo_hybrid import (  # noqa: E402
    _apply_reference_track_control, _detach_carla_static_sumo_proxies,
    _install_carla_to_sumo_spawn_exclusion, _install_sumo_mover_fidelity,
    _install_sumo_route_continuation, _prediction_risk_actor_states,
    _sumo_front_bumper_xy, _terminal_stationary_tail_start)


class _FakeVehicleDomain:
    def __init__(self, simulation):
        self.simulation = simulation
        self.calls = []

    def getIDList(self):
        return list(self.simulation.active_ids)

    def getLength(self, actor_id):
        self.calls.append(("getLength", actor_id))
        return 4.7

    def setSpeed(self, actor_id, speed):
        self.calls.append(("setSpeed", actor_id, speed))

    def setSpeedMode(self, actor_id, mode):
        self.calls.append(("setSpeedMode", actor_id, mode))

    def setMaxSpeed(self, actor_id, speed):
        self.calls.append(("setMaxSpeed", actor_id, speed))

    def setPreviousSpeed(self, actor_id, speed):
        self.calls.append(("setPreviousSpeed", actor_id, speed))

    def moveToXY(self, actor_id, edge_id, lane, x, y, **kwargs):
        self.calls.append((
            "moveToXY", actor_id, edge_id, lane, x, y, kwargs))


class _FakeSimulationDomain:
    def __init__(self):
        self.time = 0.0
        self.active_ids = set()

    def getTime(self):
        return self.time

    def getPendingVehicles(self):
        return []


class _FakeSumoSimulation:
    def __init__(self, simulation, actor_id, step_length):
        self.simulation = simulation
        self.actor_id = actor_id
        self.step_length = step_length

    def tick(self):
        self.simulation.time += self.step_length
        self.simulation.active_ids.add(self.actor_id)


class _OrderingVehicleDomain(_FakeVehicleDomain):
    def __init__(self, simulation, events):
        super().__init__(simulation)
        self.events = events

    def setSpeed(self, actor_id, speed):
        super().setSpeed(actor_id, speed)
        self.events.append(("setSpeed", actor_id, speed))


class _OrderingSumoSimulation(_FakeSumoSimulation):
    def __init__(self, simulation, actor_id, step_length, events):
        super().__init__(simulation, actor_id, step_length)
        self.events = events

    def tick(self):
        self.events.append(("simulationStep", self.simulation.time))
        super().tick()


class _ReleaseRequestingContinuator:
    def __init__(self, actor_id, events):
        self.actor_id = actor_id
        self.events = events
        self.eligible_vehicle_ids = {actor_id}
        self.release_requested = False

    def recover_persistent_stops(
            self, autonomous_ids, simulation_time,
            speed_authority_vehicle_ids=None,
            release_speed_authority=None):
        autonomous_ids = set(autonomous_ids)
        speed_authority_vehicle_ids = set(
            speed_authority_vehicle_ids or ())
        self.events.append((
            "recover", simulation_time, autonomous_ids,
            speed_authority_vehicle_ids))
        if (not self.release_requested and
                self.actor_id in speed_authority_vehicle_ids):
            self.release_requested = True
            released = release_speed_authority(
                self.actor_id, simulation_time,
                "persistent_unexplained_lane_stop")
            self.events.append(("release_result", released))
        return []

    def extend_active_routes(self, simulation_time):
        self.events.append(("extend", simulation_time))
        return []


class SumoMoverFidelityTests(unittest.TestCase):
    def test_center_pose_conversion_uses_sumo_front_bumper(self):
        self.assertEqual(
            _sumo_front_bumper_xy({
                "x": 10.0,
                "y": 20.0,
                "angle_degrees": 90.0,
                "reference_point": "vehicle_center",
            }, 4.0),
            (12.0, 20.0))
        self.assertEqual(
            _sumo_front_bumper_xy({
                "x": 10.0,
                "y": 20.0,
                "angle_degrees": 0.0,
                "reference_point": "front_center_bumper",
            }, 4.0),
            (10.0, 20.0))

    def test_tick_wrapper_initializes_pose_and_releases_speed_profile(self):
        actor_id = "nusc_actor"
        step_length = 0.05
        track = ActorTrack("actor", "vehicle.car", [
            TrackPoint(0, 0.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 0.1, 0.4, 0.0, 0.0),
        ])
        route_rows = {
            actor_id: {
                "id": "actor",
                "first_matched_time": 0.0,
                "sumo_max_speed_mps": 4.0,
                "sumo_vehicle_length_m": 4.0,
                "initial_sumo_pose": {
                    "x": 10.0,
                    "y": 20.0,
                    "angle_degrees": 90.0,
                    "reference_point": "vehicle_center",
                },
            },
        }
        simulation = _FakeSimulationDomain()
        vehicle = _FakeVehicleDomain(simulation)
        fake_traci = types.ModuleType("traci")
        fake_traci.simulation = simulation
        fake_traci.vehicle = vehicle
        sumo = _FakeSumoSimulation(simulation, actor_id, step_length)

        with mock.patch.dict(sys.modules, {"traci": fake_traci}):
            state = _install_sumo_mover_fidelity(
                sumo, route_rows, {"actor": track},
                "recorded", "recorded_profile", step_length, 8.0)

        sumo.tick()
        move = next(call for call in vehicle.calls if call[0] == "moveToXY")
        self.assertAlmostEqual(move[4], 12.0)
        self.assertAlmostEqual(move[5], 20.0)
        self.assertEqual(move[6]["angle"], 90.0)
        self.assertEqual(move[6]["keepRoute"], 1)
        self.assertEqual(state["pose_applied"], {actor_id})
        self.assertEqual(state["pose_applied_at_s"], {actor_id: step_length})
        self.assertIn(("setSpeedMode", actor_id, 29), vehicle.calls)
        self.assertEqual(state["_active_profile_ids"], {actor_id})
        self.assertTrue(callable(state["_release_profile"]))

        # The two recorded points provide a target through t < 0.1 s.  Once
        # the wrapper reaches the endpoint it releases setSpeed with -1.
        sumo.tick()
        sumo.tick()
        sumo.tick()
        self.assertEqual(state["profile_released"], {actor_id})
        self.assertEqual(state["_active_profile_ids"], set())
        self.assertIn(("setSpeed", actor_id, -1.0), vehicle.calls)

    def test_guarded_release_precedes_inner_speed_replay_and_sumo_step(self):
        actor_id = "nusc_persistent_stop"
        step_length = 0.05
        track = ActorTrack("persistent-stop", "vehicle.car", [
            TrackPoint(0, 0.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 10.0, 40.0, 0.0, 0.0),
        ])
        route_rows = {
            actor_id: {
                "id": track.actor_id,
                "first_matched_time": 0.0,
                "sumo_max_speed_mps": 4.0,
            },
        }
        simulation = _FakeSimulationDomain()
        events = []
        vehicle = _OrderingVehicleDomain(simulation, events)
        fake_traci = types.ModuleType("traci")
        fake_traci.simulation = simulation
        fake_traci.vehicle = vehicle
        sumo = _OrderingSumoSimulation(
            simulation, actor_id, step_length, events)

        with mock.patch.dict(sys.modules, {"traci": fake_traci}):
            fidelity = _install_sumo_mover_fidelity(
                sumo, route_rows, {track.actor_id: track},
                "route", "recorded_profile", step_length, 8.0)
            # Initialize the active recorded profile, then install the route
            # wrapper in the same outer-over-inner order used by run().
            sumo.tick()
            self.assertEqual(
                fidelity["_active_profile_ids"], {actor_id})
            continuator = _ReleaseRequestingContinuator(actor_id, events)
            _install_sumo_route_continuation(
                sumo, continuator, fidelity)

            events[:] = []
            vehicle.calls[:] = []
            release_time = simulation.time
            sumo.tick()

        self.assertTrue(continuator.release_requested)
        self.assertEqual(fidelity["_active_profile_ids"], set())
        self.assertEqual(fidelity["profile_released"], {actor_id})
        self.assertEqual(
            fidelity["profile_released_at_s"], {actor_id: release_time})
        self.assertEqual(fidelity["profile_release_reason"], {
            actor_id: "persistent_unexplained_lane_stop",
        })
        release_call = ("setSpeed", actor_id, -1.0)
        self.assertIn(release_call, events)
        simulation_step_index = next(
            index for index, event in enumerate(events)
            if event[0] == "simulationStep")
        self.assertLess(events.index(release_call), simulation_step_index)
        # Removing active authority in the outer pre-tick callback must keep
        # the inner fidelity wrapper from replaying a positive target speed.
        self.assertFalse(any(
            event[0] == "setSpeed" and event[2] >= 0.0
            for event in events))

    def test_terminal_stationary_tail_is_detected_but_middle_stop_is_not(self):
        terminal_stop = ActorTrack("terminal-stop", "vehicle.car", [
            TrackPoint(0, 0.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 1.0, 4.0, 0.0, 0.0),
            TrackPoint(2, 2.0, 8.0, 0.0, 0.0),
            TrackPoint(3, 3.0, 8.3, 0.1, 0.0),
            TrackPoint(4, 4.0, 7.9, -0.1, 0.0),
            TrackPoint(5, 5.0, 8.2, 0.2, 0.0),
        ])
        middle_stop = ActorTrack("middle-stop", "vehicle.car", [
            TrackPoint(0, 0.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 1.0, 5.0, 0.0, 0.0),
            TrackPoint(2, 2.0, 5.2, 0.1, 0.0),
            TrackPoint(3, 3.0, 4.9, -0.1, 0.0),
            TrackPoint(4, 4.0, 8.0, 0.0, 0.0),
            TrackPoint(5, 5.0, 12.0, 0.0, 0.0),
        ])

        self.assertEqual(
            _terminal_stationary_tail_start(
                terminal_stop, maximum_extent=1.0,
                minimum_duration=2.0),
            2.0)
        self.assertIsNone(_terminal_stationary_tail_start(
            middle_stop, maximum_extent=1.0,
            minimum_duration=2.0))

    def test_long_terminal_stop_releases_profile_without_forcing_motion(self):
        actor_id = "nusc_terminal_stop"
        step_length = 0.5
        track = ActorTrack("terminal-stop", "vehicle.car", [
            TrackPoint(0, 0.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 1.0, 4.0, 0.0, 0.0),
            TrackPoint(2, 2.0, 8.0, 0.0, 0.0),
            TrackPoint(3, 3.0, 8.3, 0.1, 0.0),
            TrackPoint(4, 4.0, 7.9, -0.1, 0.0),
            TrackPoint(5, 5.0, 8.2, 0.2, 0.0),
        ])
        route_rows = {
            actor_id: {
                "id": track.actor_id,
                "first_matched_time": 0.0,
                "sumo_max_speed_mps": 4.0,
            },
        }
        simulation = _FakeSimulationDomain()
        vehicle = _FakeVehicleDomain(simulation)
        fake_traci = types.ModuleType("traci")
        fake_traci.simulation = simulation
        fake_traci.vehicle = vehicle
        sumo = _FakeSumoSimulation(simulation, actor_id, step_length)

        with mock.patch.dict(sys.modules, {"traci": fake_traci}):
            state = _install_sumo_mover_fidelity(
                sumo, route_rows, {track.actor_id: track},
                "route", "recorded_profile", step_length, 8.0,
                terminal_stop_policy="release_to_sumo",
                terminal_stop_maximum_extent=1.0,
                terminal_stop_minimum_duration=2.0)

            # Initialization occurs on the first tick. At 2.5 s of SUMO time,
            # the replay clock reaches the stationary-tail start at track
            # time 2.0 s. Speed mode restoration occurred on an earlier tick.
            for _ in range(5):
                sumo.tick()
            calls_before_release = len(vehicle.calls)
            sumo.tick()
            release_calls = vehicle.calls[calls_before_release:]

        self.assertEqual(
            state["terminal_release_track_time_s"], {actor_id: 2.0})
        self.assertEqual(state["profile_released"], {actor_id})
        self.assertEqual(
            state["profile_release_reason"],
            {actor_id: "terminal_recorded_stationary_tail"})
        self.assertEqual(state["profile_released_at_s"], {actor_id: 2.5})
        # Releasing replay restores SUMO authority. It must not teleport the
        # actor, bypass safety modes, inject a speed, or command motion.
        self.assertEqual(release_calls, [("setSpeed", actor_id, -1.0)])
        self.assertFalse(any(call[0] == "moveToXY" for call in vehicle.calls))


class PredictionRiskActorFilterTests(unittest.TestCase):
    def test_excludes_recorded_static_and_keeps_stopped_dynamic_vehicles(self):
        # Selection follows recorded authority, not a noisy instantaneous
        # velocity. A static prop is excluded even if CARLA reports motion.
        parked = {"position": (1.0, 0.0), "speed": 2.0}
        stopped_sumo_mover = {"position": (2.0, 0.0), "speed": 0.0}
        stopped_critical_actor = {"position": (3.0, 0.0), "speed": 0.0}
        vehicle_rows = [
            (11, "track:parked", parked),
            (22, "sumo:nusc_mover", stopped_sumo_mover),
            (33, "track:critical", stopped_critical_actor),
        ]

        selected = _prediction_risk_actor_states(vehicle_rows, {11})

        self.assertEqual(set(selected), {
            "sumo:nusc_mover", "track:critical"})
        self.assertIs(selected["sumo:nusc_mover"], stopped_sumo_mover)
        self.assertIs(selected["track:critical"], stopped_critical_actor)


class CarlaStaticSumoProxyTests(unittest.TestCase):
    def test_detaches_only_static_proxies_and_does_not_respawn_them(self):
        sumo = mock.Mock()
        synchronization = types.SimpleNamespace(
            sumo=sumo,
            carla2sumo_ids={10: "carla0", 20: "carla1", 99: "carla2"})

        detached, failures = _detach_carla_static_sumo_proxies(
            synchronization, {20, 10, 404})

        self.assertEqual(detached, {"10": "carla0", "20": "carla1"})
        self.assertEqual(failures, {})
        self.assertEqual(synchronization.carla2sumo_ids, {99: "carla2"})
        self.assertEqual(sumo.unsubscribe.call_args_list, [
            mock.call("carla0"), mock.call("carla1")])
        self.assertEqual(sumo.destroy_actor.call_args_list, [
            mock.call("carla0"), mock.call("carla1")])

        # Once the bridge mapping is gone, later exclusion passes are no-ops;
        # CARLA still owns the original actors and the bridge cannot update or
        # respawn their removed SUMO shadows.
        detached, failures = _detach_carla_static_sumo_proxies(
            synchronization, {10, 20})
        self.assertEqual((detached, failures), ({}, {}))
        self.assertEqual(sumo.destroy_actor.call_count, 2)

    def test_failed_proxy_removal_keeps_bridge_mapping_for_safe_retry(self):
        sumo = mock.Mock()
        sumo.destroy_actor.side_effect = RuntimeError("remove failed")
        synchronization = types.SimpleNamespace(
            sumo=sumo, carla2sumo_ids={10: "carla0"})

        detached, failures = _detach_carla_static_sumo_proxies(
            synchronization, {10})

        self.assertEqual(detached, {})
        self.assertEqual(failures, {"10": "remove failed"})
        self.assertEqual(synchronization.carla2sumo_ids, {10: "carla0"})
        sumo.subscribe.assert_called_once_with("carla0")

    def test_spawn_filter_keeps_carla_actor_but_hides_announcement(self):
        carla_simulation = types.SimpleNamespace(spawned_actors=set())

        def original_tick():
            carla_simulation.spawned_actors = {10, 20, 99}

        carla_simulation.tick = original_tick
        synchronization = types.SimpleNamespace(carla=carla_simulation)
        excluded = {10, 20}

        state = _install_carla_to_sumo_spawn_exclusion(
            synchronization, excluded)
        synchronization.carla.tick()

        self.assertEqual(synchronization.carla.spawned_actors, {99})
        self.assertEqual(state["filtered_carla_actor_ids"], {10, 20})
        # The filter consumes only the bridge's one-tick spawn notification;
        # it has no actor-destruction API and therefore cannot remove CARLA
        # roadside vehicles from the world.
        self.assertFalse(hasattr(synchronization, "destroy_actor"))


class ReferenceTrackEndControlTests(unittest.TestCase):
    class FakeControl:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    class FakeActor:
        def __init__(self, x, y=0.0, speed=0.0):
            self._location = types.SimpleNamespace(x=x, y=y, z=0.0)
            self._velocity = types.SimpleNamespace(x=speed, y=0.0, z=0.0)
            self.controls = []

        def get_location(self):
            return self._location

        def get_velocity(self):
            return self._velocity

        def apply_control(self, control):
            self.controls.append(control)

    class FakeController:
        def __init__(self):
            self.calls = []
            self.terminal_calls = []

        def run_step(self, actor, vehicles, elapsed):
            self.calls.append((actor, vehicles, elapsed))
            return ReferenceTrackEndControlTests.FakeControl(
                throttle=0.5, steer=0.25, brake=0.0)

        def run_step_to_endpoint(self, actor, vehicles, elapsed):
            self.terminal_calls.append((actor, vehicles, elapsed))
            if actor.get_location().x >= 9.25:
                return (ReferenceTrackEndControlTests.FakeControl(
                    throttle=0.0, steer=0.0, brake=1.0), True)
            return (ReferenceTrackEndControlTests.FakeControl(
                throttle=0.35, steer=0.1, brake=0.0), False)

    def setUp(self):
        # A straight terminal segment makes "behind" and "past" unambiguous:
        # the recorded path progresses in CARLA from x=0 toward x=10.
        self.track = ActorTrack("critical", "vehicle.car", [
            TrackPoint(0, 2.0, 0.0, 0.0, 0.0),
            TrackPoint(1, 5.5, 10.0, 0.0, 0.0),
        ])

    def _apply(self, actor, controller, sim_time):
        with mock.patch.object(
                sys.modules["carla"], "VehicleControl", self.FakeControl,
                create=True):
            return _apply_reference_track_control(
                actor, controller, self.track, [actor], sim_time)

    def assertStraightFullBrake(self, control):
        self.assertEqual(control.throttle, 0.0)
        self.assertEqual(control.steer, 0.0)
        self.assertEqual(control.brake, 1.0)

    def test_actor_behind_endpoint_at_recorded_end_keeps_approaching(self):
        actor = self.FakeActor(x=5.0)
        controller = self.FakeController()

        held = self._apply(actor, controller, self.track.end_time)

        self.assertFalse(held)
        self.assertEqual(controller.calls, [])
        self.assertEqual(len(controller.terminal_calls), 1)
        self.assertAlmostEqual(
            controller.terminal_calls[0][2], self.track.duration)
        self.assertEqual(len(actor.controls), 1)
        self.assertGreater(actor.controls[-1].throttle, 0.0)

    def test_actor_near_endpoint_brakes_and_straightens(self):
        actor = self.FakeActor(x=9.75, speed=2.0)
        controller = self.FakeController()

        held = self._apply(actor, controller, self.track.end_time)

        self.assertTrue(held)
        self.assertEqual(controller.calls, [])
        self.assertEqual(len(controller.terminal_calls), 1)
        self.assertStraightFullBrake(actor.controls[-1])

    def test_actor_past_endpoint_brakes_without_steering_back(self):
        actor = self.FakeActor(x=10.5, speed=1.0)
        controller = self.FakeController()

        held = self._apply(actor, controller, self.track.end_time)

        self.assertTrue(held)
        self.assertEqual(controller.calls, [])
        self.assertEqual(len(controller.terminal_calls), 1)
        self.assertStraightFullBrake(actor.controls[-1])

        # The hold is actively reapplied on later ticks, and pure pursuit is
        # never allowed to turn the actor back toward the endpoint and orbit.
        held = self._apply(actor, controller, 12.0)
        self.assertTrue(held)
        self.assertEqual(controller.calls, [])
        self.assertEqual(len(controller.terminal_calls), 2)
        self.assertStraightFullBrake(actor.controls[-1])


if __name__ == "__main__":
    unittest.main()
