import unittest


from closed_loop.sumo_runtime import SumoRouteContinuator


class _FakeConnection:
    def __init__(self, direction):
        self.direction = direction

    def getDirection(self):
        return self.direction


class _FakeEdge:
    def __init__(self, edge_id, length=20.0, to_coordinate=(50.0, 50.0)):
        self.edge_id = edge_id
        self.length = float(length)
        self.to_coordinate = to_coordinate
        self.outgoing = {}

    def getID(self):
        return self.edge_id

    def getAllowedOutgoing(self, vehicle_class):
        return self.outgoing.get(vehicle_class, {})

    def getLength(self):
        return self.length

    def getToNode(self):
        return _FakeNode(self.to_coordinate)


class _FakeNode:
    def __init__(self, coordinate):
        self.coordinate = coordinate

    def getCoord(self):
        return self.coordinate


class _FakeNetwork:
    def __init__(self):
        self.edges = {
            edge_id: _FakeEdge(edge_id)
            for edge_id in (
                "a", "p", "q", "b", "c", "uturn", "dead", "tiny", "short",
                "tiny2", "blocked", "alt", "after", "boundary", "stub")
        }
        for edge_id in ("b", "c", "uturn", "after", "boundary"):
            self.edges[edge_id].to_coordinate = (100.0, 50.0)
        self.edges["short"].length = 4.0
        self.edges["tiny"].length = 0.2
        self.edges["tiny2"].length = 0.2
        self.edges["stub"].length = 2.0
        self.edges["a"].outgoing["passenger"] = {
            self.edges["b"]: [_FakeConnection("s")],
            self.edges["c"]: [_FakeConnection("r")],
            self.edges["stub"]: [_FakeConnection("l")],
            self.edges["uturn"]: [_FakeConnection("t")],
        }
        self.edges["tiny"].outgoing["passenger"] = {
            self.edges["tiny2"]: [_FakeConnection("s")],
            self.edges["uturn"]: [_FakeConnection("t")],
        }
        self.edges["tiny2"].outgoing["passenger"] = {
            self.edges["after"]: [_FakeConnection("s")],
        }
        self.edges["p"].outgoing["passenger"] = {
            self.edges["short"]: [_FakeConnection("s")],
            self.edges["stub"]: [_FakeConnection("r")],
        }
        self.edges["q"].outgoing["passenger"] = {
            self.edges["b"]: [_FakeConnection("s")],
            self.edges["stub"]: [_FakeConnection("r")],
        }
        self.edges["short"].outgoing["passenger"] = {
            self.edges["blocked"]: [_FakeConnection("s")],
            self.edges["alt"]: [_FakeConnection("r")],
        }
        self.edges["blocked"].outgoing["passenger"] = {
            self.edges["boundary"]: [_FakeConnection("s")],
        }
        self.edges["alt"].outgoing["passenger"] = {
            self.edges["boundary"]: [_FakeConnection("s")],
        }

    def getEdge(self, edge_id):
        return self.edges[edge_id]

    def getBoundary(self):
        return (0.0, 0.0, 100.0, 100.0)


class _FakeVehicleDomain:
    def __init__(self):
        self.routes = {
            "eligible-tail": ["a"],
            "eligible-remaining": ["a", "b"],
            "eligible-road-end": ["dead"],
            "eligible-short-final": ["a", "tiny"],
            "eligible-suffix": ["p", "short", "blocked", "boundary"],
            "eligible-stub": ["q", "b"],
            "carla-owned": ["a", "b"],
            "mirrored-carla": ["a", "b"],
        }
        self.route_indices = {
            vehicle_id: 0 for vehicle_id in self.routes
        }
        self.road_ids = {
            vehicle_id: route[0]
            for vehicle_id, route in self.routes.items()
        }
        self.speeds = {vehicle_id: 0.0 for vehicle_id in self.routes}
        self.positions = {
            vehicle_id: (0.0, 0.0) for vehicle_id in self.routes
        }
        self.stop_states = {
            vehicle_id: 0 for vehicle_id in self.routes
        }
        self.leaders = {
            vehicle_id: None for vehicle_id in self.routes
        }
        self.next_tls = {
            vehicle_id: [] for vehicle_id in self.routes
        }
        self.next_links = {
            vehicle_id: [
                ("b_0", None, None, True, False, "G")]
            for vehicle_id in self.routes
        }
        self.next_links["eligible-suffix"] = [
            ("short_0", False, True, False, ":short", "G", "s", 4.0),
            ("blocked_0", False, True, True, ":blocked", "m", "s", 20.0),
            ("boundary_0", True, True, False, ":boundary", "M", "s", 20.0),
        ]
        self.route_queries = []
        self.set_route_attempts = []
        self.set_route_calls = []
        self.set_route_failures_remaining = {}

    def add_vehicle(self, vehicle_id, route, next_links=None):
        route = list(route)
        self.routes[vehicle_id] = route
        self.route_indices[vehicle_id] = 0
        self.road_ids[vehicle_id] = route[0]
        self.speeds[vehicle_id] = 0.0
        self.positions[vehicle_id] = (0.0, 0.0)
        self.stop_states[vehicle_id] = 0
        self.leaders[vehicle_id] = None
        self.next_tls[vehicle_id] = []
        self.next_links[vehicle_id] = list(next_links or [
            (route[1] + "_0", None, None, True, False, "G")
        ])

    def getIDList(self):
        return list(self.routes)

    def getRoute(self, vehicle_id):
        self.route_queries.append(("getRoute", vehicle_id))
        return list(self.routes[vehicle_id])

    def getRouteIndex(self, vehicle_id):
        self.route_queries.append(("getRouteIndex", vehicle_id))
        return self.route_indices[vehicle_id]

    def getRoadID(self, vehicle_id):
        self.route_queries.append(("getRoadID", vehicle_id))
        return self.road_ids[vehicle_id]

    def getVehicleClass(self, vehicle_id):
        self.route_queries.append(("getVehicleClass", vehicle_id))
        return "passenger"

    def getSpeed(self, vehicle_id):
        self.route_queries.append(("getSpeed", vehicle_id))
        return self.speeds[vehicle_id]

    def getPosition(self, vehicle_id):
        self.route_queries.append(("getPosition", vehicle_id))
        return self.positions[vehicle_id]

    def getLength(self, vehicle_id):
        self.route_queries.append(("getLength", vehicle_id))
        return 4.7

    def getMinGap(self, vehicle_id):
        self.route_queries.append(("getMinGap", vehicle_id))
        return 2.5

    def getStopState(self, vehicle_id):
        self.route_queries.append(("getStopState", vehicle_id))
        return self.stop_states[vehicle_id]

    def getLeader(self, vehicle_id, maximum_distance):
        self.route_queries.append(("getLeader", vehicle_id))
        return self.leaders[vehicle_id]

    def getNextTLS(self, vehicle_id):
        self.route_queries.append(("getNextTLS", vehicle_id))
        return list(self.next_tls[vehicle_id])

    def getNextLinks(self, vehicle_id):
        self.route_queries.append(("getNextLinks", vehicle_id))
        return list(self.next_links[vehicle_id])

    def setRoute(self, vehicle_id, route):
        route = list(route)
        self.set_route_attempts.append((vehicle_id, route))
        failures_remaining = self.set_route_failures_remaining.get(
            vehicle_id, 0)
        if failures_remaining > 0:
            self.set_route_failures_remaining[vehicle_id] = (
                failures_remaining - 1)
            raise RuntimeError("injected setRoute failure")
        self.set_route_calls.append((vehicle_id, route))
        self.routes[vehicle_id] = route
        self.route_indices[vehicle_id] = 0
        self.road_ids[vehicle_id] = route[0]


class SumoRouteContinuatorTests(unittest.TestCase):
    @staticmethod
    def _continuator(eligible=None, seed=103,
                     accepted_network_terminals=None):
        vehicle = _FakeVehicleDomain()
        if eligible is None:
            eligible = {
                "eligible-tail", "eligible-remaining",
                "eligible-road-end"}
        continuator = SumoRouteContinuator(
            _FakeNetwork(), vehicle,
            eligible_vehicle_ids=eligible,
            seed=seed, allow_uturns=False,
            accepted_network_terminal_vehicle_ids=(
                accepted_network_terminals))
        return vehicle, continuator

    def test_accepts_prepared_network_terminal_without_marking_unresolved(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-road-end"},
            accepted_network_terminals={"eligible-road-end"})

        self.assertEqual(continuator.extend_active_routes(4.5), [])
        self.assertEqual(vehicle.set_route_calls, [])
        metadata = continuator.metadata()
        self.assertEqual(
            metadata["accepted_network_terminal_vehicle_ids"],
            ["eligible-road-end"])
        self.assertEqual(metadata["accepted_network_terminals"], {
            "eligible-road-end": "dead",
        })
        self.assertEqual(metadata["road_end_without_outgoing"], {
            "eligible-road-end": "dead",
        })
        self.assertEqual(metadata["boundary_road_ends"], {})
        self.assertEqual(metadata["unresolved_interior_route_tails"], {})

    def test_extends_only_eligible_vehicle_at_final_edge_once(self):
        vehicle, continuator = self._continuator(seed=103)

        events = continuator.extend_active_routes(simulation_time=4.5)

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event["vehicle_id"], "eligible-tail")
        self.assertEqual(event["from_edge"], "a")
        self.assertIn(event["to_edge"], {"b", "c"})
        self.assertNotEqual(event["to_edge"], "uturn")
        self.assertEqual(event["decision"], 0)
        self.assertEqual(event["simulation_time_s"], 4.5)
        self.assertEqual(vehicle.set_route_calls, [
            ("eligible-tail", ["a", event["to_edge"]])])

        # The new route still has one edge remaining, so a repeated pre/post
        # tick call cannot append a second edge at the same decision point.
        self.assertEqual(continuator.extend_active_routes(4.5), [])
        self.assertEqual(len(vehicle.set_route_calls), 1)

        queried_ids = {vehicle_id for _method, vehicle_id
                       in vehicle.route_queries}
        self.assertNotIn("carla-owned", queried_ids)
        self.assertNotIn("mirrored-carla", queried_ids)
        self.assertEqual(vehicle.routes["eligible-remaining"], ["a", "b"])
        self.assertEqual(vehicle.routes["eligible-road-end"], ["dead"])
        metadata = continuator.metadata()
        self.assertEqual(metadata["boundary_road_ends"], {
            "eligible-remaining": "b",
            "eligible-tail": event["to_edge"],
        })
        self.assertEqual(metadata["unresolved_interior_route_tails"], {
            "eligible-road-end": "dead",
        })

        # The same seed and state must choose the same valid outgoing edge.
        repeated_vehicle, repeated = self._continuator(seed=103)
        repeated_event = repeated.extend_active_routes(4.5)[0]
        self.assertEqual(repeated_event["to_edge"], event["to_edge"])
        self.assertEqual(
            repeated_vehicle.set_route_calls,
            [("eligible-tail", ["a", event["to_edge"]])])

    def test_failed_tail_extension_retries_same_edge_and_decision_zero(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-tail"}, seed=103)
        vehicle.set_route_failures_remaining["eligible-tail"] = 1

        self.assertEqual(
            continuator.extend_active_routes(simulation_time=4.5), [])

        self.assertEqual(len(vehicle.set_route_attempts), 1)
        failed_proposal = vehicle.set_route_attempts[0]
        self.assertEqual(failed_proposal[0], "eligible-tail")
        self.assertEqual(failed_proposal[1][0], "a")
        self.assertIn(failed_proposal[1][1], {"b", "c"})
        self.assertEqual(vehicle.routes["eligible-tail"], ["a"])
        self.assertEqual(vehicle.set_route_calls, [])
        self.assertEqual(
            continuator.decision_counts.get("eligible-tail", 0), 0)
        self.assertIn("eligible-tail", continuator.failures)

        events = continuator.extend_active_routes(simulation_time=4.6)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["decision"], 0)
        self.assertEqual(events[0]["to_edge"], failed_proposal[1][1])
        self.assertEqual(vehicle.set_route_attempts, [
            failed_proposal, failed_proposal])
        self.assertEqual(vehicle.set_route_calls, [failed_proposal])
        self.assertEqual(continuator.decision_counts["eligible-tail"], 1)
        self.assertNotIn("eligible-tail", continuator.failures)

    def test_proactively_extends_complete_witness_beyond_short_final_edges(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-short-final"})

        events = continuator.extend_active_routes(simulation_time=1.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["vehicle_id"], "eligible-short-final")
        self.assertEqual(events[0]["trigger_edge"], "a")
        self.assertEqual(events[0]["from_edge"], "tiny")
        self.assertEqual(events[0]["to_edge"], "after")
        self.assertEqual(events[0]["appended_edges"], ["tiny2", "after"])
        self.assertTrue(events[0]["proactive"])
        self.assertEqual(vehicle.set_route_calls, [
            ("eligible-short-final", [
                "a", "tiny", "tiny2", "after"])])

        # Both sub-metre edges can be consumed inside one SUMO tick. Appending
        # the whole connected witness in the pre-tick call leaves a true
        # boundary road end instead of another nonterminal short edge.
        self.assertEqual(continuator.extend_active_routes(1.0), [])
        self.assertEqual(len(vehicle.set_route_calls), 1)

    def test_avoids_a_planned_interior_sink_from_penultimate_current_edge(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        for edge_id in (
                "avoid_current", "avoid_interior", "avoid_safe"):
            network.edges[edge_id] = _FakeEdge(edge_id)
        network.edges["avoid_current"].outgoing["passenger"] = {
            network.edges["avoid_interior"]: [_FakeConnection("s")],
            network.edges["avoid_safe"]: [_FakeConnection("r")],
        }
        network.edges["avoid_safe"].outgoing["passenger"] = {
            network.edges["boundary"]: [_FakeConnection("s")],
        }
        vehicle.add_vehicle(
            "eligible-interior-avoidance",
            ["avoid_current", "avoid_interior"])
        continuator = SumoRouteContinuator(
            network, vehicle,
            eligible_vehicle_ids={"eligible-interior-avoidance"}, seed=103)

        events = continuator.extend_active_routes(simulation_time=2.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(
            events[0]["mutation_type"], "interior_route_tail_avoidance")
        self.assertEqual(events[0]["vehicle_id"],
                         "eligible-interior-avoidance")
        self.assertEqual(events[0]["appended_edges"], [
            "avoid_safe", "boundary"])
        self.assertEqual(vehicle.set_route_calls, [
            ("eligible-interior-avoidance", [
                "avoid_current", "avoid_safe", "boundary"])])
        self.assertNotIn(
            "eligible-interior-avoidance",
            continuator.metadata()["unresolved_interior_route_tails"])

    def test_runtime_cycle_witness_is_replenished_with_multi_edge_lookahead(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        for edge_id in ("cycle_current", "cycle_a", "cycle_b"):
            network.edges[edge_id] = _FakeEdge(edge_id, length=0.2)
        network.edges["cycle_current"].outgoing["passenger"] = {
            network.edges["cycle_a"]: [_FakeConnection("s")],
        }
        network.edges["cycle_a"].outgoing["passenger"] = {
            network.edges["cycle_b"]: [_FakeConnection("s")],
        }
        network.edges["cycle_b"].outgoing["passenger"] = {
            network.edges["cycle_a"]: [_FakeConnection("s")],
        }
        vehicle_id = "eligible-runtime-cycle"
        vehicle.add_vehicle(vehicle_id, ["cycle_current", "cycle_a"])
        continuator = SumoRouteContinuator(
            network, vehicle, eligible_vehicle_ids={vehicle_id}, seed=103,
            continuation_search_depth_edges=6)

        first_events = continuator.extend_active_routes(simulation_time=1.0)

        self.assertEqual(len(first_events), 1)
        self.assertEqual(first_events[0]["mutation_type"],
                         "route_tail_extension")
        first_lookahead = first_events[0]["appended_edges"]
        self.assertEqual(len(first_lookahead), 6)
        self.assertEqual(first_lookahead, [
            "cycle_b", "cycle_a", "cycle_b",
            "cycle_a", "cycle_b", "cycle_a"])
        self.assertEqual(vehicle.routes[vehicle_id], [
            "cycle_current", "cycle_a"] + first_lookahead)

        # Advance to the penultimate edge of the finite TraCI route. The same
        # cycle witness must be appended again before that finite list can be
        # exhausted; repeated edge IDs must not confuse route-index handling.
        vehicle.route_indices[vehicle_id] = (
            len(vehicle.routes[vehicle_id]) - 2)
        vehicle.road_ids[vehicle_id] = "cycle_b"
        second_events = continuator.extend_active_routes(simulation_time=1.1)

        self.assertEqual(len(second_events), 1)
        second_lookahead = second_events[0]["appended_edges"]
        self.assertEqual(second_lookahead, first_lookahead)
        self.assertEqual(vehicle.routes[vehicle_id], [
            "cycle_b", "cycle_a"] + second_lookahead)

    def test_release_preserves_route_until_one_second_unexplained_stop(self):
        vehicle, continuator = self._continuator()
        released = {"eligible-remaining", "carla-owned"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        self.assertEqual(vehicle.routes["eligible-remaining"], ["a", "b"])
        self.assertEqual(vehicle.set_route_calls, [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.99), [])
        self.assertEqual(vehicle.set_route_calls, [])

        events = continuator.recover_persistent_stops(released, 1.0)

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event["vehicle_id"], "eligible-remaining")
        self.assertEqual(event["from_edge"], "a")
        self.assertEqual(event["original_next_edge"], "b")
        self.assertEqual(event["to_edge"], "c")
        self.assertEqual(event["stopped_duration_s"], 1.0)
        self.assertEqual(vehicle.set_route_calls, [
            ("eligible-remaining", ["a", "c"])])
        self.assertNotIn(
            ("getRoute", "carla-owned"), vehicle.route_queries)
        metadata = continuator.metadata()
        self.assertEqual(metadata["terminal_release_immediate_reroute_count"], 0)
        self.assertEqual(metadata["persistent_stop_recovery_count"], 1)
        self.assertEqual(metadata["persistent_stop_recoveries"], events)
        self.assertEqual(metadata["persistent_stop_recovery_settings"], {
            "minimum_duration_s": 1.0,
            "stop_speed_mps": 0.1,
            "resume_speed_mps": 0.3,
            "progress_reset_distance_m": 0.5,
            "close_leader_distance_m": 15.0,
            "traffic_signal_guard_distance_m": 15.0,
            "downstream_blocker_minimum_duration_s": 1.0,
            "maximum_suffix_recovery_attempts": 3,
        })

    def test_active_speed_authority_releases_at_threshold_without_reroute(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-remaining"})
        vehicle_id = "eligible-remaining"
        release_calls = []

        def release_speed_authority(sumo_id, now, reason):
            release_calls.append((sumo_id, now, reason))
            return True

        for now in (0.0, 0.5, 0.999):
            self.assertEqual(continuator.recover_persistent_stops(
                set(), now,
                speed_authority_vehicle_ids={vehicle_id},
                release_speed_authority=release_speed_authority), [])
        self.assertEqual(release_calls, [])
        self.assertEqual(vehicle.set_route_calls, [])

        # Reaching the full persistence threshold releases only recorded
        # desired-speed authority. It must not also mutate the route.
        self.assertEqual(continuator.recover_persistent_stops(
            set(), 1.0,
            speed_authority_vehicle_ids={vehicle_id},
            release_speed_authority=release_speed_authority), [])

        self.assertEqual(release_calls, [(
            vehicle_id, 1.0, "persistent_unexplained_lane_stop")])
        self.assertEqual(vehicle.routes[vehicle_id], ["a", "b"])
        self.assertEqual(vehicle.set_route_calls, [])
        metadata = continuator.metadata()
        self.assertEqual(metadata["recorded_speed_authority_release_count"], 1)
        self.assertEqual(metadata["persistent_stop_recovery_count"], 0)
        release_event = metadata["recorded_speed_authority_releases"][0]
        self.assertEqual(release_event["vehicle_id"], vehicle_id)
        self.assertEqual(release_event["simulation_time_s"], 1.0)
        self.assertEqual(release_event["stopped_duration_s"], 1.0)

    def test_release_threshold_starts_only_after_hard_guard_clears(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-remaining"})
        vehicle_id = "eligible-remaining"
        vehicle.next_tls[vehicle_id] = [("tls", 0, 10.0, "r")]
        release_calls = []

        def release_speed_authority(sumo_id, now, reason):
            release_calls.append((sumo_id, now, reason))
            return True

        for now in (0.0, 10.0):
            self.assertEqual(continuator.recover_persistent_stops(
                set(), now,
                speed_authority_vehicle_ids={vehicle_id},
                release_speed_authority=release_speed_authority), [])
        self.assertEqual(release_calls, [])

        # Ten guarded seconds do not count. The one-second release timer starts
        # from the first observation after SUMO reports the signal clear.
        vehicle.next_tls[vehicle_id] = []
        for now in (10.0, 10.999):
            self.assertEqual(continuator.recover_persistent_stops(
                set(), now,
                speed_authority_vehicle_ids={vehicle_id},
                release_speed_authority=release_speed_authority), [])
        self.assertEqual(release_calls, [])
        self.assertEqual(continuator.recover_persistent_stops(
            set(), 11.0,
            speed_authority_vehicle_ids={vehicle_id},
            release_speed_authority=release_speed_authority), [])
        self.assertEqual(release_calls, [(
            vehicle_id, 11.0, "persistent_unexplained_lane_stop")])
        self.assertEqual(vehicle.set_route_calls, [])

    def test_hard_stop_guards_suppress_active_speed_authority_release(self):
        cases = {
            "scheduled_stop": lambda vehicle: vehicle.stop_states.__setitem__(
                "eligible-remaining", 1),
            "close_leader": lambda vehicle: vehicle.leaders.__setitem__(
                "eligible-remaining", ("leader", 14.0)),
            "red_tls": lambda vehicle: vehicle.next_tls.__setitem__(
                "eligible-remaining", [("tls", 0, 10.0, "r")]),
        }
        for name, configure in cases.items():
            with self.subTest(name=name):
                vehicle, continuator = self._continuator(
                    eligible={"eligible-remaining"})
                configure(vehicle)
                release_calls = []

                def release_speed_authority(sumo_id, now, reason):
                    release_calls.append((sumo_id, now, reason))
                    return True

                for now in (0.0, 30.0):
                    self.assertEqual(continuator.recover_persistent_stops(
                        set(), now,
                        speed_authority_vehicle_ids={
                            "eligible-remaining"},
                        release_speed_authority=release_speed_authority), [])
                self.assertEqual(release_calls, [])
                self.assertEqual(vehicle.set_route_calls, [])
                self.assertEqual(
                    continuator.metadata()[
                        "recorded_speed_authority_release_count"], 0)

    def test_release_starts_a_fresh_autonomous_reroute_window(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-remaining"})
        vehicle_id = "eligible-remaining"
        active_speed_authority = {vehicle_id}
        autonomous = set()
        release_calls = []

        def release_speed_authority(sumo_id, now, reason):
            release_calls.append((sumo_id, now, reason))
            active_speed_authority.discard(sumo_id)
            autonomous.add(sumo_id)
            return True

        self.assertEqual(continuator.recover_persistent_stops(
            autonomous, 0.0,
            speed_authority_vehicle_ids=active_speed_authority,
            release_speed_authority=release_speed_authority), [])
        self.assertEqual(continuator.recover_persistent_stops(
            autonomous, 1.0,
            speed_authority_vehicle_ids=active_speed_authority,
            release_speed_authority=release_speed_authority), [])
        self.assertEqual(len(release_calls), 1)
        self.assertEqual(vehicle.routes[vehicle_id], ["a", "b"])
        self.assertEqual(vehicle.set_route_calls, [])

        # Observe the newly autonomous vehicle at the release timestamp to
        # start, but not inherit, a fresh persistence window.
        self.assertEqual(continuator.recover_persistent_stops(
            autonomous, 1.0,
            speed_authority_vehicle_ids=active_speed_authority,
            release_speed_authority=release_speed_authority), [])
        self.assertEqual(continuator.recover_persistent_stops(
            autonomous, 1.999,
            speed_authority_vehicle_ids=active_speed_authority,
            release_speed_authority=release_speed_authority), [])
        self.assertEqual(vehicle.set_route_calls, [])

        events = continuator.recover_persistent_stops(
            autonomous, 2.0,
            speed_authority_vehicle_ids=active_speed_authority,
            release_speed_authority=release_speed_authority)

        self.assertEqual(len(events), 1)
        self.assertEqual(
            events[0]["mutation_type"],
            "persistent_stop_immediate_edge_recovery")
        self.assertEqual(events[0]["stopped_duration_s"], 1.0)
        self.assertEqual(vehicle.set_route_calls, [
            (vehicle_id, ["a", "c"])])
        metadata = continuator.metadata()
        self.assertEqual(metadata["recorded_speed_authority_release_count"], 1)
        self.assertEqual(metadata["persistent_stop_recovery_count"], 1)
        self.assertLess(
            metadata["recorded_speed_authority_releases"][0]["sequence"],
            metadata["persistent_stop_recoveries"][0]["sequence"])

    def test_failed_immediate_recovery_retries_proposal_and_decision_zero(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-remaining"}, seed=103)
        alternative = _FakeEdge(
            "d", to_coordinate=(100.0, 50.0))
        continuator.network.edges["d"] = alternative
        continuator.network.edges["a"].outgoing["passenger"][alternative] = [
            _FakeConnection("l")]
        vehicle.set_route_failures_remaining["eligible-remaining"] = 1
        released = {"eligible-remaining"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 1.0), [])

        self.assertEqual(len(vehicle.set_route_attempts), 1)
        failed_proposal = vehicle.set_route_attempts[0]
        self.assertEqual(failed_proposal[0], "eligible-remaining")
        self.assertEqual(failed_proposal[1][0], "a")
        self.assertIn(failed_proposal[1][1], {"c", "d"})
        self.assertEqual(vehicle.routes["eligible-remaining"], ["a", "b"])
        self.assertEqual(vehicle.set_route_calls, [])
        self.assertEqual(
            continuator.decision_counts.get("eligible-remaining", 0), 0)
        self.assertEqual(continuator.stall_recovery_events, [])
        self.assertIn(
            "eligible-remaining", continuator.stall_recovery_failures)

        events = continuator.recover_persistent_stops(released, 1.05)

        self.assertEqual(len(events), 1)
        self.assertEqual(
            events[0]["mutation_type"],
            "persistent_stop_immediate_edge_recovery")
        self.assertEqual(events[0]["decision"], 0)
        self.assertEqual(events[0]["to_edge"], failed_proposal[1][1])
        self.assertEqual(vehicle.set_route_attempts, [
            failed_proposal, failed_proposal])
        self.assertEqual(vehicle.set_route_calls, [failed_proposal])
        self.assertEqual(
            continuator.decision_counts["eligible-remaining"], 1)
        self.assertNotIn(
            "eligible-remaining", continuator.stall_recovery_failures)

    def test_immediate_recovery_installs_full_witness_then_tries_new_branch(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        custom_edges = (
            "immediate_current", "immediate_planned",
            "immediate_a", "immediate_a_mid",
            "immediate_b", "immediate_b_mid")
        for edge_id in custom_edges:
            network.edges[edge_id] = _FakeEdge(edge_id)
        network.edges["immediate_planned"].to_coordinate = (100.0, 50.0)
        network.edges["immediate_current"].outgoing["passenger"] = {
            network.edges["immediate_planned"]: [_FakeConnection("s")],
            network.edges["immediate_a"]: [_FakeConnection("l")],
            network.edges["immediate_b"]: [_FakeConnection("r")],
        }
        network.edges["immediate_a"].outgoing["passenger"] = {
            network.edges["immediate_a_mid"]: [_FakeConnection("s")],
        }
        network.edges["immediate_a_mid"].outgoing["passenger"] = {
            network.edges["boundary"]: [_FakeConnection("s")],
        }
        network.edges["immediate_b"].outgoing["passenger"] = {
            network.edges["immediate_b_mid"]: [_FakeConnection("s")],
        }
        network.edges["immediate_b_mid"].outgoing["passenger"] = {
            network.edges["boundary"]: [_FakeConnection("s")],
        }
        vehicle_id = "eligible-immediate-retry"
        vehicle.add_vehicle(
            vehicle_id, ["immediate_current", "immediate_planned"])
        continuator = SumoRouteContinuator(
            network, vehicle, eligible_vehicle_ids={vehicle_id}, seed=103)
        autonomous = {vehicle_id}
        expected_witnesses = {
            "immediate_a": [
                "immediate_a", "immediate_a_mid", "boundary"],
            "immediate_b": [
                "immediate_b", "immediate_b_mid", "boundary"],
        }

        self.assertEqual(
            continuator.recover_persistent_stops(autonomous, 0.0), [])
        first_events = continuator.recover_persistent_stops(
            autonomous, 1.0)

        self.assertEqual(len(first_events), 1)
        first_event = first_events[0]
        self.assertEqual(
            first_event["mutation_type"],
            "persistent_stop_immediate_edge_recovery")
        first_branch = first_event["selected_next_edge"]
        self.assertIn(first_branch, expected_witnesses)
        self.assertEqual(
            first_event["appended_edges"], expected_witnesses[first_branch])
        self.assertEqual(first_event["new_route_remainder"], [
            "immediate_current"] + expected_witnesses[first_branch])
        self.assertEqual(vehicle.routes[vehicle_id], [
            "immediate_current"] + expected_witnesses[first_branch])
        self.assertGreater(len(vehicle.routes[vehicle_id]), 2)
        self.assertEqual(vehicle.routes[vehicle_id][-1], "boundary")

        # The accepted mutation resets persistence. A stopped autonomous actor
        # must complete a fresh one-second window before another route change.
        self.assertEqual(
            continuator.recover_persistent_stops(autonomous, 1.1), [])
        self.assertEqual(
            continuator.recover_persistent_stops(autonomous, 2.099), [])
        self.assertEqual(len(vehicle.set_route_calls), 1)

        second_events = continuator.recover_persistent_stops(
            autonomous, 2.1)

        self.assertEqual(len(second_events), 1)
        second_event = second_events[0]
        self.assertEqual(
            second_event["mutation_type"],
            "persistent_stop_immediate_edge_recovery")
        self.assertEqual(second_event["original_next_edge"], first_branch)
        second_branch = second_event["selected_next_edge"]
        self.assertIn(second_branch, expected_witnesses)
        self.assertNotEqual(second_branch, first_branch)
        self.assertNotEqual(second_branch, "immediate_planned")
        self.assertEqual(
            second_event["appended_edges"], expected_witnesses[second_branch])
        self.assertEqual(vehicle.routes[vehicle_id], [
            "immediate_current"] + expected_witnesses[second_branch])
        self.assertEqual(len(vehicle.set_route_calls), 2)
        self.assertNotIn(
            vehicle_id,
            continuator.metadata()["persistent_stop_recovery_unresolved"])
        tried = {
            (item["from_edge"], item["to_edge"])
            for item in second_event["tried_immediate_branches"]
        }
        self.assertIn(
            ("immediate_current", "immediate_planned"), tried)
        self.assertIn(("immediate_current", first_branch), tried)

    def test_position_progress_resets_persistent_stop_timer(self):
        vehicle, continuator = self._continuator()
        released = {"eligible-remaining"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        vehicle.positions["eligible-remaining"] = (0.6, 0.0)
        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.5), [])

        # A new unexplained-stop window begins on this stationary observation,
        # rather than inheriting the half-second before the progress update.
        self.assertEqual(
            continuator.recover_persistent_stops(released, 1.49), [])
        self.assertEqual(vehicle.set_route_calls, [])
        events = continuator.recover_persistent_stops(released, 2.49)
        self.assertEqual(len(events), 1)
        self.assertAlmostEqual(events[0]["stopped_duration_s"], 1.0)

    def test_installed_traci_next_link_layout_and_planned_link_selection(self):
        vehicle, continuator = self._continuator()
        # The installed SUMO reader returns (lane, priority, open, foe,
        # internal, state, ...), unlike the published signature. The first
        # tuple is deliberately unrelated and blocked; the planned b link is
        # the second tuple and is clear.
        vehicle.next_links["eligible-remaining"] = [
            ("c_0", False, False, True, ":c_0", "m", "r", 5.0),
            ("b_0", False, True, False, ":b_0", "G", "s", 8.0),
        ]
        released = {"eligible-remaining"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        events = continuator.recover_persistent_stops(released, 1.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["original_next_edge"], "b")
        self.assertTrue(events[0]["evidence"]["next_link"]["is_open"])
        self.assertFalse(events[0]["evidence"]["next_link"]["has_foe"])

    def test_persistent_downstream_short_edge_conflict_changes_only_suffix(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-suffix"})
        released = {"eligible-suffix"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.99), [])
        self.assertEqual(vehicle.set_route_calls, [])

        events = continuator.recover_persistent_stops(released, 1.0)

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(
            event["mutation_type"], "persistent_stop_suffix_recovery")
        self.assertEqual(event["from_edge"], "p")
        self.assertEqual(event["preserved_next_edge"], "short")
        self.assertEqual(event["original_route_remainder"], [
            "p", "short", "blocked", "boundary"])
        self.assertEqual(event["new_route_remainder"], [
            "p", "short", "alt", "boundary"])
        self.assertEqual(
            event["evidence"]["persistent_downstream_blocker"],
            "downstream_short_edge_conflict")
        self.assertEqual(vehicle.set_route_calls, [
            ("eligible-suffix", ["p", "short", "alt", "boundary"])])

        # If that only alternative also encounters a persistent conflict, it
        # must not return to the original failed branch. With no third branch
        # and no viable different immediate turn, recovery remains unresolved
        # without another route mutation.
        vehicle.next_links["eligible-suffix"] = [
            ("short_0", False, True, False, ":short", "G", "s", 4.0),
            ("alt_0", False, True, True, ":alt", "m", "s", 20.0),
            ("boundary_0", True, True, False,
             ":boundary", "M", "s", 20.0),
        ]
        self.assertEqual(
            continuator.recover_persistent_stops(released, 1.05), [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 2.05), [])
        self.assertEqual(vehicle.routes["eligible-suffix"], [
            "p", "short", "alt", "boundary"])
        self.assertEqual(len(vehicle.set_route_calls), 1)
        unresolved = continuator.metadata()[
            "persistent_stop_recovery_unresolved"]["eligible-suffix"]
        self.assertEqual(
            unresolved["reason"], "no_viable_alternative_outgoing")

    def test_suffix_recovery_preserves_forced_prefix_before_later_divergence(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        for edge_id in (
                "forced_current", "forced_1", "forced_2",
                "forced_old", "forced_alt"):
            network.edges[edge_id] = _FakeEdge(edge_id)
        network.edges["forced_current"].outgoing["passenger"] = {
            network.edges["forced_1"]: [_FakeConnection("s")],
        }
        network.edges["forced_1"].outgoing["passenger"] = {
            network.edges["forced_2"]: [_FakeConnection("s")],
        }
        network.edges["forced_2"].outgoing["passenger"] = {
            network.edges["forced_old"]: [_FakeConnection("s")],
            network.edges["forced_alt"]: [_FakeConnection("r")],
        }
        for edge_id in ("forced_old", "forced_alt"):
            network.edges[edge_id].outgoing["passenger"] = {
                network.edges["boundary"]: [_FakeConnection("s")],
            }
        vehicle.add_vehicle(
            "eligible-forced-prefix",
            ["forced_current", "forced_1", "forced_2",
             "forced_old", "boundary"],
            [("forced_1_0", False, True, False,
              ":forced_1", "G", "s", 20.0)])
        continuator = SumoRouteContinuator(
            network, vehicle,
            eligible_vehicle_ids={"eligible-forced-prefix"}, seed=103)
        released = {"eligible-forced-prefix"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        events = continuator.recover_persistent_stops(released, 1.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(
            events[0]["mutation_type"],
            "persistent_stop_suffix_recovery")
        self.assertEqual(events[0]["new_route_remainder"], [
            "forced_current", "forced_1", "forced_2",
            "forced_alt", "boundary"])
        self.assertEqual(vehicle.set_route_calls, [
            ("eligible-forced-prefix", [
                "forced_current", "forced_1", "forced_2",
                "forced_alt", "boundary"])])

    def test_failed_set_route_does_not_consume_suffix_branch_or_attempt(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-suffix"})
        vehicle.set_route_failures_remaining["eligible-suffix"] = 1
        released = {"eligible-suffix"}
        refresh_key = ("eligible-suffix", "p", "short")
        expected_route = ["p", "short", "alt", "boundary"]

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 1.0), [])

        self.assertEqual(vehicle.routes["eligible-suffix"], [
            "p", "short", "blocked", "boundary"])
        self.assertEqual(vehicle.set_route_attempts, [
            ("eligible-suffix", expected_route)])
        self.assertEqual(vehicle.set_route_calls, [])
        self.assertEqual(
            continuator._suffix_attempt_counts.get(refresh_key, 0), 0)
        self.assertNotIn(
            ("short", "alt"),
            continuator._tried_suffix_branches[refresh_key])
        self.assertIn(
            "eligible-suffix", continuator.stall_recovery_failures)

        # The still-persistent stop retries the identical unapplied branch.
        # Its first successful TraCI mutation is still recovery attempt one.
        events = continuator.recover_persistent_stops(released, 1.05)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["suffix_recovery_attempt"], 1)
        self.assertEqual(events[0]["new_route_remainder"], expected_route)
        self.assertEqual(vehicle.set_route_attempts, [
            ("eligible-suffix", expected_route),
            ("eligible-suffix", expected_route),
        ])
        self.assertEqual(vehicle.set_route_calls, [
            ("eligible-suffix", expected_route)])
        self.assertEqual(
            continuator._suffix_attempt_counts[refresh_key], 1)
        self.assertNotIn(
            "eligible-suffix", continuator.stall_recovery_failures)

    def test_divergent_suffix_cannot_reuse_forbidden_downstream_transition(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        edge_ids = (
            "forbidden_current", "forbidden_entry", "forbidden_old",
            "forbidden_rejoin_choice", "forbidden_rejoin",
            "forbidden_failed", "forbidden_safe")
        for edge_id in edge_ids:
            network.edges[edge_id] = _FakeEdge(edge_id)
        network.edges["forbidden_current"].outgoing["passenger"] = {
            network.edges["forbidden_entry"]: [_FakeConnection("s")],
        }
        network.edges["forbidden_entry"].outgoing["passenger"] = {
            network.edges["forbidden_old"]: [_FakeConnection("s")],
            network.edges["forbidden_rejoin_choice"]: [
                _FakeConnection("l")],
            network.edges["forbidden_safe"]: [_FakeConnection("r")],
        }
        network.edges["forbidden_old"].outgoing["passenger"] = {
            network.edges["boundary"]: [_FakeConnection("s")],
        }
        network.edges["forbidden_rejoin_choice"].outgoing["passenger"] = {
            network.edges["forbidden_rejoin"]: [_FakeConnection("s")],
        }
        network.edges["forbidden_rejoin"].outgoing["passenger"] = {
            network.edges["forbidden_failed"]: [_FakeConnection("s")],
        }
        network.edges["forbidden_failed"].outgoing["passenger"] = {
            network.edges["boundary"]: [_FakeConnection("s")],
        }
        network.edges["forbidden_safe"].outgoing["passenger"] = {
            network.edges["boundary"]: [_FakeConnection("s")],
        }
        vehicle_id = "eligible-forbidden-rejoin"
        vehicle.add_vehicle(
            vehicle_id,
            ["forbidden_current", "forbidden_entry",
             "forbidden_old", "boundary"],
            [("forbidden_entry_0", False, True, False,
              ":forbidden_entry", "G", "s", 20.0)])
        continuator = SumoRouteContinuator(
            network, vehicle, eligible_vehicle_ids={vehicle_id}, seed=103)
        refresh_key = (
            vehicle_id, "forbidden_current", "forbidden_entry")
        forbidden_transition = (
            "forbidden_rejoin", "forbidden_failed")
        # This transition represents downstream evidence remembered from an
        # earlier failed suffix. The new left divergence is viable only if it
        # is allowed to reconstruct that same failed transition.
        continuator._tried_suffix_branches[refresh_key] = {
            forbidden_transition}
        released = {vehicle_id}

        self.assertTrue(continuator._has_viable_continuation(
            "forbidden_rejoin_choice", "passenger"))
        self.assertFalse(continuator._has_viable_continuation(
            "forbidden_rejoin_choice", "passenger",
            {forbidden_transition}))
        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        events = continuator.recover_persistent_stops(released, 1.0)

        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["new_route_remainder"], [
            "forbidden_current", "forbidden_entry",
            "forbidden_safe", "boundary"])
        transitions = set(zip(
            events[0]["new_route_remainder"],
            events[0]["new_route_remainder"][1:]))
        self.assertNotIn(forbidden_transition, transitions)
        self.assertNotIn("forbidden_rejoin_choice",
                         events[0]["new_route_remainder"])

    def test_suffix_retries_use_three_unique_untried_branches_only(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        branch_edges = ("retry_old", "retry_alt_1",
                        "retry_alt_2", "retry_alt_3")
        for edge_id in ("retry_current", "retry_entry") + branch_edges:
            network.edges[edge_id] = _FakeEdge(edge_id)
        network.edges["retry_current"].outgoing["passenger"] = {
            network.edges["retry_entry"]: [_FakeConnection("s")],
        }
        network.edges["retry_entry"].outgoing["passenger"] = {
            network.edges[edge_id]: [_FakeConnection("s")]
            for edge_id in branch_edges
        }
        for edge_id in branch_edges:
            network.edges[edge_id].outgoing["passenger"] = {
                network.edges["boundary"]: [_FakeConnection("s")],
            }
        vehicle.add_vehicle(
            "eligible-unique-retries",
            ["retry_current", "retry_entry", "retry_old", "boundary"],
            [("retry_entry_0", False, True, False,
              ":retry_entry", "G", "s", 20.0)])
        continuator = SumoRouteContinuator(
            network, vehicle,
            eligible_vehicle_ids={"eligible-unique-retries"}, seed=103)
        released = {"eligible-unique-retries"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        recovery_events = list(
            continuator.recover_persistent_stops(released, 1.0))
        for stopped_at, recover_at in ((1.1, 2.1), (2.2, 3.2)):
            self.assertEqual(
                continuator.recover_persistent_stops(
                    released, stopped_at), [])
            recovery_events.extend(
                continuator.recover_persistent_stops(
                    released, recover_at))

        self.assertEqual(len(recovery_events), 3)
        selected_branches = []
        for attempt, event in enumerate(recovery_events, 1):
            self.assertEqual(
                event["mutation_type"],
                "persistent_stop_suffix_recovery")
            self.assertEqual(event["suffix_recovery_attempt"], attempt)
            self.assertEqual(event["new_route_remainder"][:2], [
                "retry_current", "retry_entry"])
            selected_branches.append(event["new_route_remainder"][2])
        self.assertEqual(len(set(selected_branches)), 3)
        self.assertEqual(set(selected_branches), {
            "retry_alt_1", "retry_alt_2", "retry_alt_3"})

        # A fourth persistent-stop window cannot revisit any failed suffix or
        # exceed the configured three suffix mutations.
        self.assertEqual(
            continuator.recover_persistent_stops(released, 3.3), [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 4.3), [])
        self.assertEqual(len(vehicle.set_route_calls), 3)
        self.assertEqual(
            continuator.metadata()["persistent_stop_recovery_count"], 3)

    def test_reachable_cycle_is_viable_but_diamond_to_interior_sink_is_not(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        for edge_id in (
                "cycle_start", "cycle_a", "cycle_b",
                "diamond_start", "diamond_left", "diamond_right",
                "diamond_join", "diamond_sink"):
            network.edges[edge_id] = _FakeEdge(edge_id)
        network.edges["cycle_start"].outgoing["passenger"] = {
            network.edges["cycle_a"]: [_FakeConnection("s")],
        }
        network.edges["cycle_a"].outgoing["passenger"] = {
            network.edges["cycle_b"]: [_FakeConnection("s")],
        }
        network.edges["cycle_b"].outgoing["passenger"] = {
            network.edges["cycle_a"]: [_FakeConnection("s")],
        }
        network.edges["diamond_start"].outgoing["passenger"] = {
            network.edges["diamond_left"]: [_FakeConnection("l")],
            network.edges["diamond_right"]: [_FakeConnection("r")],
        }
        for edge_id in ("diamond_left", "diamond_right"):
            network.edges[edge_id].outgoing["passenger"] = {
                network.edges["diamond_join"]: [_FakeConnection("s")],
            }
        network.edges["diamond_join"].outgoing["passenger"] = {
            network.edges["diamond_sink"]: [_FakeConnection("s")],
        }
        continuator = SumoRouteContinuator(
            network, vehicle, eligible_vehicle_ids=set(), seed=103)

        self.assertTrue(continuator._has_viable_continuation(
            "cycle_start", "passenger"))
        cycle_path, _decisions = continuator._seeded_viable_path(
            "cycle-test", "cycle_start", "passenger")
        self.assertEqual(
            cycle_path, ["cycle_start", "cycle_a", "cycle_b", "cycle_a"])
        self.assertFalse(continuator._has_viable_continuation(
            "diamond_start", "passenger"))
        rejected_path, _decisions = continuator._seeded_viable_path(
            "diamond-test", "diamond_start", "passenger")
        self.assertIsNone(rejected_path)

    def test_large_shallow_dag_is_not_mistaken_for_a_cycle(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        entry = _FakeEdge("large_dag_entry")
        network.edges[entry.getID()] = entry
        chain = []
        for index in range(1100):
            edge = _FakeEdge("large_dag_%04d" % index)
            network.edges[edge.getID()] = edge
            chain.append(edge)

        # Every chain node is one hop from the entry, keeping its discovered
        # shortest depth below the 64-edge horizon. The additional 1,100-edge
        # chain makes a recursive cycle detector overflow even though this is
        # a finite acyclic graph ending at an interior sink.
        entry.outgoing["passenger"] = dict(
            (edge, [_FakeConnection("s")]) for edge in chain)
        for current_edge, next_edge in zip(chain, chain[1:]):
            current_edge.outgoing["passenger"] = {
                next_edge: [_FakeConnection("s")],
            }
        continuator = SumoRouteContinuator(
            network, vehicle, eligible_vehicle_ids=set(), seed=103)

        self.assertGreater(len(chain), 1000)
        self.assertFalse(continuator._has_viable_continuation(
            "large_dag_entry", "passenger"))

    def test_deep_acyclic_chain_beyond_search_depth_is_not_viable(self):
        vehicle = _FakeVehicleDomain()
        network = _FakeNetwork()
        chain = []
        # The final node is an interior sink, deliberately two transitions
        # beyond the configured 64-edge search depth. Reaching the search
        # horizon is not evidence of a boundary exit or a directed cycle.
        for index in range(67):
            edge = _FakeEdge("deep_dag_%02d" % index)
            network.edges[edge.getID()] = edge
            chain.append(edge)
        for current_edge, next_edge in zip(chain, chain[1:]):
            current_edge.outgoing["passenger"] = {
                next_edge: [_FakeConnection("s")],
            }
        continuator = SumoRouteContinuator(
            network, vehicle, eligible_vehicle_ids=set(), seed=103,
            continuation_search_depth_edges=64)

        self.assertFalse(continuator._has_viable_continuation(
            chain[0].getID(), "passenger"))
        witness, decisions = continuator._seeded_viable_path(
            "deep-dag-test", chain[0].getID(), "passenger")
        self.assertIsNone(witness)
        self.assertEqual(decisions, [])

    def test_long_enough_next_edge_ignores_later_conflict_as_cause(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-suffix"})
        # Exactly enough storage for the 4.7 m vehicle plus its 2.5 m
        # minimum gap. The conflict after this edge cannot explain a hold on
        # p, so the ordinary one-second unexplained-stop timer applies.
        continuator.network.edges["short"].length = 7.2
        released = {"eligible-suffix"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.99), [])
        events = continuator.recover_persistent_stops(released, 1.0)

        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(
            event["mutation_type"], "persistent_stop_suffix_recovery")
        self.assertEqual(event["stopped_duration_s"], 1.0)
        self.assertNotIn(
            "persistent_downstream_blocker", event["evidence"])
        self.assertEqual(
            [item["planned_edge"]
             for item in event["evidence"]["next_links_checked"]],
            ["short"])
        self.assertEqual(event["new_route_remainder"], [
            "p", "short", "alt", "boundary"])

    def test_rejects_an_interior_sink_as_immediate_alternative(self):
        vehicle, continuator = self._continuator(
            eligible={"eligible-stub"})
        released = {"eligible-stub"}

        self.assertEqual(
            continuator.recover_persistent_stops(released, 0.0), [])
        self.assertEqual(
            continuator.recover_persistent_stops(released, 1.0), [])

        self.assertEqual(vehicle.set_route_calls, [])
        unresolved = continuator.metadata()[
            "persistent_stop_recovery_unresolved"]["eligible-stub"]
        self.assertEqual(unresolved["reason"],
                         "no_viable_alternative_outgoing")
        self.assertEqual(
            unresolved["rejected_interior_sink_edges"], ["stub"])

    def test_legitimate_sumo_stop_reasons_prevent_recovery(self):
        cases = {
            "scheduled_stop": lambda vehicle: vehicle.stop_states.__setitem__(
                "eligible-remaining", 1),
            "close_leader": lambda vehicle: vehicle.leaders.__setitem__(
                "eligible-remaining", ("leader", 14.0)),
            "red_tls": lambda vehicle: vehicle.next_tls.__setitem__(
                "eligible-remaining", [("tls", 0, 10.0, "r")]),
            "yellow_tls": lambda vehicle: vehicle.next_tls.__setitem__(
                "eligible-remaining", [("tls", 0, 10.0, "y")]),
            "closed_next_link": lambda vehicle: vehicle.next_links.__setitem__(
                "eligible-remaining",
                [("b_0", None, None, False, False, "G")]),
            "foe_on_next_link": lambda vehicle: vehicle.next_links.__setitem__(
                "eligible-remaining",
                [("b_0", None, None, True, True, "G")]),
            "installed_layout_closed": lambda vehicle: (
                vehicle.next_links.__setitem__(
                    "eligible-remaining",
                    [("b_0", False, False, False, ":b_0", "G")])),
            "installed_layout_foe": lambda vehicle: (
                vehicle.next_links.__setitem__(
                    "eligible-remaining",
                    [("b_0", False, True, True, ":b_0", "G")])),
        }
        for name, configure in cases.items():
            with self.subTest(name=name):
                vehicle, continuator = self._continuator()
                configure(vehicle)
                released = {"eligible-remaining"}

                self.assertEqual(
                    continuator.recover_persistent_stops(
                        released, 0.0), [])
                self.assertEqual(
                    continuator.recover_persistent_stops(
                        released, 30.0), [])
                self.assertEqual(vehicle.routes["eligible-remaining"], [
                    "a", "b"])
                self.assertEqual(vehicle.set_route_calls, [])


if __name__ == "__main__":
    unittest.main()
