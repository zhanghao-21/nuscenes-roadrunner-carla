"""Runtime safeguards for finite data-seeded SUMO routes."""

from collections import deque
import math
import random


def _connection_direction(connection):
    getter = getattr(connection, "getDirection", None)
    if getter is None:
        return None
    try:
        return getter()
    except Exception:
        return None


class SumoRouteContinuator:
    """Keep owned SUMO movers on valid routes without forcing their motion.

    Normal continuation only appends beyond the already prepared route. A
    separate, conservative recovery may first release recorded speed authority
    and later replace the remaining route if the now-autonomous mover remains
    persistently stopped. Scheduled stops, signals,
    close leaders, and immediate link conflicts remain hard guards. A conflict
    beyond a too-short storage edge may refresh only the downstream suffix
    after a longer guarded wait. Neither path commands a positive speed,
    changes speed mode, or bypasses SUMO safety.

    Only the data-seeded background IDs supplied by the caller are eligible;
    CARLA-owned actors mirrored into SUMO are never queried or modified.
    Prepared ``network_terminal`` IDs are explicit source-map road ends and
    are reported separately instead of being treated as unresolved interiors.
    """

    def __init__(self, network, vehicle_domain, eligible_vehicle_ids, seed=0,
                 allow_uturns=False, recovery_wait_s=1.0,
                 recovery_stop_speed_mps=0.1,
                 recovery_resume_speed_mps=0.3,
                 recovery_progress_distance_m=0.5,
                 recovery_leader_clearance_m=15.0,
                 recovery_tls_distance_m=15.0,
                 recovery_downstream_blocked_wait_s=1.0,
                 maximum_suffix_recovery_attempts=3,
                 boundary_terminal_tolerance_m=15.0,
                 continuation_search_depth_edges=64,
                 accepted_network_terminal_vehicle_ids=None):
        self.network = network
        self.vehicle = vehicle_domain
        self.eligible_vehicle_ids = set(
            str(value) for value in eligible_vehicle_ids)
        self.accepted_network_terminal_vehicle_ids = ({
            str(value)
            for value in (accepted_network_terminal_vehicle_ids or ())
        } & self.eligible_vehicle_ids)
        self.seed = int(seed)
        self.allow_uturns = bool(allow_uturns)
        self.recovery_wait_s = self._positive_setting(
            "recovery_wait_s", recovery_wait_s)
        self.recovery_stop_speed_mps = self._nonnegative_setting(
            "recovery_stop_speed_mps", recovery_stop_speed_mps)
        self.recovery_resume_speed_mps = self._positive_setting(
            "recovery_resume_speed_mps", recovery_resume_speed_mps)
        if (self.recovery_resume_speed_mps <=
                self.recovery_stop_speed_mps):
            raise ValueError(
                "recovery_resume_speed_mps must exceed "
                "recovery_stop_speed_mps")
        self.recovery_progress_distance_m = self._positive_setting(
            "recovery_progress_distance_m",
            recovery_progress_distance_m)
        self.recovery_leader_clearance_m = self._nonnegative_setting(
            "recovery_leader_clearance_m",
            recovery_leader_clearance_m)
        self.recovery_tls_distance_m = self._nonnegative_setting(
            "recovery_tls_distance_m", recovery_tls_distance_m)
        self.recovery_downstream_blocked_wait_s = self._positive_setting(
            "recovery_downstream_blocked_wait_s",
            recovery_downstream_blocked_wait_s)
        self.maximum_suffix_recovery_attempts = self._positive_integer_setting(
            "maximum_suffix_recovery_attempts",
            maximum_suffix_recovery_attempts)
        self.boundary_terminal_tolerance_m = self._nonnegative_setting(
            "boundary_terminal_tolerance_m",
            boundary_terminal_tolerance_m)
        self.continuation_search_depth_edges = self._positive_integer_setting(
            "continuation_search_depth_edges",
            continuation_search_depth_edges)
        self._network_boundary = self._read_network_boundary()
        self._viability_cache = {}
        self.decision_counts = {}
        self.events = []
        self.failures = {}
        self.no_outgoing = {}
        self.boundary_road_ends = {}
        self.accepted_network_terminals = {}
        self.unresolved_interior_route_tails = {}
        self.nonviable_outgoing = {}
        # Retained as an empty compatibility stream. Releasing a terminal
        # recorded speed tail no longer mutates the route immediately.
        self.terminal_release_events = []
        self.stall_recovery_events = []
        self.speed_authority_release_events = []
        self.stall_recovery_failures = {}
        self.stall_recovery_unresolved = {}
        self._stall_states = {}
        self._suffix_attempt_counts = {}
        self._tried_suffix_branches = {}
        self._tried_immediate_branches = {}
        self._unresolved_edges = set()
        self._event_sequence = 0

    @staticmethod
    def _positive_setting(name, value):
        try:
            converted = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("%s must be positive" % name)
        if not math.isfinite(converted) or converted <= 0.0:
            raise ValueError("%s must be positive" % name)
        return converted

    @staticmethod
    def _nonnegative_setting(name, value):
        try:
            converted = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("%s must be non-negative" % name)
        if not math.isfinite(converted) or converted < 0.0:
            raise ValueError("%s must be non-negative" % name)
        return converted

    @staticmethod
    def _positive_integer_setting(name, value):
        if isinstance(value, bool):
            raise ValueError("%s must be a positive integer" % name)
        try:
            converted = int(value)
            exact = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("%s must be a positive integer" % name)
        if (not math.isfinite(exact) or exact != converted or
                converted <= 0):
            raise ValueError("%s must be a positive integer" % name)
        return converted

    def _read_network_boundary(self):
        getter = getattr(self.network, "getBoundary", None)
        if getter is None:
            return None
        try:
            boundary = tuple(float(value) for value in getter())
        except Exception:
            return None
        if (len(boundary) < 4 or
                not all(math.isfinite(value) for value in boundary[:4])):
            return None
        return boundary[:4]

    def _record_event(self, stream, event):
        event["sequence"] = self._event_sequence
        self._event_sequence += 1
        stream.append(event)
        return event

    def _outgoing_edge_ids(self, edge_id, vehicle_class):
        edge = self.network.getEdge(edge_id)
        outgoing = edge.getAllowedOutgoing(vehicle_class)
        candidates = []
        for target_edge, connections in outgoing.items():
            connections = list(connections or [])
            if not self.allow_uturns and connections:
                permitted = [
                    connection for connection in connections
                    if _connection_direction(connection) != "t"
                ]
                if not permitted:
                    continue
            candidates.append(str(target_edge.getID()))
        return sorted(set(candidates))

    def _is_boundary_terminal(self, edge_id):
        """Return whether a sink edge ends at the usable network boundary."""
        if self._network_boundary is None:
            return True
        try:
            coordinate = self.network.getEdge(
                edge_id).getToNode().getCoord()
            x_value = float(coordinate[0])
            y_value = float(coordinate[1])
        except Exception:
            # A network adapter without geometry cannot safely distinguish a
            # conversion stub from a genuine exit; retain legacy behavior.
            return True
        x_min, y_min, x_max, y_max = self._network_boundary
        distance = min(
            abs(x_value - x_min), abs(x_value - x_max),
            abs(y_value - y_min), abs(y_value - y_max))
        return distance <= self.boundary_terminal_tolerance_m

    @staticmethod
    def _forbidden_branch_set(forbidden_branches):
        return set(
            (str(from_edge), str(to_edge))
            for from_edge, to_edge in (forbidden_branches or ()))

    def _continuation_outgoing(
            self, edge_id, vehicle_class, forbidden_branches=None):
        forbidden = self._forbidden_branch_set(forbidden_branches)
        return [
            candidate for candidate in self._outgoing_edge_ids(
                edge_id, vehicle_class)
            if (str(edge_id), candidate) not in forbidden
        ]

    def _has_viable_continuation(
            self, edge_id, vehicle_class, forbidden_branches=None):
        """Reject paths that can only terminate at an interior network stub.

        A boundary sink is a valid road exit. A reachable cycle is also valid
        because the actor can keep driving until the simulation ends. A finite
        acyclic path to an interior sink is never accepted, regardless of its
        length.
        """
        if self._network_boundary is None:
            return True
        forbidden = self._forbidden_branch_set(forbidden_branches)
        cache_key = (str(edge_id), str(vehicle_class))
        if not forbidden and cache_key in self._viability_cache:
            return self._viability_cache[cache_key]
        start_edge = str(edge_id)
        pending = deque([(start_edge, 0)])
        discovered_depth = {start_edge: 0}
        adjacency = {}
        while pending:
            candidate_edge, depth = pending.popleft()
            outgoing = self._continuation_outgoing(
                candidate_edge, vehicle_class, forbidden)
            adjacency[candidate_edge] = outgoing
            if not outgoing and self._is_boundary_terminal(candidate_edge):
                if not forbidden:
                    self._viability_cache[cache_key] = True
                return True
            for next_edge in outgoing:
                next_depth = depth + 1
                if (next_edge not in discovered_depth or
                        next_depth < discovered_depth[next_edge]):
                    discovered_depth[next_edge] = next_depth
                    pending.append((next_edge, next_depth))

        # Kahn's iterative elimination distinguishes a real directed cycle
        # from a diamond-shaped merge without risking Python recursion limits.
        reachable = set(adjacency)
        for outgoing in adjacency.values():
            reachable.update(outgoing)
        indegree = dict((candidate, 0) for candidate in reachable)
        for outgoing in adjacency.values():
            for next_edge in outgoing:
                indegree[next_edge] += 1
        roots = deque(sorted(
            candidate for candidate, degree in indegree.items()
            if degree == 0))
        removed = 0
        while roots:
            candidate_edge = roots.popleft()
            removed += 1
            for next_edge in adjacency.get(candidate_edge, ()):
                indegree[next_edge] -= 1
                if indegree[next_edge] == 0:
                    roots.append(next_edge)
        result = removed < len(reachable)
        if not forbidden:
            self._viability_cache[cache_key] = result
        return result

    def _viable_candidates(
            self, candidates, vehicle_class, forbidden_branches=None):
        viable = []
        rejected = []
        for edge_id in candidates:
            if self._has_viable_continuation(
                    edge_id, vehicle_class, forbidden_branches):
                viable.append(edge_id)
            else:
                rejected.append(edge_id)
        return viable, rejected

    def _seeded_candidate_order(
            self, vehicle_id, current_edge, candidates, decision):
        """Return a reproducible random order without committing a decision."""
        candidates = sorted(set(str(value) for value in candidates))
        if len(candidates) <= 1:
            return candidates
        generator = random.Random(
            "%s|%s|%s|%s" % (
                self.seed, vehicle_id, current_edge, decision))
        first_index = generator.randrange(len(candidates))
        first = candidates.pop(first_index)
        generator.shuffle(candidates)
        return [first] + candidates

    @staticmethod
    def _reconstruct_path(predecessor, target_edge):
        path = [target_edge]
        while predecessor[path[-1]] is not None:
            path.append(predecessor[path[-1]])
        path.reverse()
        return path

    def _viable_path_witness(
            self, vehicle_id, start_edge, vehicle_class,
            forbidden_branches=None, decision_base=0):
        """Return a concrete boundary or cycle path if one exists."""
        start_edge = str(start_edge)
        forbidden = self._forbidden_branch_set(forbidden_branches)
        pending = deque([start_edge])
        predecessor = {start_edge: None}
        discovered_depth = {start_edge: 0}
        discovery_order = [start_edge]
        adjacency = {}
        while pending:
            current_edge = pending.popleft()
            depth = discovered_depth[current_edge]
            outgoing = self._continuation_outgoing(
                current_edge, vehicle_class, forbidden)
            outgoing = self._seeded_candidate_order(
                vehicle_id, current_edge, outgoing,
                int(decision_base) + depth)
            adjacency[current_edge] = outgoing
            if not outgoing:
                if self._is_boundary_terminal(current_edge):
                    return self._reconstruct_path(
                        predecessor, current_edge)
                continue
            for next_edge in outgoing:
                if next_edge in predecessor:
                    continue
                predecessor[next_edge] = current_edge
                discovered_depth[next_edge] = depth + 1
                discovery_order.append(next_edge)
                pending.append(next_edge)

        # No boundary target was reached. Find an actual directed
        # cycle with an explicit DFS stack, then join the BFS prefix to it.
        color = {}
        parent = {}
        for root_edge in discovery_order:
            if color.get(root_edge, 0) != 0:
                continue
            color[root_edge] = 1
            parent[root_edge] = None
            stack = [(root_edge, 0)]
            while stack:
                current_edge, next_index = stack[-1]
                outgoing = adjacency.get(current_edge, ())
                if next_index >= len(outgoing):
                    color[current_edge] = 2
                    stack.pop()
                    continue
                next_edge = outgoing[next_index]
                stack[-1] = (current_edge, next_index + 1)
                state = color.get(next_edge, 0)
                if state == 0:
                    parent[next_edge] = current_edge
                    color[next_edge] = 1
                    stack.append((next_edge, 0))
                    continue
                if state != 1:
                    continue
                cycle_reverse = [current_edge]
                while cycle_reverse[-1] != next_edge:
                    cycle_reverse.append(parent[cycle_reverse[-1]])
                cycle = list(reversed(cycle_reverse)) + [next_edge]
                prefix = self._reconstruct_path(predecessor, next_edge)
                combined = prefix + cycle[1:]
                first_seen = {}
                for index, edge_id in enumerate(combined):
                    if edge_id in first_seen:
                        return combined[:index + 1]
                    first_seen[edge_id] = index
                return combined
        return None

    @staticmethod
    def _path_choice_records(path, starting_decision):
        return [
            {
                "from_edge": from_edge,
                "to_edge": to_edge,
                "decision": int(starting_decision) + index,
            }
            for index, (from_edge, to_edge) in enumerate(
                zip(path, path[1:]))
        ]

    def _seeded_viable_path(
            self, vehicle_id, start_edge, vehicle_class,
            forbidden_branches=None):
        """Build and commit a connected boundary or cycle path."""
        starting_decision = self.decision_counts.get(vehicle_id, 0)
        path = self._viable_path_witness(
            vehicle_id, start_edge, vehicle_class,
            forbidden_branches=forbidden_branches,
            decision_base=starting_decision)
        if path is None:
            return None, []
        choices = self._path_choice_records(path, starting_decision)
        self.decision_counts[vehicle_id] = (
            starting_decision + len(choices))
        return path, choices

    def _prefer_longest_edges(self, candidates):
        """Avoid tiny conversion artifacts, retaining seeded ties."""
        candidates = list(candidates)
        if len(candidates) <= 1:
            return candidates
        lengths = {}
        for edge_id in candidates:
            try:
                lengths[edge_id] = float(
                    self.network.getEdge(edge_id).getLength())
            except Exception:
                lengths[edge_id] = 0.0
        maximum_length = max(lengths.values())
        return [
            edge_id for edge_id in candidates
            if abs(lengths[edge_id] - maximum_length) <= 1.0e-9
        ]

    def _remember_failed_suffix_branches(
            self, suffix, vehicle_class, tried_branches):
        """Remember every viable branch transition in a failed route."""
        suffix = [str(edge_id) for edge_id in suffix]
        for current_edge, next_edge in zip(suffix, suffix[1:]):
            outgoing = self._outgoing_edge_ids(
                current_edge, vehicle_class)
            viable, _rejected = self._viable_candidates(
                outgoing, vehicle_class)
            if len(viable) > 1 and next_edge in viable:
                tried_branches.add((current_edge, next_edge))

    def _seeded_divergent_suffix(
            self, vehicle_id, existing_suffix, vehicle_class,
            tried_branches):
        """Preserve forced edges and choose the first untried viable branch.

        The immediate planned edge is always retained. If that edge has only
        one viable successor, recovery follows the forced prefix until it finds
        a later branch. Previously failed branch transitions are excluded so
        repeated recovery cannot oscillate between old suffixes.
        """
        existing_suffix = [str(edge_id) for edge_id in existing_suffix]
        if len(existing_suffix) < 2:
            return None, [], None, None, None
        starting_decision = self.decision_counts.get(vehicle_id, 0)
        for divergence_index in range(len(existing_suffix) - 1):
            current_edge = existing_suffix[divergence_index]
            existing_next = existing_suffix[divergence_index + 1]
            outgoing = self._outgoing_edge_ids(
                current_edge, vehicle_class)
            viable, _rejected = self._viable_candidates(
                outgoing, vehicle_class, tried_branches)
            alternatives = [
                edge_id for edge_id in viable
                if (edge_id != existing_next and
                    (current_edge, edge_id) not in tried_branches)
            ]
            if not alternatives:
                continue
            alternatives = self._prefer_longest_edges(alternatives)
            alternatives = self._seeded_candidate_order(
                vehicle_id, current_edge, alternatives,
                starting_decision)
            for selected_edge in alternatives:
                continuation = self._viable_path_witness(
                    vehicle_id, selected_edge, vehicle_class,
                    forbidden_branches=tried_branches,
                    decision_base=starting_decision + 1)
                if continuation is None:
                    continue
                continuation_choices = self._path_choice_records(
                    continuation, starting_decision + 1)
                new_suffix = (
                    existing_suffix[:divergence_index + 1] + continuation)
                decisions = [{
                    "from_edge": current_edge,
                    "to_edge": selected_edge,
                    "decision": starting_decision,
                    "suffix_divergence": True,
                }] + continuation_choices
                branch = (current_edge, selected_edge)
                next_decision = starting_decision + len(decisions)
                return (new_suffix, decisions, branch, divergence_index,
                        next_decision)
        return None, [], None, None, None

    def _expand_cycle_lookahead(self, path):
        """Repeat a witnessed cycle so a finite SUMO route stays well ahead."""
        path = [str(edge_id) for edge_id in path]
        if len(path) < 2 or path[-1] not in path[:-1]:
            return path
        cycle_start = path[:-1].index(path[-1])
        cycle_steps = path[cycle_start + 1:]
        if not cycle_steps:
            return path
        minimum_edges = max(
            len(path), self.continuation_search_depth_edges)
        while len(path) < minimum_edges:
            remaining = minimum_edges - len(path)
            path.extend(cycle_steps[:remaining])
        return path

    def _propose_continuation_path(
            self, vehicle_id, from_edge, candidates, vehicle_class,
            forbidden_branches=None, prefer_longest=False):
        """Propose a seeded path ending at a boundary or reusable cycle."""
        viable, rejected = self._viable_candidates(
            candidates, vehicle_class, forbidden_branches)
        if prefer_longest:
            viable = self._prefer_longest_edges(viable)
        starting_decision = self.decision_counts.get(vehicle_id, 0)
        ordered = self._seeded_candidate_order(
            vehicle_id, from_edge, viable, starting_decision)
        for selected_edge in ordered:
            witness = self._viable_path_witness(
                vehicle_id, selected_edge, vehicle_class,
                forbidden_branches=forbidden_branches,
                decision_base=starting_decision + 1)
            if witness is None:
                continue
            witness = self._expand_cycle_lookahead(witness)
            decisions = [{
                "from_edge": str(from_edge),
                "to_edge": selected_edge,
                "decision": starting_decision,
            }] + self._path_choice_records(
                witness, starting_decision + 1)
            return (witness, decisions, rejected,
                    starting_decision + len(decisions))
        return None, [], rejected, None

    def _choose(self, vehicle_id, current_edge, candidates, commit=True):
        decision = self.decision_counts.get(vehicle_id, 0)
        generator = random.Random(
            "%s|%s|%s|%s" % (
                self.seed, vehicle_id, current_edge, decision))
        selected = candidates[generator.randrange(len(candidates))]
        if commit:
            self.decision_counts[vehicle_id] = decision + 1
        return selected, decision

    def extend_active_routes(self, simulation_time=None):
        """Keep every eligible route ahead to a boundary or reusable cycle.

        The penultimate-edge lookahead matters for very short SUMO edges: a
        vehicle can otherwise enter and leave its final edge inside one
        simulation tick and disappear before a post-tick repair can run. One
        maintenance operation appends a complete boundary/cycle witness rather
        than another finite one-edge tail. An interior sink is avoided from its
        predecessor whenever a viable alternative exists.
        """
        new_events = []
        active_ids = set(str(value) for value in self.vehicle.getIDList())
        for vehicle_id in sorted(active_ids.intersection(
                self.eligible_vehicle_ids)):
            try:
                route = list(self.vehicle.getRoute(vehicle_id))
                route_index = int(self.vehicle.getRouteIndex(vehicle_id))
                if (not route or route_index < 0 or
                        route_index >= len(route) or
                        route_index < len(route) - 2):
                    continue
                current_edge = str(route[route_index])
                if current_edge.startswith(":"):
                    continue
                get_road_id = getattr(self.vehicle, "getRoadID", None)
                if get_road_id is not None:
                    actual_edge = str(get_road_id(vehicle_id))
                    # setRoute requires its first edge to be the vehicle's
                    # current normal edge. Wait through internal junction
                    # lanes and any transient route/road mismatch.
                    if (actual_edge.startswith(":") or
                            actual_edge != current_edge):
                        continue
                # Append beyond the prepared tail without replacing any of
                # its still-untraversed evidence-backed edges.
                tail_edge = str(route[-1])
                if tail_edge.startswith(":"):
                    continue
                vehicle_class = self.vehicle.getVehicleClass(vehicle_id)
                remaining_route = [
                    str(edge_id) for edge_id in route[route_index:]
                ]
                outgoing = self._outgoing_edge_ids(
                    tail_edge, vehicle_class)
                appended, decisions, rejected, next_decision = (
                    self._propose_continuation_path(
                        vehicle_id, tail_edge, outgoing, vehicle_class))
                if appended is not None:
                    self.vehicle.setRoute(
                        vehicle_id, remaining_route + appended)
                    self.decision_counts[vehicle_id] = next_decision
                    event = self._record_event(self.events, {
                        "mutation_type": "route_tail_extension",
                        "vehicle_id": vehicle_id,
                        "from_edge": tail_edge,
                        "to_edge": appended[-1],
                        "selected_next_edge": appended[0],
                        "appended_edges": appended,
                        "trigger_edge": current_edge,
                        "proactive": route_index == len(route) - 2,
                        "decision": decisions[0]["decision"],
                        "decisions": decisions,
                        "simulation_time_s": (
                            None if simulation_time is None
                            else float(simulation_time)),
                    })
                    new_events.append(event)
                    self.failures.pop(vehicle_id, None)
                    self.no_outgoing.pop(vehicle_id, None)
                    self.boundary_road_ends.pop(vehicle_id, None)
                    self.accepted_network_terminals.pop(vehicle_id, None)
                    self.unresolved_interior_route_tails.pop(vehicle_id, None)
                    self.nonviable_outgoing.pop(vehicle_id, None)
                    continue

                if rejected:
                    self.nonviable_outgoing[vehicle_id] = {
                        "from_edge": tail_edge,
                        "rejected_edges": rejected,
                        "reason": "no_boundary_or_cycle_continuation",
                    }
                if (not outgoing and vehicle_id in
                        self.accepted_network_terminal_vehicle_ids):
                    self.no_outgoing[vehicle_id] = tail_edge
                    self.accepted_network_terminals[vehicle_id] = tail_edge
                    self.boundary_road_ends.pop(vehicle_id, None)
                    self.unresolved_interior_route_tails.pop(vehicle_id, None)
                    continue
                if not outgoing and self._is_boundary_terminal(tail_edge):
                    self.no_outgoing[vehicle_id] = tail_edge
                    self.boundary_road_ends[vehicle_id] = tail_edge
                    self.accepted_network_terminals.pop(vehicle_id, None)
                    self.unresolved_interior_route_tails.pop(vehicle_id, None)
                    continue

                # The prepared final edge is an interior dead end (or can only
                # lead to one). At the penultimate edge, preserve the current
                # edge and choose a different viable successor before entry.
                planned_next = (
                    str(route[route_index + 1])
                    if route_index + 1 < len(route) else None)
                alternatives = []
                if current_edge != tail_edge and planned_next is not None:
                    alternatives = [
                        edge_id for edge_id in self._outgoing_edge_ids(
                            current_edge, vehicle_class)
                        if edge_id != planned_next
                    ]
                replacement, replacement_decisions, replacement_rejected, (
                    replacement_next_decision) = (
                        self._propose_continuation_path(
                            vehicle_id, current_edge, alternatives,
                            vehicle_class, prefer_longest=True))
                if replacement is not None:
                    new_route = [current_edge] + replacement
                    self.vehicle.setRoute(vehicle_id, new_route)
                    self.decision_counts[vehicle_id] = (
                        replacement_next_decision)
                    event = self._record_event(self.events, {
                        "mutation_type": "interior_route_tail_avoidance",
                        "vehicle_id": vehicle_id,
                        "from_edge": current_edge,
                        "original_next_edge": planned_next,
                        "original_terminal_edge": tail_edge,
                        "to_edge": replacement[-1],
                        "selected_next_edge": replacement[0],
                        "appended_edges": replacement,
                        "decision": replacement_decisions[0]["decision"],
                        "decisions": replacement_decisions,
                        "simulation_time_s": (
                            None if simulation_time is None
                            else float(simulation_time)),
                    })
                    new_events.append(event)
                    self.failures.pop(vehicle_id, None)
                    self.no_outgoing.pop(vehicle_id, None)
                    self.boundary_road_ends.pop(vehicle_id, None)
                    self.accepted_network_terminals.pop(vehicle_id, None)
                    self.unresolved_interior_route_tails.pop(vehicle_id, None)
                    continue

                self.no_outgoing[vehicle_id] = tail_edge
                self.boundary_road_ends.pop(vehicle_id, None)
                self.accepted_network_terminals.pop(vehicle_id, None)
                self.unresolved_interior_route_tails[vehicle_id] = tail_edge
            except Exception as exc:
                self.failures[vehicle_id] = str(exc)
        return new_events

    @staticmethod
    def _optional_call(domain, method_name, *args):
        method = getattr(domain, method_name, None)
        if method is None:
            return None, False
        return method(*args), True

    @staticmethod
    def _serializable_leader(value):
        if not value:
            return None
        try:
            return {
                "vehicle_id": str(value[0]),
                "gap_m": float(value[1]),
            }
        except (IndexError, TypeError, ValueError, OverflowError):
            return {"unparsed": repr(value)}

    @staticmethod
    def _lane_matches_edge(lane_id, edge_id):
        lane_id = str(lane_id)
        edge_id = str(edge_id)
        return lane_id == edge_id or lane_id.startswith(edge_id + "_")

    @staticmethod
    def _as_bool(value):
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes")
        return bool(value)

    def _parse_next_link(self, link):
        try:
            approached_lane = str(link[0])
            # SUMO's published signature includes an internal-lane string as
            # item 1, while several released TraCI readers return
            # (lane, priority, open, foe, internal, ...). Accept both forms.
            parser_layout = (
                len(link) >= 6 and isinstance(link[1], bool) and
                isinstance(link[4], str))
            if parser_layout:
                is_open = self._as_bool(link[2])
                has_foe = self._as_bool(link[3])
            else:
                is_open = self._as_bool(link[3])
                has_foe = self._as_bool(link[4])
            link_state = str(link[5])
        except (IndexError, TypeError, ValueError):
            return None
        return {
            "approached_lane": approached_lane,
            "is_open": is_open,
            "has_foe": has_foe,
            "state": link_state,
        }

    def _stop_guard(self, vehicle_id, planned_route_edges):
        """Return a blocker and JSON-safe evidence from SUMO's own state.

        SUMO may hold a vehicle before a clear immediate link when the next
        normal edge is too short to store it and the following link is still
        conflicted. Inspect that short-edge chain instead of misclassifying
        the hold as unexplained.
        """
        evidence = {}
        planned_route_edges = [str(value) for value in planned_route_edges]
        if not planned_route_edges:
            return "next_link_state_unavailable", evidence

        stop_state, available = self._optional_call(
            self.vehicle, "getStopState", vehicle_id)
        if not available:
            return "stop_state_unavailable", evidence
        try:
            stop_state = int(stop_state)
        except (TypeError, ValueError, OverflowError):
            return "stop_state_unavailable", evidence
        evidence["stop_state"] = stop_state
        if stop_state:
            return "scheduled_stop", evidence

        leader, available = self._optional_call(
            self.vehicle, "getLeader", vehicle_id,
            max(100.0, self.recovery_leader_clearance_m))
        if not available:
            return "leader_state_unavailable", evidence
        serialized_leader = self._serializable_leader(leader)
        evidence["leader"] = serialized_leader
        if (serialized_leader is not None and
                "gap_m" in serialized_leader and
                serialized_leader["gap_m"] <=
                self.recovery_leader_clearance_m):
            return "close_leader", evidence

        next_tls, available = self._optional_call(
            self.vehicle, "getNextTLS", vehicle_id)
        if not available:
            return "traffic_signal_state_unavailable", evidence
        tls_evidence = []
        for item in next_tls or ():
            try:
                tls_id = str(item[0])
                distance = float(item[2])
                state = str(item[3])
            except (IndexError, TypeError, ValueError, OverflowError):
                continue
            tls_evidence.append({
                "tls_id": tls_id,
                "distance_m": distance,
                "state": state,
            })
            if (distance <= self.recovery_tls_distance_m and
                    state[:1].lower() in ("r", "y")):
                evidence["next_tls"] = tls_evidence
                return "red_or_yellow_signal", evidence
        evidence["next_tls"] = tls_evidence

        next_links, available = self._optional_call(
            self.vehicle, "getNextLinks", vehicle_id)
        if not available:
            return "next_link_state_unavailable", evidence
        next_links = list(next_links or ())
        if not next_links:
            return "next_link_state_unavailable", evidence

        vehicle_length, length_available = self._optional_call(
            self.vehicle, "getLength", vehicle_id)
        minimum_gap, gap_available = self._optional_call(
            self.vehicle, "getMinGap", vehicle_id)
        required_storage = None
        if length_available and gap_available:
            try:
                required_storage = max(
                    0.0, float(vehicle_length) + float(minimum_gap))
            except (TypeError, ValueError, OverflowError):
                required_storage = None
        evidence["required_storage_m"] = required_storage
        checked_links = []
        for planned_index, planned_edge in enumerate(planned_route_edges):
            selected_link = None
            for link in next_links:
                try:
                    if self._lane_matches_edge(link[0], planned_edge):
                        selected_link = link
                        break
                except (IndexError, TypeError):
                    continue
            if selected_link is None and planned_index == 0:
                selected_link = next_links[0]
            parsed_link = (
                None if selected_link is None
                else self._parse_next_link(selected_link))
            if parsed_link is None:
                if planned_index == 0:
                    return "next_link_state_unavailable", evidence
                break
            parsed_link["planned_edge"] = planned_edge
            checked_links.append(parsed_link)
            evidence["next_links_checked"] = checked_links
            if planned_index == 0:
                evidence["next_link"] = parsed_link
            state_initial = parsed_link["state"][:1].lower()
            if state_initial in ("r", "y"):
                return (
                    "red_or_yellow_link" if planned_index == 0 else
                    "downstream_red_or_yellow_link"), evidence
            if state_initial in ("s", "w"):
                return (
                    "stop_controlled_link" if planned_index == 0 else
                    "downstream_stop_controlled_link"), evidence
            if not parsed_link["is_open"]:
                return (
                    "closed_next_link" if planned_index == 0 else
                    "downstream_short_edge_closed"), evidence
            if parsed_link["has_foe"]:
                return (
                    "conflicting_next_link" if planned_index == 0 else
                    "downstream_short_edge_conflict"), evidence

            # Once the approached edge can store this vehicle, conflicts at
            # later junctions cannot explain why it is held on the current
            # edge. Without length/minGap support, inspect only the immediate
            # link rather than guessing.
            if required_storage is None:
                break
            try:
                approached_length = float(
                    self.network.getEdge(planned_edge).getLength())
            except Exception:
                break
            parsed_link["approached_edge_length_m"] = approached_length
            if approached_length + 1.0e-9 >= required_storage:
                break
        return None, evidence

    def _reset_stall_state(self, state, now, position, reason):
        state["stopped"] = False
        state["anchor_position"] = position
        state["unexplained_since_s"] = None
        state["guarded_since_s"] = None
        state["guarded_reason"] = None
        state["last_observed_time_s"] = now
        state["last_guard_reason"] = reason

    def recover_persistent_stops(
            self, released_vehicle_ids, simulation_time=None,
            speed_authority_vehicle_ids=None,
            release_speed_authority=None):
        """Reroute only a persistent, autonomous, unexplained mid-route stop.

        For recorded-profile operation, candidates come from the profile
        lifecycle; other speed policies may pass any autonomous eligible ID.
        Merely releasing a profile never changes a route and starts a fresh
        autonomous persistence window. Recovery first keeps
        the immediate planned edge and changes only its generated suffix. An
        immediate alternative is a last resort and must have a viable path to
        a boundary exit or cycle; interior conversion stubs are rejected.
        """
        now = 0.0 if simulation_time is None else float(simulation_time)
        active_ids = set(str(value) for value in self.vehicle.getIDList())
        speed_authority_vehicle_ids = set(
            str(value) for value in (speed_authority_vehicle_ids or ()))
        candidates_to_process = (
            (set(str(value) for value in released_vehicle_ids) |
             speed_authority_vehicle_ids) &
            self.eligible_vehicle_ids & active_ids)
        new_events = []
        for vehicle_id in sorted(candidates_to_process):
            try:
                route = list(self.vehicle.getRoute(vehicle_id))
                route_index = int(self.vehicle.getRouteIndex(vehicle_id))
                if (not route or route_index < 0 or
                        route_index >= len(route)):
                    continue
                current_edge = str(route[route_index])
                get_road_id = getattr(self.vehicle, "getRoadID", None)
                if get_road_id is None:
                    continue
                actual_edge = str(get_road_id(vehicle_id))
                if (actual_edge.startswith(":") or
                        actual_edge != current_edge):
                    continue
                # Ordinary continuation owns route exhaustion. Recovery is
                # only for a failure to traverse an already planned link.
                if route_index >= len(route) - 1:
                    state = self._stall_states.setdefault(vehicle_id, {})
                    self._reset_stall_state(
                        state, now, None, "route_tail")
                    continue
                planned_next_edge = str(route[route_index + 1])

                speed = float(self.vehicle.getSpeed(vehicle_id))
                raw_position = self.vehicle.getPosition(vehicle_id)
                position = (float(raw_position[0]), float(raw_position[1]))
                if (not math.isfinite(speed) or
                        not all(math.isfinite(value) for value in position)):
                    raise ValueError("non-finite speed or position")
                state = self._stall_states.setdefault(vehicle_id, {
                    "stopped": False,
                    "anchor_position": position,
                    "unexplained_since_s": None,
                    "guarded_since_s": None,
                    "guarded_reason": None,
                    "last_observed_time_s": now,
                    "last_guard_reason": None,
                })
                anchor = state.get("anchor_position")
                progress = 0.0
                if anchor is not None:
                    progress = math.hypot(
                        position[0] - anchor[0], position[1] - anchor[1])
                if speed > self.recovery_resume_speed_mps:
                    self._reset_stall_state(
                        state, now, position, "moving")
                    continue
                if progress > self.recovery_progress_distance_m:
                    self._reset_stall_state(
                        state, now, position, "position_progress")
                    continue
                if not state.get("stopped", False):
                    if speed > self.recovery_stop_speed_mps:
                        state["last_observed_time_s"] = now
                        state["anchor_position"] = position
                        state["last_guard_reason"] = "slow_but_not_stopped"
                        continue
                    state["stopped"] = True
                    state["anchor_position"] = position

                blocker, evidence = self._stop_guard(
                    vehicle_id, route[route_index + 1:])
                evidence["speed_mps"] = speed
                evidence["position"] = {
                    "x": position[0], "y": position[1]}
                evidence["planned_next_edge"] = planned_next_edge
                state["last_observed_time_s"] = now
                state["last_guard_reason"] = blocker
                soft_downstream_blockers = {
                    "downstream_short_edge_closed",
                    "downstream_short_edge_conflict",
                }
                if blocker in soft_downstream_blockers:
                    state["unexplained_since_s"] = None
                    if state.get("guarded_reason") != blocker:
                        state["guarded_reason"] = blocker
                        state["guarded_since_s"] = now
                    guarded_since = state.get("guarded_since_s")
                    if guarded_since is None:
                        guarded_since = now
                        state["guarded_since_s"] = now
                    stopped_duration = max(0.0, now - guarded_since)
                    if (stopped_duration + 1.0e-9 <
                            self.recovery_downstream_blocked_wait_s):
                        continue
                    evidence["persistent_downstream_blocker"] = blocker
                elif blocker is not None:
                    state["unexplained_since_s"] = None
                    state["guarded_since_s"] = None
                    state["guarded_reason"] = None
                    continue
                else:
                    state["guarded_since_s"] = None
                    state["guarded_reason"] = None
                    if state.get("unexplained_since_s") is None:
                        state["unexplained_since_s"] = now
                    stopped_duration = max(
                        0.0, now - state["unexplained_since_s"])
                    if stopped_duration + 1.0e-9 < self.recovery_wait_s:
                        continue

                if vehicle_id in speed_authority_vehicle_ids:
                    if release_speed_authority is None:
                        state["last_guard_reason"] = (
                            "active_speed_authority_release_unavailable")
                        continue
                    release_reason = (
                        "persistent_downstream_short_edge_blocker"
                        if blocker in soft_downstream_blockers else
                        "persistent_unexplained_lane_stop")
                    released = bool(release_speed_authority(
                        vehicle_id, now, release_reason))
                    if released:
                        self._record_event(
                            self.speed_authority_release_events, {
                                "event_type": "recorded_speed_authority_release",
                                "vehicle_id": vehicle_id,
                                "current_edge": current_edge,
                                "planned_next_edge": planned_next_edge,
                                "reason": release_reason,
                                "simulation_time_s": now,
                                "stopped_duration_s": stopped_duration,
                                "evidence": evidence,
                            })
                    self._reset_stall_state(
                        state, now, position,
                        "recorded_speed_authority_released")
                    continue

                recovery_key = (vehicle_id, current_edge)
                if recovery_key in self._unresolved_edges:
                    continue
                vehicle_class = self.vehicle.getVehicleClass(vehicle_id)
                refresh_key = (
                    vehicle_id, current_edge, planned_next_edge)
                suffix_attempt = self._suffix_attempt_counts.get(
                    refresh_key, 0)
                if suffix_attempt < self.maximum_suffix_recovery_attempts:
                    old_remaining_route = [
                        str(edge_id) for edge_id in route[route_index:]
                    ]
                    existing_suffix = old_remaining_route[1:]
                    tried_branches = self._tried_suffix_branches.setdefault(
                        refresh_key, set())
                    self._remember_failed_suffix_branches(
                        existing_suffix, vehicle_class, tried_branches)
                    (refreshed_path, decisions, selected_branch, divergence,
                     next_decision) = self._seeded_divergent_suffix(
                         vehicle_id, existing_suffix, vehicle_class,
                         tried_branches)
                    new_remaining_route = (
                        [current_edge] + refreshed_path
                        if refreshed_path is not None else None)
                    if (new_remaining_route is not None and
                            new_remaining_route != old_remaining_route):
                        self.vehicle.setRoute(
                            vehicle_id, new_remaining_route)
                        # Only a route accepted by TraCI consumes an applied
                        # attempt or bans its selected branch.
                        tried_branches.add(selected_branch)
                        self._suffix_attempt_counts[refresh_key] = (
                            suffix_attempt + 1)
                        self.decision_counts[vehicle_id] = next_decision
                        event = self._record_event(
                            self.stall_recovery_events, {
                                "mutation_type": (
                                    "persistent_stop_suffix_recovery"),
                                "vehicle_id": vehicle_id,
                                "from_edge": current_edge,
                                "original_next_edge": planned_next_edge,
                                "preserved_next_edge": planned_next_edge,
                                "original_route_remainder": (
                                    old_remaining_route),
                                "new_route_remainder": new_remaining_route,
                                "to_edge": new_remaining_route[-1],
                                "suffix_recovery_attempt": (
                                    suffix_attempt + 1),
                                "divergence_from_edge": selected_branch[0],
                                "divergence_to_edge": selected_branch[1],
                                "preserved_suffix_prefix": (
                                    refreshed_path[:divergence + 1]),
                                "tried_suffix_branches": [
                                    {"from_edge": from_edge,
                                     "to_edge": to_edge}
                                    for from_edge, to_edge in sorted(
                                        tried_branches)
                                ],
                                "decisions": decisions,
                                "simulation_time_s": now,
                                "stopped_duration_s": stopped_duration,
                                "evidence": evidence,
                            })
                        new_events.append(event)
                        self._reset_stall_state(
                            state, now, position, "suffix_recovered")
                        self.stall_recovery_failures.pop(vehicle_id, None)
                        continue

                outgoing = self._outgoing_edge_ids(
                    current_edge, vehicle_class)
                tried_immediate_branches = (
                    self._tried_immediate_branches.setdefault(
                        recovery_key, set()))
                # Reaching this point means the currently planned immediate
                # link has already failed for a complete guarded persistence
                # window. Remember it before selecting another branch so a
                # later retry cannot oscillate back to the same blocked turn.
                tried_immediate_branches.add(
                    (current_edge, planned_next_edge))
                raw_alternatives = [
                    edge_id for edge_id in outgoing
                    if ((current_edge, edge_id) not in
                        tried_immediate_branches)
                ]
                (replacement_path, replacement_decisions, rejected,
                 replacement_next_decision) = (
                    self._propose_continuation_path(
                        vehicle_id, current_edge, raw_alternatives,
                        vehicle_class,
                        forbidden_branches=tried_immediate_branches,
                        prefer_longest=True))
                if replacement_path is None:
                    detail = {
                        "vehicle_id": vehicle_id,
                        "from_edge": current_edge,
                        "planned_next_edge": planned_next_edge,
                        "simulation_time_s": now,
                        "stopped_duration_s": stopped_duration,
                        "reason": "no_viable_alternative_outgoing",
                        "suffix_recovery_attempts": suffix_attempt,
                        "tried_suffix_branches": [
                            {"from_edge": from_edge,
                             "to_edge": to_edge}
                            for from_edge, to_edge in sorted(
                                self._tried_suffix_branches.get(
                                    refresh_key, ()))
                        ],
                        "tried_immediate_branches": [
                            {"from_edge": from_edge,
                             "to_edge": to_edge}
                            for from_edge, to_edge in sorted(
                                tried_immediate_branches)
                        ],
                        "rejected_interior_sink_edges": rejected,
                        "evidence": evidence,
                    }
                    self.stall_recovery_unresolved[vehicle_id] = detail
                    self._unresolved_edges.add(recovery_key)
                    continue
                new_remaining_route = [current_edge] + replacement_path
                self.vehicle.setRoute(
                    vehicle_id, new_remaining_route)
                self.decision_counts[vehicle_id] = (
                    replacement_next_decision)
                event = self._record_event(self.stall_recovery_events, {
                    "mutation_type": (
                        "persistent_stop_immediate_edge_recovery"),
                    "vehicle_id": vehicle_id,
                    "from_edge": current_edge,
                    "original_next_edge": planned_next_edge,
                    "to_edge": replacement_path[-1],
                    "selected_next_edge": replacement_path[0],
                    "new_route_remainder": new_remaining_route,
                    "appended_edges": replacement_path,
                    "decision": replacement_decisions[0]["decision"],
                    "decisions": replacement_decisions,
                    "tried_immediate_branches": [
                        {"from_edge": from_edge,
                         "to_edge": to_edge}
                        for from_edge, to_edge in sorted(
                            tried_immediate_branches)
                    ],
                    "simulation_time_s": now,
                    "stopped_duration_s": stopped_duration,
                    "evidence": evidence,
                })
                new_events.append(event)
                self._reset_stall_state(
                    state, now, position, "recovered")
                self.stall_recovery_failures.pop(vehicle_id, None)
                self.stall_recovery_unresolved.pop(vehicle_id, None)
            except Exception as exc:
                self.stall_recovery_failures[vehicle_id] = str(exc)
        return new_events

    def reroute_terminal_releases(self, vehicle_ids, simulation_time=None):
        """Compatibility alias for guarded persistent-stop recovery."""
        return self.recover_persistent_stops(
            vehicle_ids, simulation_time=simulation_time)

    def metadata(self):
        return {
            "enabled": True,
            "policy": (
                "preserve_prepared_route_then_guarded_persistent_stop_"
                "recovery"),
            "eligible_vehicle_count": len(self.eligible_vehicle_ids),
            "accepted_network_terminal_vehicle_ids": sorted(
                self.accepted_network_terminal_vehicle_ids),
            "allow_uturns": self.allow_uturns,
            "seed": self.seed,
            "tail_extension_lookahead": "penultimate_normal_edge",
            "extension_count": len(self.events),
            "extended_vehicle_ids": sorted(set(
                event["vehicle_id"] for event in self.events)),
            "events": list(self.events),
            "terminal_release_immediate_reroute_count": 0,
            "persistent_stop_recovery_settings": {
                "minimum_duration_s": self.recovery_wait_s,
                "stop_speed_mps": self.recovery_stop_speed_mps,
                "resume_speed_mps": self.recovery_resume_speed_mps,
                "progress_reset_distance_m": (
                    self.recovery_progress_distance_m),
                "close_leader_distance_m": (
                    self.recovery_leader_clearance_m),
                "traffic_signal_guard_distance_m": (
                    self.recovery_tls_distance_m),
                "downstream_blocker_minimum_duration_s": (
                    self.recovery_downstream_blocked_wait_s),
                "maximum_suffix_recovery_attempts": (
                    self.maximum_suffix_recovery_attempts),
            },
            "boundary_terminal_tolerance_m": (
                self.boundary_terminal_tolerance_m),
            "continuation_search_depth_edges": (
                self.continuation_search_depth_edges),
            "persistent_stop_recovery_count": len(
                self.stall_recovery_events),
            "persistent_stop_recoveries": list(
                self.stall_recovery_events),
            "persistent_stop_recovery_unresolved": dict(sorted(
                self.stall_recovery_unresolved.items())),
            "persistent_stop_recovery_failures": dict(sorted(
                self.stall_recovery_failures.items())),
            "recorded_speed_authority_release_count": len(
                self.speed_authority_release_events),
            "recorded_speed_authority_releases": list(
                self.speed_authority_release_events),
            "road_end_without_outgoing": dict(sorted(
                self.no_outgoing.items())),
            "boundary_road_ends": dict(sorted(
                self.boundary_road_ends.items())),
            "accepted_network_terminals": dict(sorted(
                self.accepted_network_terminals.items())),
            "unresolved_interior_route_tails": dict(sorted(
                self.unresolved_interior_route_tails.items())),
            "interior_sink_candidates_rejected": dict(sorted(
                self.nonviable_outgoing.items())),
            "failures": dict(sorted(self.failures.items())),
        }
