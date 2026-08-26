"""Create data-seeded SUMO routes from a netconvert-generated network."""

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
import re
import xml.etree.ElementTree as ET

from .tracks import (
    DEFAULT_MINIMUM_TRACK_DISTANCE_M, DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS,
    actor_kind, classify_vehicle_motion, validated_minimum_track_distance,
    validated_minimum_two_point_speed)


MAXIMUM_ROUTE_HEADING_ERROR_RAD = math.pi / 2.0
MAXIMUM_SAME_EDGE_REGRESSION_M = 2.0
DEFAULT_SUMO_VTYPE_MAX_SPEED_MPS = 55.55
MAXIMUM_CYCLE_CONSTRUCTION_EDGES = 100000


@dataclass
class LaneGeometry:
    edge_id: str
    lane_id: str
    lane_index: int
    shape: list
    length: float
    allow: set
    disallow: set


@dataclass
class NetworkGeometry:
    offset: tuple
    lanes: list
    internal_lanes: list
    adjacency: dict
    connections: dict
    lane_by_edge_index: dict
    internal_lane_by_id: dict
    via_connections: dict
    boundary: tuple


@dataclass(frozen=True)
class EdgeConnection:
    from_edge: str
    to_edge: str
    from_lane: int
    to_lane: int
    via_lane_id: str
    direction: str
    via_lane_ids: tuple = ()


def _parse_shape(value):
    points = []
    for item in (value or "").split():
        x, y = item.split(",")[:2]
        points.append((float(x), float(y)))
    return points


def load_network(path):
    root = ET.parse(path).getroot()
    location = root.find("location")
    offset = (0.0, 0.0)
    boundary = None
    if location is not None and location.get("netOffset"):
        offset = tuple(float(value) for value in location.get("netOffset").split(",")[:2])
    if location is not None and location.get("convBoundary"):
        values = tuple(
            float(value) for value in
            location.get("convBoundary").split(","))
        if len(values) >= 4:
            boundary = values[:4]

    lanes = []
    internal_lanes = []
    lane_by_edge_index = {}
    internal_lane_by_id = {}
    normal_edges = set()
    for edge in root.findall("edge"):
        edge_id = edge.get("id")
        if not edge_id:
            continue
        internal = edge_id.startswith(":") or edge.get("function") == "internal"
        if not internal:
            normal_edges.add(edge_id)
        for lane in edge.findall("lane"):
            shape = _parse_shape(lane.get("shape"))
            if len(shape) < 2:
                continue
            geometry = LaneGeometry(
                edge_id=edge_id,
                lane_id=lane.get("id", "%s_0" % edge_id),
                lane_index=int(lane.get("index", 0)),
                shape=shape,
                length=float(lane.get("length", 0.0)),
                allow=set((lane.get("allow") or "").split()),
                disallow=set((lane.get("disallow") or "").split()))
            if internal:
                internal_lanes.append(geometry)
                internal_lane_by_id[geometry.lane_id] = geometry
            else:
                lanes.append(geometry)
                lane_by_edge_index[(edge_id, geometry.lane_index)] = geometry

    raw_connections = list(root.findall("connection"))
    internal_successors = {}
    for connection in raw_connections:
        left = connection.get("from")
        if not left or not left.startswith(":"):
            continue
        key = (
            left,
            int(connection.get("fromLane", 0)),
            connection.get("to"))
        internal_successors.setdefault(key, []).append(connection)

    def resolve_via_lanes(first_lane_id, target_edge):
        """Follow SUMO's chained internal connections to a normal edge."""
        if not first_lane_id:
            return ()
        lane_ids = []
        seen = set()
        current_lane_id = first_lane_id
        while current_lane_id:
            if current_lane_id in seen:
                return None
            seen.add(current_lane_id)
            lane = internal_lane_by_id.get(current_lane_id)
            if lane is None:
                return None
            lane_ids.append(current_lane_id)
            successors = internal_successors.get(
                (lane.edge_id, lane.lane_index, target_edge), [])
            if not successors:
                break
            if len(successors) != 1:
                return None
            current_lane_id = successors[0].get("via", "")
        return tuple(lane_ids)

    adjacency = {edge_id: set() for edge_id in normal_edges}
    connections = {edge_id: [] for edge_id in normal_edges}
    via_connections = {}
    ambiguous_via_lanes = set()
    for connection in raw_connections:
        left, right = connection.get("from"), connection.get("to")
        if left in normal_edges and right in normal_edges:
            adjacency[left].add(right)
            via_lane_id = connection.get("via", "")
            via_lane_ids = resolve_via_lanes(via_lane_id, right)
            item = EdgeConnection(
                from_edge=left,
                to_edge=right,
                from_lane=int(connection.get("fromLane", 0)),
                to_lane=int(connection.get("toLane", 0)),
                via_lane_id=via_lane_id,
                direction=connection.get("dir", ""),
                via_lane_ids=(
                    via_lane_ids if via_lane_ids is not None else ()))
            connections[left].append(item)
            if via_lane_ids is not None:
                for lane_id in via_lane_ids:
                    existing = via_connections.get(lane_id)
                    if existing is not None and existing != item:
                        ambiguous_via_lanes.add(lane_id)
                    else:
                        via_connections[lane_id] = item
    for lane_id in ambiguous_via_lanes:
        via_connections.pop(lane_id, None)
    return NetworkGeometry(
        offset, lanes, internal_lanes, adjacency, connections,
        lane_by_edge_index, internal_lane_by_id, via_connections, boundary)


def _vehicle_class(kind):
    return {
        "truck": "truck",
        "bus": "bus",
        "motorcycle": "motorcycle",
        "bicycle": "bicycle",
    }.get(kind, "passenger")


def _allows(lane, vehicle_class):
    if lane.allow and vehicle_class not in lane.allow:
        return False
    return vehicle_class not in lane.disallow


def _angle_difference(left, right):
    return abs((left - right + math.pi) % (2.0 * math.pi) - math.pi)


def _project_segment(point, left, right):
    dx, dy = right[0] - left[0], right[1] - left[1]
    denom = dx * dx + dy * dy
    ratio = 0.0 if denom <= 1.0e-12 else (
        ((point[0] - left[0]) * dx + (point[1] - left[1]) * dy) / denom)
    ratio = max(0.0, min(1.0, ratio))
    projected = (left[0] + ratio * dx, left[1] + ratio * dy)
    distance = math.hypot(point[0] - projected[0], point[1] - projected[1])
    return distance, ratio, math.atan2(dy, dx)


def project_to_lane(point, yaw, lane):
    best = None
    travelled = 0.0
    for left, right in zip(lane.shape, lane.shape[1:]):
        segment_length = math.hypot(right[0] - left[0], right[1] - left[1])
        distance, ratio, heading = _project_segment(point, left, right)
        result = {
            "distance": distance,
            "heading_error": _angle_difference(yaw, heading),
            "position": travelled + ratio * segment_length,
        }
        if best is None or result["distance"] < best["distance"]:
            best = result
        travelled += segment_length
    return best


def _nearest_lane_from(lanes, point, yaw, vehicle_class, heading_weight):
    best = None
    for lane in lanes:
        if not _allows(lane, vehicle_class):
            continue
        projection = project_to_lane(point, yaw, lane)
        if projection is None:
            continue
        score = projection["distance"] + heading_weight * projection["heading_error"]
        candidate = (score, lane, projection)
        if best is None or score < best[0]:
            best = candidate
    return best


def nearest_lane(network, point, yaw, kind="car", heading_weight=3.0):
    sumo_point = (point[0] + network.offset[0], point[1] + network.offset[1])
    return _nearest_lane_from(
        network.lanes, sumo_point, yaw, _vehicle_class(kind), heading_weight)


def _connection_allows(network, connection, vehicle_class):
    source = network.lane_by_edge_index.get(
        (connection.from_edge, connection.from_lane))
    target = network.lane_by_edge_index.get(
        (connection.to_edge, connection.to_lane))
    if not (source is not None and target is not None and
            _allows(source, vehicle_class) and
            _allows(target, vehicle_class)):
        return False
    if connection.via_lane_id and not connection.via_lane_ids:
        return False
    for lane_id in connection.via_lane_ids:
        lane = network.internal_lane_by_id.get(lane_id)
        if lane is None or not _allows(lane, vehicle_class):
            return False
    return True


def _point_route_candidates(
        network, point, kind="car", heading_weight=3.0,
        maximum_snap_distance=8.0,
        maximum_heading_error=MAXIMUM_ROUTE_HEADING_ERROR_RAD):
    """Return plausible normal-edge and internal-connector observations."""
    sumo_point = (
        point.x + network.offset[0], point.y + network.offset[1])
    vehicle_class = _vehicle_class(kind)
    candidates = {}
    for lane in network.lanes:
        if not _allows(lane, vehicle_class):
            continue
        projection = project_to_lane(sumo_point, point.yaw, lane)
        if (projection is None or
                projection["distance"] > maximum_snap_distance or
                projection["heading_error"] > maximum_heading_error):
            continue
        score = (
            projection["distance"] +
            heading_weight * projection["heading_error"])
        state = ("normal", lane.edge_id)
        item = {
            "state": state,
            "score": score,
            "lane": lane,
            "projection": projection,
            "snap_distance": projection["distance"],
            "heading_error": projection["heading_error"],
            "edges": [lane.edge_id],
            "kind": "normal_lane",
            "matched_lane": lane,
            "matched_projection": projection,
        }
        if state not in candidates or score < candidates[state]["score"]:
            candidates[state] = item

    for lane in network.internal_lanes:
        connection = network.via_connections.get(lane.lane_id)
        if (connection is None or not _allows(lane, vehicle_class) or
                not _connection_allows(network, connection, vehicle_class)):
            continue
        projection = project_to_lane(sumo_point, point.yaw, lane)
        if (projection is None or
                projection["distance"] > maximum_snap_distance or
                projection["heading_error"] > maximum_heading_error):
            continue
        score = (
            projection["distance"] +
            heading_weight * projection["heading_error"])
        source_lane = network.lane_by_edge_index[
            (connection.from_edge, connection.from_lane)]
        depart_projection = dict(projection)
        depart_projection["position"] = max(0.0, source_lane.length - 0.05)
        state = (
            "connection", connection.from_edge, connection.to_edge)
        item = {
            "state": state,
            "score": score,
            "lane": source_lane,
            "projection": depart_projection,
            "snap_distance": projection["distance"],
            "heading_error": projection["heading_error"],
            "edges": [connection.from_edge, connection.to_edge],
            "kind": "internal_connection",
            "internal_lane_id": lane.lane_id,
            "connection_direction": connection.direction,
            "matched_lane": lane,
            "matched_projection": projection,
        }
        if state not in candidates or score < candidates[state]["score"]:
            candidates[state] = item
    return sorted(
        candidates.values(),
        key=lambda item: (item["score"], item["state"]))


def _vehicle_adjacency(network, kind, allow_uturns=False):
    vehicle_class = _vehicle_class(kind)
    return {
        edge_id: {
            item.to_edge for item in _allowed_outgoing_connections(
                network, edge_id, vehicle_class, allow_uturns)}
        for edge_id in network.adjacency
    }


def _candidate_transition_cost(adjacency, previous, current):
    if previous["state"] == current["state"]:
        if (current["state"][0] == "normal" and
                current["projection"]["position"] +
                MAXIMUM_SAME_EDGE_REGRESSION_M <
                previous["projection"]["position"]):
            return None
        return 0.0
    start = previous["edges"][-1]
    target = current["edges"][0]
    bridge = shortest_edge_path(adjacency, start, target)
    if bridge is None:
        return None
    # Moving to the next observed edge/connector is cheap; skipping normal
    # edges is permitted but explicitly penalized and reported later.
    return 0.15 + 0.5 * max(0, len(bridge) - 1)


def _match_track_points(network, track, maximum_snap_distance):
    """Continuity-constrained dynamic-programming match over track samples."""
    kind = actor_kind(track.category)
    # Recorded evidence takes precedence over the continuation policy.  If a
    # real track contains a legal U-turn, retain it; generated tails still
    # avoid U-turns unless explicitly enabled.
    adjacency = _vehicle_adjacency(network, kind, allow_uturns=True)
    layers = []
    for index, point in enumerate(track.points):
        candidates = _point_route_candidates(
            network, point, kind=kind,
            maximum_snap_distance=maximum_snap_distance)
        if not candidates:
            continue
        if not layers:
            costs = [item["score"] for item in candidates]
            layers.append((index, point, candidates, costs, [None] * len(candidates)))
            continue
        previous_candidates = layers[-1][2]
        previous_costs = layers[-1][3]
        costs = []
        parents = []
        for current in candidates:
            best_cost = None
            best_parent = None
            for parent_index, previous in enumerate(previous_candidates):
                transition = _candidate_transition_cost(
                    adjacency, previous, current)
                if transition is None:
                    continue
                cost = previous_costs[parent_index] + transition + current["score"]
                if best_cost is None or cost < best_cost:
                    best_cost = cost
                    best_parent = parent_index
            costs.append(best_cost)
            parents.append(best_parent)
        reachable = [
            item for item, cost, parent in zip(candidates, costs, parents)
            if cost is not None and parent is not None]
        if not reachable:
            # Treat a wholly inconsistent observation as unmatched instead of
            # forcing a backward jump to an already-consumed connector.
            continue
        filtered_costs = [
            cost for cost in costs if cost is not None]
        filtered_parents = [
            parent for cost, parent in zip(costs, parents)
            if cost is not None]
        layers.append((
            index, point, reachable, filtered_costs, filtered_parents))

    if not layers:
        return []
    selected = [None] * len(layers)
    selected[-1] = min(
        range(len(layers[-1][3])), key=lambda item: layers[-1][3][item])
    for layer_index in range(len(layers) - 1, 0, -1):
        selected[layer_index - 1] = layers[layer_index][4][
            selected[layer_index]]
    return [
        (layer[0], layer[1], layer[2][choice])
        for layer, choice in zip(layers, selected)]


def shortest_edge_path(adjacency, start, target):
    if start == target:
        return [start]
    queue = deque([(start, [start])])
    visited = {start}
    while queue:
        edge, path = queue.popleft()
        for successor in sorted(adjacency.get(edge, ())):
            if successor == target:
                return path + [successor]
            if successor not in visited:
                visited.add(successor)
                queue.append((successor, path + [successor]))
    return None


def connect_edge_sequence(adjacency, edges):
    route, _ = connect_edge_sequence_with_inference(adjacency, edges)
    return route


def connect_edge_sequence_with_inference(adjacency, edges):
    collapsed = []
    for edge in edges:
        if not collapsed or edge != collapsed[-1]:
            collapsed.append(edge)
    if not collapsed:
        return None, []
    route = [collapsed[0]]
    inferred = []
    for target in collapsed[1:]:
        bridge = shortest_edge_path(adjacency, route[-1], target)
        if bridge is None:
            return None, []
        inferred.extend(bridge[1:-1])
        route.extend(bridge[1:])
    return route, inferred


def plan_track_route(network, track, maximum_snap_distance=8.0):
    matches = _match_track_points(
        network, track, maximum_snap_distance)
    if not matches:
        return None, (
            "no direction- and vehicle-class-compatible point was close "
            "enough to a SUMO lane")
    if len(matches) < 2:
        return None, "fewer than 2 direction-consistent points matched SUMO lanes"
    state_matches = []
    for item in matches:
        if (not state_matches or
                state_matches[-1][2]["state"] != item[2]["state"]):
            state_matches.append(item)
    observed_edges = []
    for _, _, match in state_matches:
        evidence = match["edges"]
        if not observed_edges:
            observed_edges.extend(evidence)
        elif observed_edges[-1] == evidence[0]:
            observed_edges.extend(evidence[1:])
        elif observed_edges[-1] != evidence[-1]:
            observed_edges.extend(evidence)
    route, inferred = connect_edge_sequence_with_inference(
        _vehicle_adjacency(
            network, actor_kind(track.category), allow_uturns=True),
        observed_edges)
    if route is None:
        return None, "matched edges are not connected in the SUMO network"
    first_index, first_point, first_match = matches[0]
    last_index, last_point, _ = matches[-1]
    first_lane = first_match["lane"]
    first_projection = first_match["projection"]
    first_matched_lane = first_match["matched_lane"]
    first_matched_projection = first_match["matched_projection"]
    distances = sorted(match["snap_distance"] for _, _, match in matches)
    heading_errors = sorted(
        match["heading_error"] for _, _, match in matches)
    percentile_index = max(0, int(math.ceil(0.95 * len(distances))) - 1)
    internal_transitions = []
    seen_internal = set()
    for _, point, match in matches:
        if match["kind"] != "internal_connection":
            continue
        transition = tuple(match["edges"])
        if transition in seen_internal:
            continue
        seen_internal.add(transition)
        internal_transitions.append({
            "from": transition[0],
            "to": transition[1],
            "direction": match.get("connection_direction", ""),
            "internal_lane": match.get("internal_lane_id", ""),
            "first_evidence_frame": point.frame,
            "first_evidence_time": point.time,
        })
    return {
        "edges": route,
        "recorded_edges": list(route),
        "observed_edge_sequence": list(observed_edges),
        "inferred_connector_edges": inferred,
        "internal_transitions": internal_transitions,
        "depart_lane": first_lane.lane_index,
        "depart_pos": max(0.0, min(first_projection["position"], first_lane.length)),
        "first_match_kind": first_match["kind"],
        "first_matched_lane_id": first_matched_lane.lane_id,
        "first_matched_lane_position_m": first_matched_projection["position"],
        "initial_sumo_pose": {
            "x": first_point.x + network.offset[0],
            "y": first_point.y + network.offset[1],
            "angle_degrees": (
                90.0 - math.degrees(first_point.yaw)) % 360.0,
            # nuScenes annotations and ActorTrack points describe the vehicle
            # center.  TraCI moveToXY describes SUMO's front-center bumper, so
            # the runtime converts this pose using the generated vehicle length.
            "reference_point": "vehicle_center",
        },
        "first_matched_frame": first_point.frame,
        "first_matched_time": first_point.time,
        "last_matched_frame": last_point.frame,
        "last_matched_time": last_point.time,
        "matched_sample_count": len(matches),
        "total_sample_count": len(track.points),
        "matched_sample_fraction": len(matches) / float(len(track.points)),
        "unmatched_prefix_count": first_index,
        "unmatched_suffix_count": len(track.points) - last_index - 1,
        "mean_snap_distance": sum(distances) / len(distances),
        "p95_snap_distance": distances[percentile_index],
        "maximum_snap_distance": distances[-1],
        "mean_heading_error_degrees": math.degrees(
            sum(heading_errors) / len(heading_errors)),
        "p95_heading_error_degrees": math.degrees(
            heading_errors[percentile_index]),
        "maximum_heading_error_degrees": math.degrees(
            heading_errors[-1]),
    }, None


def _allowed_outgoing_connections(network, edge_id, vehicle_class,
                                  allow_uturns=False):
    by_target = {}
    for connection in network.connections.get(edge_id, []):
        if connection.direction == "t" and not allow_uturns:
            continue
        if not _connection_allows(network, connection, vehicle_class):
            continue
        current = by_target.get(connection.to_edge)
        key = (
            connection.from_lane, connection.to_lane,
            connection.direction, connection.via_lane_ids)
        if current is None or key < (
                current.from_lane, current.to_lane,
                current.direction, current.via_lane_ids):
            by_target[connection.to_edge] = connection
    return [by_target[target] for target in sorted(by_target)]


def _edge_boundary_distance(network, edge_id):
    if network.boundary is None:
        return math.inf
    lanes = [lane for lane in network.lanes if lane.edge_id == edge_id]
    if not lanes:
        return math.inf
    x, y = lanes[0].shape[-1]
    left, bottom, right, top = network.boundary
    return min(abs(x - left), abs(right - x),
               abs(y - bottom), abs(top - y))


def _reported_boundary_distance(network, edge_id):
    distance = _edge_boundary_distance(network, edge_id)
    return distance if math.isfinite(distance) else None


def _stable_connection_order(connections, seed, actor_id, decision):
    def key(connection):
        payload = "%s|%s|%s|%s|%s|%s" % (
            seed, actor_id, decision, connection.from_edge,
            connection.to_edge, connection.direction)
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()
    return sorted(connections, key=key)


def _connection_route_length(network, connection):
    """Return conservative progress along a connection's target normal lane."""
    lane = network.lane_by_edge_index.get(
        (connection.to_edge, connection.to_lane))
    if lane is None:
        return 0.0
    if lane.length > 0.0:
        return lane.length
    return sum(
        math.hypot(right[0] - left[0], right[1] - left[1])
        for left, right in zip(lane.shape, lane.shape[1:]))


def extend_route_to_terminal(
        network, recorded_edges, kind, actor_id, seed=0,
        maximum_edges=64, allow_uturns=False,
        boundary_tolerance=15.0, minimum_cycle_distance_m=0.0):
    """Append a deterministic path to a boundary exit or reusable cycle.

    A recorded prefix that already reaches a source-defined graph sink has
    reached the end of the available road and is therefore a valid network
    terminal. Generated tails never select interior graph sinks. When no
    reachable boundary exists, the finite route closes and repeats a
    class-valid cycle long enough to cover the configured simulation horizon.
    """
    if not recorded_edges:
        raise ValueError("recorded_edges must not be empty")
    if isinstance(maximum_edges, bool):
        raise ValueError("route continuation search depth must be positive")
    try:
        converted_maximum_edges = int(maximum_edges)
        exact_maximum_edges = float(maximum_edges)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("route continuation search depth must be positive")
    if (not math.isfinite(exact_maximum_edges) or
            exact_maximum_edges != converted_maximum_edges):
        raise ValueError(
            "route continuation search depth must be a positive integer")
    maximum_edges = converted_maximum_edges
    if maximum_edges <= 0:
        raise ValueError("route continuation search depth must be positive")
    try:
        boundary_tolerance = float(boundary_tolerance)
    except (TypeError, ValueError):
        raise ValueError("boundary_terminal_tolerance must be non-negative")
    if (not math.isfinite(boundary_tolerance) or
            boundary_tolerance < 0.0):
        raise ValueError("boundary_terminal_tolerance must be non-negative")
    try:
        minimum_cycle_distance_m = float(minimum_cycle_distance_m)
    except (TypeError, ValueError):
        raise ValueError("minimum_cycle_distance_m must be non-negative")
    if (not math.isfinite(minimum_cycle_distance_m) or
            minimum_cycle_distance_m < 0.0):
        raise ValueError("minimum_cycle_distance_m must be non-negative")

    vehicle_class = _vehicle_class(kind)

    def outgoing(edge_id):
        return _allowed_outgoing_connections(
            network, edge_id, vehicle_class, allow_uturns)

    def terminal_kind(edge_id):
        return (
            "boundary_terminal"
            if _edge_boundary_distance(network, edge_id) <= boundary_tolerance
            else "interior_terminal")

    def search(edge_id, visited, remaining, boundary_only, decision):
        candidates = outgoing(edge_id)
        if not candidates:
            if not boundary_only or terminal_kind(edge_id) == "boundary_terminal":
                return []
            return None
        if remaining <= 0:
            return None
        candidates = [
            item for item in candidates if item.to_edge not in visited]
        if not candidates:
            return None
        for connection in _stable_connection_order(
                candidates, seed, actor_id, decision):
            tail = search(
                connection.to_edge,
                visited | {connection.to_edge},
                remaining - 1, boundary_only, decision + 1)
            if tail is not None:
                return [connection] + tail
        return None

    final_edge = recorded_edges[-1]
    if not outgoing(final_edge):
        final_terminal_kind = terminal_kind(final_edge)
        if (final_terminal_kind != "boundary_terminal" and
                network.connections.get(final_edge)):
            # A raw connection exists, but vehicle-class or U-turn filtering
            # removed every legal successor. This is not a real map road end.
            raise RuntimeError(
                "actor %s has no valid boundary-or-cycle continuation from "
                "interior terminal edge %s" % (actor_id, final_edge))
        return [], {
            "status": (
                final_terminal_kind
                if final_terminal_kind == "boundary_terminal"
                else "network_terminal"),
            "terminal_edge": final_edge,
            "terminal_boundary_distance_m": _reported_boundary_distance(
                network, final_edge),
            "directions": [],
        }

    visited = set(recorded_edges)
    path = search(
        final_edge, visited, maximum_edges, True, 0)
    if path is None:
        # A ring road or closed grid may not contain a reachable sink.  Keep
        # the actor alive for the configured horizon by choosing the
        # least-visited deterministic successor and allowing revisits. Build
        # the reachable graph first and remove every branch that can only end
        # at an interior conversion stub. Each retained edge can either reach
        # a real boundary terminal or participate in/reach a directed cycle.
        reachable_connections = {}
        reverse_edges = {}
        pending_edges = deque([final_edge])
        discovered_edges = {final_edge}
        while pending_edges:
            reachable_edge = pending_edges.popleft()
            candidates = outgoing(reachable_edge)
            reachable_connections[reachable_edge] = candidates
            for candidate in candidates:
                reverse_edges.setdefault(candidate.to_edge, set()).add(
                    reachable_edge)
                if candidate.to_edge not in discovered_edges:
                    discovered_edges.add(candidate.to_edge)
                    pending_edges.append(candidate.to_edge)

        boundary_terminals = {
            edge_id for edge_id, candidates in
            reachable_connections.items()
            if (not candidates and
                terminal_kind(edge_id) == "boundary_terminal")
        }
        boundary_reachable = set(boundary_terminals)
        pending_edges = deque(sorted(boundary_terminals))
        while pending_edges:
            target_edge = pending_edges.popleft()
            for predecessor in sorted(reverse_edges.get(target_edge, ())):
                if predecessor in boundary_reachable:
                    continue
                boundary_reachable.add(predecessor)
                pending_edges.append(predecessor)

        # Iteratively remove graph sinks. What survives is exactly the set of
        # nodes from which an infinite walk (and therefore a directed cycle in
        # this finite graph) is reachable.
        cycle_reachable = set(reachable_connections)
        remaining_successors = {
            edge_id: len({
                candidate.to_edge for candidate in candidates
                if candidate.to_edge in cycle_reachable
            })
            for edge_id, candidates in reachable_connections.items()
        }
        pending_edges = deque(sorted(
            edge_id for edge_id, count in remaining_successors.items()
            if count == 0))
        while pending_edges:
            dead_edge = pending_edges.popleft()
            if dead_edge not in cycle_reachable:
                continue
            cycle_reachable.remove(dead_edge)
            for predecessor in reverse_edges.get(dead_edge, ()):
                if predecessor not in cycle_reachable:
                    continue
                remaining_successors[predecessor] -= 1
                if remaining_successors[predecessor] == 0:
                    pending_edges.append(predecessor)
        viable_edges = boundary_reachable | cycle_reachable
        if final_edge not in viable_edges:
            raise RuntimeError(
                "actor %s has no valid boundary-or-cycle continuation from "
                "edge %s" % (actor_id, final_edge))

        # ``maximum_edges`` is the legacy internal name for the terminal-search
        # depth and minimum cyclic tail.  A
        # distance-derived finite construction limit lets long simulations
        # exceed that bound without permitting an infinite builder loop.
        visit_counts = {}
        for edge_id in recorded_edges:
            visit_counts[edge_id] = visit_counts.get(edge_id, 0) + 1
        positive_lengths = []
        for edge_id in network.connections:
            for item in outgoing(edge_id):
                length = _connection_route_length(network, item)
                if length > 0.0:
                    positive_lengths.append(length)
        if minimum_cycle_distance_m > 0.0 and not positive_lengths:
            raise RuntimeError(
                "cannot cover the simulation horizon on a cyclic route "
                "without positive-length edges")
        if maximum_edges > MAXIMUM_CYCLE_CONSTRUCTION_EDGES:
            raise RuntimeError(
                "route continuation search depth exceeds the cyclic route "
                "construction safety limit (%d)" %
                MAXIMUM_CYCLE_CONSTRUCTION_EDGES)
        if minimum_cycle_distance_m > 0.0:
            distance_ratio = (
                minimum_cycle_distance_m / min(positive_lengths))
            distance_edge_count = int(math.ceil(min(
                distance_ratio,
                float(MAXIMUM_CYCLE_CONSTRUCTION_EDGES))))
        else:
            distance_edge_count = 0
        # The graph-size factor gives deterministic selection room to pass
        # through zero-length connector-like normal edges, while remaining a
        # strict finite guard for malformed zero-progress cycles.
        construction_edge_limit = min(
            MAXIMUM_CYCLE_CONSTRUCTION_EDGES,
            max(
                maximum_edges,
                maximum_edges + distance_edge_count * max(
                    1, len(network.connections))))
        cycle_path = []
        cycle_distance = 0.0
        current_edge = final_edge
        discovered_terminal = None
        route_edge_history = set(recorded_edges)
        repeated_endpoint = False
        for decision in range(construction_edge_limit):
            if (repeated_endpoint and
                    len(cycle_path) >= maximum_edges and
                    cycle_distance >= minimum_cycle_distance_m):
                break
            candidates = [
                candidate for candidate in outgoing(current_edge)
                if candidate.to_edge in viable_edges
            ]
            if not candidates:
                if terminal_kind(current_edge) == "boundary_terminal":
                    discovered_terminal = current_edge
                    break
                raise RuntimeError(
                    "actor %s has no valid boundary-or-cycle continuation; "
                    "candidate path ended at interior edge %s" %
                    (actor_id, current_edge))
            minimum_visits = min(
                visit_counts.get(item.to_edge, 0) for item in candidates)
            candidates = [
                item for item in candidates
                if visit_counts.get(item.to_edge, 0) == minimum_visits]
            selected = _stable_connection_order(
                candidates, seed, actor_id, decision)[0]
            cycle_path.append(selected)
            cycle_distance += _connection_route_length(network, selected)
            current_edge = selected.to_edge
            repeated_endpoint = current_edge in route_edge_history
            route_edge_history.add(current_edge)
            visit_counts[current_edge] = visit_counts.get(current_edge, 0) + 1
        if not cycle_path:
            raise RuntimeError(
                "actor %s has no valid boundary-or-cycle continuation from "
                "edge %s" % (actor_id, final_edge))
        if discovered_terminal is not None:
            return [item.to_edge for item in cycle_path], {
                "status": "extended_to_boundary_terminal",
                "terminal_edge": discovered_terminal,
                "terminal_boundary_distance_m": _reported_boundary_distance(
                    network, discovered_terminal),
                "directions": [item.direction for item in cycle_path],
            }
        if not repeated_endpoint:
            raise RuntimeError(
                "cyclic route construction safety limit was exhausted before "
                "reaching a boundary terminal or closing a reusable cycle")
        if cycle_distance + 1.0e-9 < minimum_cycle_distance_m:
            raise RuntimeError(
                "cyclic route construction safety limit was exhausted before "
                "covering %.3f m (planned %.3f m)" % (
                    minimum_cycle_distance_m, cycle_distance))
        return [item.to_edge for item in cycle_path], {
            "status": "extended_bounded_cycle",
            "terminal_edge": cycle_path[-1].to_edge,
            "terminal_boundary_distance_m": _reported_boundary_distance(
                network, cycle_path[-1].to_edge),
            "directions": [item.direction for item in cycle_path],
            "minimum_cycle_distance_m": minimum_cycle_distance_m,
            "planned_cycle_distance_m": cycle_distance,
            "construction_edge_limit": construction_edge_limit,
            "horizon_covered": True,
        }

    terminal_edge = path[-1].to_edge if path else final_edge
    return [item.to_edge for item in path], {
        "status": "extended_to_boundary_terminal",
        "terminal_edge": terminal_edge,
        "terminal_boundary_distance_m": _reported_boundary_distance(
            network, terminal_edge),
        "directions": [item.direction for item in path],
    }


def schedule_departure_times(recorded_times, start_delay=0.0,
                             maximum_gap=None):
    """Map recorded start groups to an ordered, optionally gap-capped schedule."""
    try:
        start_delay = float(start_delay)
    except (TypeError, ValueError):
        raise ValueError("moving_start_delay must be a non-negative number")
    if not math.isfinite(start_delay) or start_delay < 0.0:
        raise ValueError("moving_start_delay must be a non-negative number")
    if maximum_gap is not None:
        try:
            maximum_gap = float(maximum_gap)
        except (TypeError, ValueError):
            raise ValueError("maximum_departure_gap must be positive")
        if not math.isfinite(maximum_gap) or maximum_gap <= 0.0:
            raise ValueError("maximum_departure_gap must be positive")

    groups = sorted(set(float(value) for value in recorded_times))
    if any(not math.isfinite(value) or value < 0.0 for value in groups):
        raise ValueError("recorded departure times must be non-negative")
    if not groups:
        return {}

    # ``start_delay`` is an internal bridge offset, not a normalization of the
    # recording.  A scene whose first moving actor appears at 5 s must still
    # begin that actor at 5 s (plus the bridge offset), rather than at startup.
    scheduled = {groups[0]: round(groups[0] + start_delay, 12)}
    for previous, current in zip(groups, groups[1:]):
        gap = current - previous
        if maximum_gap is not None:
            gap = min(gap, maximum_gap)
        scheduled[current] = round(scheduled[previous] + gap, 12)
    return scheduled


def _safe_id(value):
    return re.sub(r"[^A-Za-z0-9_.:-]", "_", str(value))


_VTYPE_COMMON = {
    "carFollowModel": "IDM",
    # CARLA 0.9.15's SumoSimulation unconditionally enables
    # ``--lateral-resolution 0.25``; sublane simulation requires SL2015.
    "laneChangeModel": "SL2015",
    "emergencyDecel": "9.0",
    "lcStrategic": "1.0",
    "lcCooperative": "0.6",
    "lcSpeedGain": "0.8",
    "lcKeepRight": "0.5",
    "lcAssertive": "1.0",
}

_VTYPE_DEFINITIONS = {
    "nusc_car": dict(vClass="passenger", guiShape="passenger", length="4.7",
                     width="1.85", accel="2.6", decel="4.5", minGap="2.5", tau="1.2"),
    "nusc_truck": dict(vClass="truck", guiShape="truck", length="7.5",
                       width="2.5", accel="1.3", decel="3.5", minGap="3.0", tau="1.5"),
    "nusc_bus": dict(vClass="bus", guiShape="bus", length="11.0",
                     width="2.5", accel="1.2", decel="3.5", minGap="3.0", tau="1.5"),
    "nusc_motorcycle": dict(vClass="motorcycle", guiShape="motorcycle", length="2.2",
                            width="0.8", accel="3.0", decel="5.0", minGap="1.2", tau="1.0"),
    "nusc_bicycle": dict(vClass="bicycle", guiShape="bicycle", length="1.8",
                         width="0.7", accel="1.2", decel="3.0", minGap="1.0", tau="1.0"),
}


def _add_vtypes(root):
    for type_id, values in _VTYPE_DEFINITIONS.items():
        ET.SubElement(root, "vType", {
            "id": type_id, **_VTYPE_COMMON, **values})


def _vtype_for(track):
    kind = actor_kind(track.category)
    return "nusc_%s" % (kind if kind in {"truck", "bus", "motorcycle", "bicycle"}
                         else "car")


def _sumo_behavior_baseline(track):
    """Return explicit per-class behavior values written to the SUMO vType."""
    base_type = _vtype_for(track)
    values = {**_VTYPE_COMMON, **_VTYPE_DEFINITIONS[base_type]}
    return {
        "car_following": {
            "tau_s": float(values["tau"]),
            "min_gap_m": float(values["minGap"]),
            "accel_mps2": float(values["accel"]),
            "decel_mps2": float(values["decel"]),
            "apparent_decel_mps2": float(values["decel"]),
            "emergency_decel_mps2": float(values["emergencyDecel"]),
        },
        "lane_changing": {
            "lc_strategic": float(values["lcStrategic"]),
            "lc_cooperative": float(values["lcCooperative"]),
            "lc_speed_gain": float(values["lcSpeedGain"]),
            "lc_keep_right": float(values["lcKeepRight"]),
            "lc_assertive": float(values["lcAssertive"]),
        },
    }


def _moving_speed_ceiling(track, policy, minimum_speed):
    if policy == "recorded_profile":
        return max(minimum_speed, track.peak_speed)
    if policy == "recorded_mean":
        return max(minimum_speed, track.mean_speed)
    # SUMO's default vType maximum is 55.55 m/s.  The network's lane speed
    # normally imposes a lower limit, so using the vType default is a safe
    # route-distance bound for the explicitly unbounded policy.
    return DEFAULT_SUMO_VTYPE_MAX_SPEED_MPS


def _add_recorded_speed_vtype(root, track, actor_id, maximum_speed):
    base_type = _vtype_for(track)
    type_id = "%s_%s" % (base_type, _safe_id(actor_id))
    ET.SubElement(root, "vType", {
        "id": type_id,
        **_VTYPE_COMMON,
        **_VTYPE_DEFINITIONS[base_type],
        "maxSpeed": "%.3f" % maximum_speed,
        "speedFactor": "1.0",
        "speedDev": "0.0",
    })
    return type_id


def write_sumo_scenario(network_path, tracks, output_dir, scene_name,
                        maximum_snap_distance=8.0, excluded_actor_ids=(),
                        step_length=0.05, end_time=None,
                        minimum_track_distance=DEFAULT_MINIMUM_TRACK_DISTANCE_M,
                        minimum_two_point_speed=DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS,
                        moving_start_delay=0.0,
                        maximum_departure_gap=None,
                        eager_insert=False,
                        moving_route_continuation="none",
                        route_continuation_seed=0,
                        maximum_route_continuation_edges=64,
                        allow_continuation_uturns=False,
                        boundary_terminal_tolerance=15.0,
                        moving_initial_pose="recorded",
                        moving_speed_policy="recorded_profile",
                        minimum_moving_speed=0.1,
                        terminal_stop_policy="release_to_sumo",
                        terminal_stop_maximum_extent=1.0,
                        terminal_stop_minimum_duration=2.0,
                        sumo_seed=0):
    os.makedirs(output_dir, exist_ok=True)
    network = load_network(network_path)
    root = ET.Element("routes")
    _add_vtypes(root)
    # Validate once even when the scene contains no vehicle tracks.
    minimum_track_distance = validated_minimum_track_distance(
        minimum_track_distance)
    minimum_two_point_speed = validated_minimum_two_point_speed(
        minimum_two_point_speed)
    if moving_route_continuation not in ("none", "terminal"):
        raise ValueError(
            "moving_route_continuation must be none or terminal")
    if moving_initial_pose not in ("recorded", "route"):
        raise ValueError(
            "moving_initial_pose must be recorded or route")
    if moving_speed_policy not in (
            "recorded_profile", "recorded_mean", "unbounded"):
        raise ValueError(
            "moving_speed_policy must be recorded_profile, recorded_mean, "
            "or unbounded")
    if terminal_stop_policy not in ("release_to_sumo", "preserve_recorded"):
        raise ValueError(
            "terminal_stop_policy must be release_to_sumo or preserve_recorded")
    try:
        minimum_moving_speed = float(minimum_moving_speed)
        terminal_stop_maximum_extent = float(
            terminal_stop_maximum_extent)
        terminal_stop_minimum_duration = float(
            terminal_stop_minimum_duration)
    except (TypeError, ValueError):
        raise ValueError(
            "moving speed and terminal-stop thresholds must be positive")
    if not math.isfinite(minimum_moving_speed) or minimum_moving_speed <= 0.0:
        raise ValueError("minimum_moving_speed must be positive")
    if (not math.isfinite(terminal_stop_maximum_extent) or
            terminal_stop_maximum_extent <= 0.0 or
            not math.isfinite(terminal_stop_minimum_duration) or
            terminal_stop_minimum_duration <= 0.0):
        raise ValueError(
            "terminal-stop extent and duration must be positive")
    if isinstance(maximum_route_continuation_edges, bool):
        raise ValueError("route continuation search depth must be positive")
    try:
        converted_continuation_edges = int(maximum_route_continuation_edges)
        exact_continuation_edges = float(maximum_route_continuation_edges)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("route continuation search depth must be positive")
    if (not math.isfinite(exact_continuation_edges) or
            exact_continuation_edges != converted_continuation_edges or
            converted_continuation_edges <= 0):
        raise ValueError(
            "route continuation search depth must be a positive integer")
    maximum_route_continuation_edges = converted_continuation_edges
    try:
        route_continuation_seed = int(route_continuation_seed)
        sumo_seed = int(sumo_seed)
        boundary_terminal_tolerance = float(boundary_terminal_tolerance)
    except (TypeError, ValueError):
        raise ValueError("route continuation/SUMO seed and boundary tolerance are invalid")
    if (not math.isfinite(boundary_terminal_tolerance) or
            boundary_terminal_tolerance < 0.0):
        raise ValueError("boundary_terminal_tolerance must be non-negative")
    # Validate the requested schedule even when no moving route survives.
    schedule_departure_times(
        (), moving_start_delay, maximum_departure_gap)
    if end_time is None:
        scenario_end_time = max(
            (track.end_time for track in tracks.values()),
            default=20.0) + 5.0
    else:
        try:
            scenario_end_time = float(end_time)
        except (TypeError, ValueError):
            raise ValueError("end_time must be a non-negative number")
        if (not math.isfinite(scenario_end_time) or
                scenario_end_time < 0.0):
            raise ValueError("end_time must be a non-negative number")
    scenario_end_time += float(moving_start_delay)

    report = {
        "scene": scene_name,
        "maximum_snap_distance_m": float(maximum_snap_distance),
        "maximum_route_heading_error_degrees": math.degrees(
            MAXIMUM_ROUTE_HEADING_ERROR_RAD),
        "maximum_same_edge_regression_m": MAXIMUM_SAME_EDGE_REGRESSION_M,
        "minimum_track_distance_m": minimum_track_distance,
        "minimum_two_point_speed_mps": minimum_two_point_speed,
        "moving_departure_schedule": {
            "start_delay_s": float(moving_start_delay),
            "maximum_departure_gap_s": (
                None if maximum_departure_gap is None
                else float(maximum_departure_gap)),
            "eager_insert": bool(eager_insert),
        },
        "moving_route_continuation": {
            "policy": moving_route_continuation,
            "seed": int(route_continuation_seed),
            "runtime_extend_at_route_end": (
                moving_route_continuation == "terminal"),
            "runtime_selection_policy": (
                "preserve_prepared_route_then_guarded_persistent_stop_"
                "recovery"),
            "tail_extension_selection_policy": (
                "seeded_random_viable_outgoing_before_route_end"),
            "persistent_stop_recovery": {
                "minimum_duration_s": 1.0,
                "stop_speed_mps": 0.1,
                "resume_speed_mps": 0.3,
                "progress_reset_distance_m": 0.5,
                "close_leader_distance_m": 15.0,
                "traffic_signal_guard_distance_m": 15.0,
                "downstream_blocker_minimum_duration_s": 1.0,
                "maximum_suffix_recovery_attempts": 3,
            },
            # Schema-v1 compatibility for report consumers.  This legacy key
            # is a search/minimum-tail setting, not a hard duration-cycle cap.
            "maximum_edges": int(maximum_route_continuation_edges),
            "maximum_edges_deprecated": True,
            "terminal_search_depth_edges": int(
                maximum_route_continuation_edges),
            "minimum_cyclic_tail_edges": int(
                maximum_route_continuation_edges),
            "cyclic_construction_safety_limit_edges": int(
                MAXIMUM_CYCLE_CONSTRUCTION_EDGES),
            "duration_sized_cycles": True,
            "allow_uturns": bool(allow_continuation_uturns),
            "boundary_terminal_tolerance_m": float(
                boundary_terminal_tolerance),
        },
        "moving_initialization": {
            "position_policy": moving_initial_pose,
        },
        "moving_speed": {
            "policy": moving_speed_policy,
            "minimum_mps": minimum_moving_speed,
            "terminal_stop_policy": terminal_stop_policy,
            "terminal_stop_maximum_extent_m": (
                terminal_stop_maximum_extent),
            "terminal_stop_minimum_duration_s": (
                terminal_stop_minimum_duration),
        },
        "included": [],
        "carla_static": [],
        "skipped": [],
    }
    excluded = set(excluded_actor_ids)
    planned = []

    for actor_id, track in sorted(tracks.items(), key=lambda item: item[1].start_time):
        if actor_id in excluded:
            report["skipped"].append({"id": actor_id, "reason": "CARLA authority"})
            continue
        if not track.is_vehicle:
            report["skipped"].append({"id": actor_id, "reason": "not a vehicle"})
            continue
        motion = classify_vehicle_motion(
            track, minimum_track_distance, minimum_two_point_speed)
        if motion in (None, "static"):
            report["carla_static"].append({
                "id": actor_id,
                "category": track.category,
                "authority": "carla_static",
                "motion_classification": (
                    "single_observation_static" if motion is None else
                    "recorded_static"),
                "depart": track.start_time,
                "recorded_start_time": track.start_time,
                "samples": len(track.points),
                "recorded_distance_m": track.distance,
                "motion_extent_m": track.motion_extent,
                "net_displacement_m": track.net_displacement,
                "mean_speed_mps": track.mean_speed,
            })
            continue
        plan, error = plan_track_route(network, track, maximum_snap_distance)
        if error:
            report["skipped"].append({"id": actor_id, "reason": error})
            continue
        planned.append((actor_id, track, plan))

    departure_times = schedule_departure_times(
        [plan["first_matched_time"] for _, _, plan in planned],
        moving_start_delay, maximum_departure_gap)
    scheduled = [
        (departure_times[plan["first_matched_time"]], actor_id, track, plan)
        for actor_id, track, plan in planned
    ]

    for depart, actor_id, track, plan in sorted(
            scheduled, key=lambda item: (item[0], item[1])):
        route_id = "route_%s" % _safe_id(actor_id)
        route_speed_ceiling = _moving_speed_ceiling(
            track, moving_speed_policy, minimum_moving_speed)
        if moving_speed_policy in ("recorded_profile", "recorded_mean"):
            maximum_speed = route_speed_ceiling
            type_id = _add_recorded_speed_vtype(
                root, track, actor_id, maximum_speed)
        else:
            maximum_speed = None
            type_id = _vtype_for(track)
        vehicle_length = float(
            _VTYPE_DEFINITIONS[_vtype_for(track)]["length"])
        continuation_horizon = max(0.0, scenario_end_time - depart)
        # Budget the whole remaining horizon against continuation edges.  This
        # deliberately does not subtract the recorded prefix, and includes one
        # simulation step plus the vehicle length, so the route cannot expire
        # from a boundary/equality rounding case before the configured stop.
        minimum_cycle_distance = (
            route_speed_ceiling * (
                continuation_horizon + max(0.0, float(step_length))) +
            vehicle_length)
        if moving_route_continuation == "terminal":
            continuation, continuation_report = extend_route_to_terminal(
                network, plan["recorded_edges"], actor_kind(track.category),
                actor_id, seed=route_continuation_seed,
                maximum_edges=maximum_route_continuation_edges,
                allow_uturns=allow_continuation_uturns,
                boundary_tolerance=boundary_terminal_tolerance,
                minimum_cycle_distance_m=minimum_cycle_distance)
            continuation_report["simulation_horizon_s"] = continuation_horizon
            continuation_report["speed_bound_mps"] = route_speed_ceiling
        else:
            continuation = []
            continuation_report = {
                "status": "disabled",
                "terminal_edge": plan["recorded_edges"][-1],
                "terminal_boundary_distance_m": _reported_boundary_distance(
                    network, plan["recorded_edges"][-1]),
                "directions": [],
            }
        plan["continuation_edges"] = continuation
        plan["continuation"] = continuation_report
        plan["edges"] = plan["recorded_edges"] + continuation
        ET.SubElement(root, "route", {
            "id": route_id,
            "edges": " ".join(plan["edges"]),
        })
        attributes = {
            "id": "nusc_%s" % _safe_id(actor_id),
            "type": type_id,
            "route": route_id,
            "depart": "%.3f" % depart,
            "departLane": str(plan["depart_lane"]),
            "departPos": "%.3f" % plan["depart_pos"],
            # Starting at the recorded lane position can be close to a signal or
            # junction. A zero insertion speed is accepted there reliably; the
            # selected car-following model takes authority immediately after.
            "departSpeed": "0",
        }
        ET.SubElement(root, "vehicle", attributes)
        report["included"].append({
            "id": actor_id,
            "sumo_id": attributes["id"],
            "category": track.category,
            "recorded_start_time": track.start_time,
            "first_matched_time": plan["first_matched_time"],
            "depart": depart,
            "recorded_initial_speed_mps": track.initial_speed,
            "mean_speed_mps": track.mean_speed,
            "peak_speed_mps": track.peak_speed,
            "motion_extent_m": track.motion_extent,
            "net_displacement_m": track.net_displacement,
            "sumo_max_speed_mps": maximum_speed,
            "sumo_vehicle_length_m": vehicle_length,
            "sumo_type_id": type_id,
            "sumo_behavior_baseline": _sumo_behavior_baseline(track),
            **plan,
        })

    if hasattr(ET, "indent"):
        ET.indent(root, space="  ")
    routes_path = os.path.join(output_dir, "routes.rou.xml")
    ET.ElementTree(root).write(routes_path, encoding="utf-8", xml_declaration=True)

    config_root = ET.Element("configuration")
    input_node = ET.SubElement(config_root, "input")
    ET.SubElement(input_node, "net-file", {"value": os.path.basename(network_path)})
    ET.SubElement(input_node, "route-files", {"value": os.path.basename(routes_path)})
    time_node = ET.SubElement(config_root, "time")
    ET.SubElement(time_node, "begin", {"value": "0"})
    ET.SubElement(time_node, "end", {"value": "%.3f" % scenario_end_time})
    ET.SubElement(time_node, "step-length", {"value": "%.3f" % step_length})
    processing = ET.SubElement(config_root, "processing")
    ET.SubElement(processing, "time-to-teleport", {"value": "-1"})
    if eager_insert:
        ET.SubElement(processing, "eager-insert", {"value": "true"})
    random_number = ET.SubElement(config_root, "random_number")
    ET.SubElement(random_number, "seed", {"value": str(sumo_seed)})
    if hasattr(ET, "indent"):
        ET.indent(config_root, space="  ")
    config_path = os.path.join(output_dir, "scene.sumocfg")
    ET.ElementTree(config_root).write(config_path, encoding="utf-8", xml_declaration=True)

    report_path = os.path.join(output_dir, "route_report.json")
    with open(report_path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=False)
        stream.write("\n")
    return config_path, report_path, report
