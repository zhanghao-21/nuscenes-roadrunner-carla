"""Guarded, one-time TM ego route correction for the tested scene-0103 map.

Raw points inside its junctions are ambiguous to TM's ImportPath. Non-junction
exit-lane anchors select the recorded corridor without changing driving rules.
This module is used only by the CARLA-only runner, not the shared controller.
"""

from dataclasses import dataclass
import hashlib
import json
import math

from carla_reconstruction.closed_loop.tracks import carla_pose


SCENE_ID = "boston-seaport_scene-0103"
OVERRIDE_ID = "0103_tm_ego_exit_lane_anchors_v1"
EXPECTED_MAP_NAME = (
    "Nusc_boston_seaport_0103/Maps/Nusc_boston_seaport_0103/"
    "Nusc_boston_seaport_0103_Decorated")
EXPECTED_TRACK_SHA256 = (
    "33d07473f0446c85b50fc3b396e5e304dc9793e213487414c451909bd31e5c6f")
ANCHOR_SPECS = ((17, -1, 155.0), (9, -1, 5.0), (12, -1, 5.0),
                (11, -1, 15.0), (11, -1, 25.0))
EXPECTED_ANCHOR_XYZ = (
    (60.882523, -117.005676, 0.0),
    (88.759972, -95.125473, 0.0),
    (118.599289, -69.650894, 0.0),
    (137.132675, -53.359085, 0.0),
    (144.649475, -46.763809, 0.0),
)
ROAD_SEQUENCE = (17, 69, 9, 56, 12, 80, 11)


@dataclass
class PreparedEgoRoute:
    locations: tuple
    metadata: dict


def _reject(reason):
    raise ValueError(
        "Scene-0103 TM ego route correction refused: %s. "
        "Use the matching Decorated map and original ego track, or explicitly "
        "add --disable-0103-ego-route to use the original raw-path behavior."
        % reason)


def _track_fingerprint(track):
    # Stable against insignificant serialization precision; also guards against
    # a different ego, coordinate frame, route, or heading under the same name.
    poses = [[round(float(value), 5) for value in carla_pose(point)]
             for point in track.points]
    return hashlib.sha256(json.dumps(
        poses, separators=(",", ":"), allow_nan=False).encode("ascii")).hexdigest()


def _normalized_map_name(name):
    name = name.replace("\\", "/").strip("/")
    return name[5:] if name.startswith("Game/") else name


def _validate_connections(start):
    """Require CARLA's *directed* waypoint API to traverse the known corridor."""
    pending = [(start, 0)]
    seen = set()
    while pending and len(seen) < 1000:
        waypoint, index = pending.pop()
        key = (index, round(waypoint.s, 4))
        if key in seen:
            continue
        seen.add(key)
        if index == len(ROAD_SEQUENCE) - 1 and waypoint.s >= ANCHOR_SPECS[-1][2]:
            return
        for candidate in waypoint.next(1.0):
            if candidate.lane_id != -1 or str(candidate.lane_type) != "Driving":
                continue
            if candidate.road_id == ROAD_SEQUENCE[index]:
                if candidate.s > waypoint.s:
                    pending.append((candidate, index))
            elif (index + 1 < len(ROAD_SEQUENCE) and
                  candidate.road_id == ROAD_SEQUENCE[index + 1]):
                pending.append((candidate, index + 1))
    _reject("loaded map does not connect roads 17 -> 69 -> 9 -> 56 -> 12 -> 80 -> 11")


def prepare_ego_route(manifest, ego_track, carla_map, ego_mode, disabled=False):
    """Validate before spawning; return None outside this narrow exception.

    No world, actor, or TM state is modified here. Applicable but incompatible
    inputs are rejected rather than silently applying hard-coded road IDs.
    """
    if disabled or ego_mode != "tm" or manifest.get("scene") != SCENE_ID:
        return None
    if _normalized_map_name(carla_map.name) != EXPECTED_MAP_NAME:
        _reject("actual loaded map is %r" % carla_map.name)
    if (not ego_track.is_ego or ego_track.actor_id != "ego" or
            _track_fingerprint(ego_track) != EXPECTED_TRACK_SHA256):
        _reject("recorded ego track differs from the validated scene-0103 track")

    waypoints = []
    records = []
    for (road, lane, distance), expected_xyz in zip(ANCHOR_SPECS, EXPECTED_ANCHOR_XYZ):
        waypoint = carla_map.get_waypoint_xodr(road, lane, distance)
        if (waypoint is None or waypoint.road_id != road or
                waypoint.lane_id != lane or waypoint.is_junction or
                str(waypoint.lane_type) != "Driving" or
                not math.isfinite(waypoint.s) or abs(waypoint.s - distance) > 0.001):
            _reject("missing or incompatible non-junction anchor road %d lane %d s=%.1f"
                    % (road, lane, distance))
        location = waypoint.transform.location
        xyz = (location.x, location.y, location.z)
        if (not all(math.isfinite(value) for value in xyz) or
                math.dist(xyz, expected_xyz) > 0.25):
            _reject("anchor geometry changed on road %d at s=%.1f" % (road, distance))
        waypoints.append(waypoint)
        records.append({"road_id": road, "lane_id": lane, "s": distance,
                        "carla_xyz_m": list(xyz)})
    _validate_connections(waypoints[0])
    return PreparedEgoRoute(
        locations=tuple(waypoint.transform.location for waypoint in waypoints),
        metadata={
            "override_id": OVERRIDE_ID,
            "applied": False,
            "scene": SCENE_ID,
            "loaded_map": carla_map.name,
            "ego_track_sha256": EXPECTED_TRACK_SHA256,
            "anchors": records,
            "road_sequence": list(ROAD_SEQUENCE),
            "end_policy": "native_tm_continuation_after_final_anchor",
        })
