"""Load nuScenes replay metadata as controller-independent actor tracks."""

from dataclasses import dataclass, field
import json
import math
import os


DYNAMIC_PREFIXES = ("vehicle.", "human.pedestrian.")
DEFAULT_MINIMUM_TRACK_DISTANCE_M = 2.0
DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS = 1.0
DEFAULT_MAXIMUM_TWO_POINT_DURATION_S = 0.5


def normalize_angle(angle):
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def interpolate_angle(left, right, ratio):
    return left + normalize_angle(right - left) * ratio


def actor_kind(category):
    if category.startswith("human.pedestrian"):
        return "pedestrian"
    for kind in ("bicycle", "motorcycle", "bus", "truck"):
        if kind in category:
            return kind
    return "car"


def validated_minimum_track_distance(value):
    """Return a finite, non-negative movement-classification boundary."""
    if isinstance(value, bool):
        raise ValueError("minimum_track_distance must be a non-negative number")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(
            "minimum_track_distance must be a non-negative number")
    if not math.isfinite(result) or result < 0.0:
        raise ValueError("minimum_track_distance must be a non-negative number")
    return result


def validated_minimum_two_point_speed(value):
    """Return a finite, non-negative two-observation speed boundary."""
    if isinstance(value, bool):
        raise ValueError(
            "minimum_two_point_speed must be a non-negative number")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(
            "minimum_two_point_speed must be a non-negative number")
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(
            "minimum_two_point_speed must be a non-negative number")
    return result


def classify_vehicle_motion(
        track, minimum_track_distance=DEFAULT_MINIMUM_TRACK_DISTANCE_M,
        minimum_two_point_speed=DEFAULT_MINIMUM_TWO_POINT_SPEED_MPS):
    """Classify a recorded vehicle track as ``moving``, ``static``, or absent.

    This shared classifier is used by both the Traffic Manager and hybrid
    baselines. Tracks with insufficient observations and non-vehicles return
    ``None`` because their motion state cannot be established here.
    """
    threshold = validated_minimum_track_distance(minimum_track_distance)
    short_speed = validated_minimum_two_point_speed(minimum_two_point_speed)
    if not track.is_vehicle or len(track.points) < 2:
        return None
    # Accumulated annotation jitter can exceed a distance threshold even when
    # a parked actor never leaves a small area.  Spatial extent measures actual
    # separation between observations and is robust to that back-and-forth
    # noise.  A two-observation track gets one narrow velocity escape hatch so
    # a genuinely moving actor seen only at the end of a scene is not lost.
    moving = track.motion_extent >= threshold
    if (len(track.points) == 2 and
            track.duration <= DEFAULT_MAXIMUM_TWO_POINT_DURATION_S + 1.0e-9 and
            track.mean_speed >= short_speed):
        moving = True
    return "moving" if moving else "static"


@dataclass(frozen=True)
class TrackPoint:
    frame: int
    time: float
    x: float
    y: float
    yaw: float
    wlh: tuple = field(default_factory=tuple)


@dataclass
class ActorTrack:
    actor_id: str
    category: str
    points: list
    is_ego: bool = False

    @property
    def kind(self):
        return "ego" if self.is_ego else actor_kind(self.category)

    @property
    def is_vehicle(self):
        return self.is_ego or self.category.startswith("vehicle.")

    @property
    def start_time(self):
        return self.points[0].time

    @property
    def end_time(self):
        return self.points[-1].time

    @property
    def duration(self):
        return max(0.0, self.end_time - self.start_time)

    @property
    def distance(self):
        return sum(math.hypot(b.x - a.x, b.y - a.y)
                   for a, b in zip(self.points, self.points[1:]))

    @property
    def net_displacement(self):
        if len(self.points) < 2:
            return 0.0
        first, last = self.points[0], self.points[-1]
        return math.hypot(last.x - first.x, last.y - first.y)

    @property
    def motion_extent(self):
        """Maximum separation between any two recorded observations."""
        maximum = 0.0
        for index, left in enumerate(self.points):
            for right in self.points[index + 1:]:
                maximum = max(
                    maximum, math.hypot(
                        right.x - left.x, right.y - left.y))
        return maximum

    @property
    def mean_speed(self):
        if self.duration <= 1.0e-9:
            return 0.0
        return self.distance / self.duration

    @property
    def peak_speed(self):
        speeds = [
            math.hypot(right.x - left.x, right.y - left.y) /
            (right.time - left.time)
            for left, right in zip(self.points, self.points[1:])
            if right.time > left.time
        ]
        return max(speeds, default=0.0)

    @property
    def initial_speed(self):
        if len(self.points) < 2:
            return 0.0
        a, b = self.points[0], self.points[1]
        dt = b.time - a.time
        return math.hypot(b.x - a.x, b.y - a.y) / dt if dt > 0 else 0.0


def recorded_speed_at(track, time_seconds):
    """Return the recorded segment speed, or ``None`` outside the track."""
    if (len(track.points) < 2 or
            time_seconds < track.start_time or
            time_seconds >= track.end_time):
        return None
    for left, right in zip(track.points, track.points[1:]):
        if left.time <= time_seconds < right.time:
            dt = right.time - left.time
            if dt <= 0.0:
                continue
            return math.hypot(
                right.x - left.x, right.y - left.y) / dt
    return None


@dataclass
class TrackBundle:
    scene: str
    frame_dt: float
    ego: ActorTrack
    actors: dict

    @property
    def duration(self):
        return self.ego.end_time

    @property
    def vehicle_tracks(self):
        return {actor_id: track for actor_id, track in self.actors.items()
                if track.is_vehicle}


def load_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def load_manifest_tracks(manifest_path):
    manifest_path = os.path.abspath(manifest_path)
    manifest = load_json(manifest_path)
    meta_path = manifest["trajectories"]["meta"]
    if not os.path.isabs(meta_path):
        meta_path = os.path.join(os.path.dirname(manifest_path), meta_path)
    meta = load_json(meta_path)
    return manifest, tracks_from_meta(meta)


def tracks_from_meta(meta):
    frame_dt = float(meta.get("frame_dt", 0.5))
    ego_points = []
    for frame, item in enumerate(meta.get("ego_trajectory_local", [])):
        ego_points.append(TrackPoint(
            frame=frame,
            time=frame * frame_dt,
            x=float(item["x"]),
            y=float(item["y"]),
            yaw=float(item["yaw"])))
    if not ego_points:
        raise ValueError("metadata contains no ego_trajectory_local points")

    grouped = {}
    categories = {}
    for frame, annotations in enumerate(meta.get("agent_frames", [])):
        for item in annotations:
            category = item.get("category", "")
            if not category.startswith(DYNAMIC_PREFIXES):
                continue
            actor_id = str(item["id"])
            grouped.setdefault(actor_id, []).append(TrackPoint(
                frame=frame,
                time=frame * frame_dt,
                x=float(item["x"]),
                y=float(item["y"]),
                yaw=float(item["yaw"]),
                wlh=tuple(float(value) for value in item.get("wlh", []))))
            categories.setdefault(actor_id, category)

    actors = {
        actor_id: ActorTrack(actor_id, categories[actor_id], points)
        for actor_id, points in grouped.items()
        if points
    }
    ego = ActorTrack("ego", "vehicle.ego", ego_points, is_ego=True)
    return TrackBundle(str(meta.get("scene", "unknown")), frame_dt, ego, actors)


def point_at(track, time_seconds):
    """Linearly interpolate a track pose at simulation time."""
    if time_seconds <= track.points[0].time:
        return track.points[0]
    if time_seconds >= track.points[-1].time:
        return track.points[-1]
    for left, right in zip(track.points, track.points[1:]):
        if left.time <= time_seconds <= right.time:
            span = right.time - left.time
            ratio = 0.0 if span <= 0 else (time_seconds - left.time) / span
            return TrackPoint(
                frame=left.frame,
                time=time_seconds,
                x=left.x + (right.x - left.x) * ratio,
                y=left.y + (right.y - left.y) * ratio,
                yaw=interpolate_angle(left.yaw, right.yaw, ratio),
                wlh=left.wlh or right.wlh)
    return track.points[-1]


def simplify_points(points, minimum_spacing=2.0):
    """Keep route points far enough apart while always retaining the end."""
    if not points:
        return []
    result = [points[0]]
    for point in points[1:-1]:
        if math.hypot(point.x - result[-1].x,
                      point.y - result[-1].y) >= minimum_spacing:
            result.append(point)
    if len(points) > 1 and points[-1] != result[-1]:
        result.append(points[-1])
    return result


def carla_pose(point):
    """Convert an OpenDRIVE-local track point to CARLA's left-handed frame."""
    return point.x, -point.y, -math.degrees(point.yaw)
