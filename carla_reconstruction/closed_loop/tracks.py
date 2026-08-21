"""Load nuScenes replay metadata as controller-independent actor tracks."""

from dataclasses import dataclass, field
import json
import math
import os


DYNAMIC_PREFIXES = ("vehicle.", "human.pedestrian.")


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
    def mean_speed(self):
        if self.duration <= 1.0e-9:
            return 0.0
        return self.distance / self.duration

    @property
    def initial_speed(self):
        if len(self.points) < 2:
            return 0.0
        a, b = self.points[0], self.points[1]
        dt = b.time - a.time
        return math.hypot(b.x - a.x, b.y - a.y) / dt if dt > 0 else 0.0


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
