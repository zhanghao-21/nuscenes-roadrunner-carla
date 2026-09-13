"""CARLA-specific helpers kept separate from offline/testable modules."""

from datetime import datetime
import math
import os

import carla

from replay_geo import replay_geo_carla as legacy

from .tracks import carla_pose, simplify_points


class GroundProjector:
    def __init__(self, carla_map):
        self.carla_map = carla_map
        self.cache = {}

    def z(self, x, y):
        key = (round(x), round(y))
        if key not in self.cache:
            waypoint = self.carla_map.get_waypoint(
                carla.Location(x=x, y=y, z=0.0), project_to_road=True,
                lane_type=carla.LaneType.Driving)
            self.cache[key] = waypoint.transform.location.z if waypoint else 0.0
        return self.cache[key]


def spawn_track_actor(world, track, projector, role_name, physics=True):
    point = track.points[0]
    x, y, yaw = carla_pose(point)
    kind = "car" if track.is_ego else legacy._category_kind(track.category)
    key = sum(bytearray(track.actor_id.encode("utf-8")))
    blueprint = legacy._pick_blueprint(world, kind, key)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", role_name)
    transform = carla.Transform(
        carla.Location(x=x, y=y, z=projector.z(x, y) + 1.0),
        carla.Rotation(yaw=yaw))
    actor = world.try_spawn_actor(blueprint, transform)
    if actor is None:
        transform.location.z += 2.0
        actor = world.try_spawn_actor(blueprint, transform)
    if actor is None and not physics:
        # CARLA's spawn collision check can reject a dense but valid recorded
        # parked-vehicle layout.  Create a physics-disabled actor well above
        # the scene, then place it at the authoritative recorded pose below.
        # This mirrors the elevated-spawn technique used by CARLA's SUMO
        # bridge, without bypassing collision checks for physics actors.
        transform.location.z = projector.z(x, y) + 25.0
        actor = world.try_spawn_actor(blueprint, transform)
    if actor is None:
        return None, None
    try:
        actor.set_simulate_physics(bool(physics))
    except Exception:
        pass
    offset = legacy._z_offset(actor)
    if not physics:
        # Physics-disabled replay/static actors do not settle under gravity.
        # Put their bounding-box bottom on the projected road immediately.
        place_from_point(actor, point, projector, offset)
    return actor, offset


def place_from_point(actor, point, projector, z_offset):
    x, y, yaw = carla_pose(point)
    actor.set_transform(carla.Transform(
        carla.Location(x=x, y=y, z=projector.z(x, y) + z_offset),
        carla.Rotation(yaw=yaw)))


def carla_path(track, minimum_spacing=2.0):
    result = []
    for point in simplify_points(track.points, minimum_spacing):
        x, y, _ = carla_pose(point)
        result.append((x, y))
    return result


def configure_tm_actor(traffic_manager, actor, track, tm_port,
                       minimum_spacing=2.0, leading_distance=2.5,
                       auto_lane_change=False, minimum_speed_kmh=3.0,
                       behavior_variant=None):
    behavior = dict(behavior_variant or {})
    desired_speed_scale = float(behavior.get("desired_speed_scale", 1.0))
    effective_leading_distance = float(behavior.get(
        "leading_distance_m", leading_distance))
    effective_auto_lane_change = bool(behavior.get(
        "auto_lane_change", auto_lane_change))
    actor.set_autopilot(True, tm_port)
    traffic_manager.auto_lane_change(actor, effective_auto_lane_change)
    traffic_manager.distance_to_leading_vehicle(
        actor, effective_leading_distance)
    ignore_vehicles = float(behavior.get("ignore_vehicles_percentage", 0.0))
    ignore_lights = float(behavior.get("ignore_lights_percentage", 0.0))
    ignore_signs = float(behavior.get("ignore_signs_percentage", 0.0))
    traffic_manager.ignore_vehicles_percentage(actor, ignore_vehicles)
    traffic_manager.ignore_walkers_percentage(actor, 0.0)
    traffic_manager.ignore_lights_percentage(actor, ignore_lights)
    traffic_manager.ignore_signs_percentage(actor, ignore_signs)
    speed_floor = float(behavior.get("target_speed_floor_kmh", 0.0))
    speed_ceiling = float(behavior.get("target_speed_ceiling_kmh", 0.0))
    desired_speed = max(
        float(minimum_speed_kmh), speed_floor,
        track.mean_speed * 3.6 * desired_speed_scale)
    if speed_ceiling > 0.0:
        desired_speed = min(desired_speed, speed_ceiling)
    traffic_manager.set_desired_speed(actor, desired_speed)
    if behavior_variant is not None:
        traffic_manager.random_left_lanechange_percentage(
            actor, float(behavior["random_left_lane_change_percentage"]))
        traffic_manager.random_right_lanechange_percentage(
            actor, float(behavior["random_right_lane_change_percentage"]))
        traffic_manager.keep_right_rule_percentage(
            actor, float(behavior["keep_right_rule_percentage"]))
    locations = [carla.Location(x=x, y=y, z=0.0)
                 for x, y in carla_path(track, minimum_spacing)]
    if len(locations) >= 2:
        traffic_manager.set_path(actor, locations[1:])
    return {
        "desired_speed_kmh": desired_speed,
        "desired_speed_scale": desired_speed_scale,
        "recorded_mean_speed_kmh": track.mean_speed * 3.6,
        "target_speed_floor_kmh": speed_floor,
        "target_speed_ceiling_kmh": speed_ceiling,
        "ignore_vehicles_percentage": ignore_vehicles,
        "ignore_walkers_percentage": 0.0,
        "ignore_lights_percentage": ignore_lights,
        "ignore_signs_percentage": ignore_signs,
        "leading_distance_m": effective_leading_distance,
        "auto_lane_change": effective_auto_lane_change,
        "random_left_lane_change_percentage": float(behavior.get(
            "random_left_lane_change_percentage", 0.0)),
        "random_right_lane_change_percentage": float(behavior.get(
            "random_right_lane_change_percentage", 0.0)),
        "keep_right_rule_percentage": float(behavior.get(
            "keep_right_rule_percentage", 0.0)),
    }


def actor_state(actor):
    transform = actor.get_transform()
    location = transform.location
    velocity = actor.get_velocity()
    extent = actor.bounding_box.extent
    yaw = math.radians(transform.rotation.yaw)
    return {
        "position": (location.x, location.y),
        "velocity": (velocity.x, velocity.y),
        "heading": (math.cos(yaw), math.sin(yaw)),
        "radius": math.hypot(extent.x, extent.y),
    }


def chase_transform(actor):
    transform = actor.get_transform()
    yaw = math.radians(transform.rotation.yaw)
    location = carla.Location(
        x=transform.location.x - 10.0 * math.cos(yaw),
        y=transform.location.y - 10.0 * math.sin(yaw),
        z=transform.location.z + 5.0)
    return carla.Transform(location, carla.Rotation(
        pitch=-15.0, yaw=transform.rotation.yaw))


class CollisionMonitor:
    def __init__(self, world, parent, metrics, time_provider):
        blueprint = world.get_blueprint_library().find("sensor.other.collision")
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=parent)
        self.metrics = metrics
        self.time_provider = time_provider
        self.sensor.listen(self._on_collision)

    def _on_collision(self, event):
        impulse = event.normal_impulse
        magnitude = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        self.metrics.record_collision(
            self.time_provider(), getattr(event.other_actor, "id", "unknown"), magnitude)

    def destroy(self):
        try:
            self.sensor.stop()
            self.sensor.destroy()
        except Exception:
            pass


def default_run_output(repo_root, scene, mode):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(repo_root, "carla_reconstruction", "generated",
                        "closed_loop_runs", scene, "%s_%s" % (mode, stamp))
