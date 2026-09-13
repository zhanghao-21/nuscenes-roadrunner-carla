"""Optional, temporary weather and lane-obstacle cases for the TM runner."""

import math


def add_environment_arguments(parser):
    parser.add_argument("--weather", choices=("default", "clear", "rain"), default="default",
                        help="temporary weather; rain uses CARLA HardRainNoon")
    parser.add_argument("--obstacle-distance", type=float,
                        help="place one fixed barrier this many metres ahead along the ego lane")
    parser.add_argument("--obstacle-blueprint", default="static.prop.streetbarrier",
                        help="CARLA static.prop.* blueprint for the lane obstacle")


def validate_environment_arguments(args):
    distance = getattr(args, "obstacle_distance", None)
    if distance is not None and (not math.isfinite(distance) or distance < 5.0):
        raise ValueError("--obstacle-distance must be finite and at least 5 metres")
    if distance is not None and not args.obstacle_blueprint.startswith("static.prop."):
        raise ValueError("--obstacle-blueprint must be a static.prop.* object")


def waypoint_ahead(start, distance):
    """Follow the actual directed lane, refusing to guess at a route fork."""
    current = start
    remaining = distance
    while remaining > 1e-6:
        step = min(1.0, remaining)
        candidates = list(current.next(step))
        if len(candidates) != 1:
            raise ValueError("obstacle placement reaches a lane fork or dead end; "
                             "use a shorter --obstacle-distance")
        current = candidates[0]
        remaining -= step
    return current


class EnvironmentScenario:
    def __init__(self, args, world):
        self.args = args
        self.world = world
        self.original_weather = None
        self.obstacle = None
        self.metadata = {"weather": getattr(args, "weather", "default"), "obstacle": None}

    def apply_weather(self):
        import carla
        selection = self.metadata["weather"]
        if selection == "default":
            return
        self.original_weather = self.world.get_weather()
        weather = (carla.WeatherParameters.HardRainNoon if selection == "rain"
                   else carla.WeatherParameters.ClearNoon)
        self.world.set_weather(weather)
        self.metadata["weather_parameters"] = {
            key: float(getattr(weather, key)) for key in (
                "cloudiness", "precipitation", "precipitation_deposits", "wetness",
                "wind_intensity", "sun_azimuth_angle", "sun_altitude_angle",
                "fog_density", "fog_distance", "fog_falloff")}

    def spawn_obstacle(self, ego_location, carla_map):
        import carla
        distance = getattr(self.args, "obstacle_distance", None)
        if distance is None:
            return None
        # Use the known spawn pose: actor.get_location() can still return the
        # default zero location until CARLA publishes its first actor snapshot.
        start = carla_map.get_waypoint(ego_location, project_to_road=True,
                                      lane_type=carla.LaneType.Driving)
        if start is None or str(start.lane_type) != "Driving":
            raise ValueError("ego has no driving lane for obstacle placement")
        target = waypoint_ahead(start, distance)
        blueprint = self.world.get_blueprint_library().find(self.args.obstacle_blueprint)
        transform = target.transform
        road_z = transform.location.z
        # CARLA static-prop bounds can be world-axis-aligned. Inspect at zero
        # yaw so choosing the long axis does not depend on the road heading.
        spawn_transform = carla.Transform(
            carla.Location(transform.location.x, transform.location.y, road_z + 0.2),
            carla.Rotation())
        actor = self.world.try_spawn_actor(blueprint, spawn_transform)
        if actor is None:
            raise RuntimeError("cannot spawn lane obstacle; location may be occupied")
        self.obstacle = actor  # retain immediately so even configuration failures clean up
        actor.set_simulate_physics(False)
        bbox = actor.bounding_box
        # Put the long horizontal dimension across the lane.
        if bbox.extent.x > bbox.extent.y:
            transform.rotation.yaw += 90.0
        transform.location.z = road_z + bbox.extent.z - bbox.location.z + 0.02
        actor.set_transform(transform)
        self.metadata["obstacle"] = {
            "actor_id": int(actor.id), "blueprint": actor.type_id,
            "distance_ahead_along_lane_m": float(distance),
            "ego_start_carla_xyz_m": [ego_location.x, ego_location.y, ego_location.z],
            "road_id": target.road_id, "lane_id": target.lane_id, "s": target.s,
            "carla_xyz_m": [transform.location.x, transform.location.y, transform.location.z],
            "yaw_degrees": transform.rotation.yaw,
            "dimensions_m": [2 * bbox.extent.x, 2 * bbox.extent.y, 2 * bbox.extent.z],
            "physics": "fixed collision-enabled prop", "source": "synthetic_scenario_obstacle",
        }
        return actor

    def close(self):
        try:
            if self.obstacle is not None and self.obstacle.is_alive:
                self.obstacle.destroy()
        finally:
            if self.original_weather is not None:
                self.world.set_weather(self.original_weather)
