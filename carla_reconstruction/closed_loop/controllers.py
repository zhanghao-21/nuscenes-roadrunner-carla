"""CARLA-physics reference controller for ego or designated critical actors."""

import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def _speed(vehicle):
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)


def _radius(actor):
    extent = actor.bounding_box.extent
    return math.hypot(extent.x, extent.y)


class ReferencePathController:
    """Pure-pursuit steering plus speed/headway control using CARLA physics."""

    def __init__(self, path, desired_speed, time_headway=1.4, minimum_gap=2.5,
                 lookahead_time=0.8, minimum_lookahead=4.0, speed_kp=0.45,
                 max_steer_angle=0.65, events=None):
        if len(path) < 2:
            raise ValueError("reference controller needs at least two path points")
        self.path = list(path)
        self.desired_speed = max(0.0, float(desired_speed))
        self.time_headway = max(0.1, float(time_headway))
        self.minimum_gap = max(0.0, float(minimum_gap))
        self.lookahead_time = max(0.1, float(lookahead_time))
        self.minimum_lookahead = max(1.0, float(minimum_lookahead))
        self.speed_kp = max(0.01, float(speed_kp))
        self.max_steer_angle = max(0.1, float(max_steer_angle))
        self.events = list(events or [])
        self.index = 0

    def _nearest_index(self, location):
        lower = max(0, self.index - 3)
        upper = min(len(self.path), self.index + 40)
        self.index = min(
            range(lower, upper),
            key=lambda index: ((self.path[index][0] - location.x) ** 2 +
                               (self.path[index][1] - location.y) ** 2))
        return self.index

    def _target(self, location, lookahead):
        index = self._nearest_index(location)
        travelled = 0.0
        while index + 1 < len(self.path) and travelled < lookahead:
            left, right = self.path[index], self.path[index + 1]
            travelled += math.hypot(right[0] - left[0], right[1] - left[1])
            index += 1
        self.index = max(self.index, index - 1)
        return self.path[index]

    def _lead_gap(self, vehicle, other_vehicles):
        transform = vehicle.get_transform()
        yaw = math.radians(transform.rotation.yaw)
        forward = (math.cos(yaw), math.sin(yaw))
        right = (-forward[1], forward[0])
        location = transform.location
        best = float("inf")
        own_radius = _radius(vehicle)
        for other in other_vehicles:
            if other.id == vehicle.id or not other.is_alive:
                continue
            other_location = other.get_location()
            dx, dy = other_location.x - location.x, other_location.y - location.y
            longitudinal = dx * forward[0] + dy * forward[1]
            lateral = abs(dx * right[0] + dy * right[1])
            if longitudinal <= 0.0 or lateral > 2.8:
                continue
            gap = longitudinal - own_radius - _radius(other)
            if gap < best:
                best = gap
        return best

    def _event_override(self, elapsed):
        for event in self.events:
            if event.get("type") != "brake":
                continue
            start = float(event.get("start", 0.0))
            duration = float(event.get("duration", 1.0))
            if start <= elapsed < start + duration:
                return clamp(float(event.get("intensity", 1.0)), 0.0, 1.0)
        return None

    def run_step(self, vehicle, other_vehicles, elapsed=0.0):
        """Return a ``carla.VehicleControl`` command for the current tick."""
        import carla

        transform = vehicle.get_transform()
        current_speed = _speed(vehicle)
        lookahead = max(self.minimum_lookahead, current_speed * self.lookahead_time)
        target_x, target_y = self._target(transform.location, lookahead)
        dx = target_x - transform.location.x
        dy = target_y - transform.location.y
        yaw = math.radians(transform.rotation.yaw)
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(local_y, max(0.1, local_x))
        wheelbase = max(2.0, vehicle.bounding_box.extent.x * 1.6)
        steer_angle = math.atan2(2.0 * wheelbase * math.sin(alpha), lookahead)
        steer = clamp(steer_angle / self.max_steer_angle, -1.0, 1.0)

        target_speed = self.desired_speed
        lead_gap = self._lead_gap(vehicle, other_vehicles)
        if math.isfinite(lead_gap):
            safe_speed = max(0.0, (lead_gap - self.minimum_gap) / self.time_headway)
            target_speed = min(target_speed, safe_speed)
        error = target_speed - current_speed
        throttle = clamp(self.speed_kp * error, 0.0, 0.75)
        brake = clamp(-self.speed_kp * error, 0.0, 1.0)
        emergency_brake = self._event_override(elapsed)
        if emergency_brake is not None:
            throttle, brake = 0.0, emergency_brake
        return carla.VehicleControl(throttle=throttle, steer=steer, brake=brake)
