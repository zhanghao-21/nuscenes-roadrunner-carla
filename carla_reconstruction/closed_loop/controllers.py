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


def endpoint_progress(path, x, y, minimum_segment=0):
    """Measure monotonic path distance remaining to a finite endpoint.

    The nearest projection supplies remaining polyline distance on curved
    approaches. ``signed_final_distance`` separately identifies an overshoot
    once the actor is on the last non-zero segment, so pursuit never turns it
    back into an endpoint orbit.
    """
    end_x, end_y = path[-1]
    to_end_x = end_x - float(x)
    to_end_y = end_y - float(y)
    distance = math.hypot(to_end_x, to_end_y)
    segment_lengths = [
        math.hypot(right[0] - left[0], right[1] - left[1])
        for left, right in zip(path, path[1:])
    ]
    cumulative = [0.0]
    for segment_length in segment_lengths:
        cumulative.append(cumulative[-1] + segment_length)
    total_length = cumulative[-1]

    final_segment = None
    final_tangent = None
    for index in range(len(segment_lengths) - 1, -1, -1):
        segment_length = segment_lengths[index]
        if segment_length <= 1.0e-9:
            continue
        left, right = path[index], path[index + 1]
        final_segment = index
        final_tangent = (
            (right[0] - left[0]) / segment_length,
            (right[1] - left[1]) / segment_length,
        )
        break
    if final_segment is None:
        return {
            "remaining_distance": 0.0,
            "signed_final_distance": 0.0,
            "endpoint_distance": distance,
            "projected_segment": None,
            "on_final_segment": True,
        }

    lower = max(0, min(int(minimum_segment), final_segment))
    best = None
    for index in range(lower, len(segment_lengths)):
        segment_length = segment_lengths[index]
        if segment_length <= 1.0e-9:
            continue
        left, right = path[index], path[index + 1]
        tangent_x = (right[0] - left[0]) / segment_length
        tangent_y = (right[1] - left[1]) / segment_length
        projection = clamp(
            ((float(x) - left[0]) * tangent_x +
             (float(y) - left[1]) * tangent_y),
            0.0, segment_length)
        projected_x = left[0] + projection * tangent_x
        projected_y = left[1] + projection * tangent_y
        error_squared = (
            (float(x) - projected_x) ** 2 +
            (float(y) - projected_y) ** 2)
        path_progress = cumulative[index] + projection
        candidate = (error_squared, -path_progress, index, path_progress)
        if best is None or candidate < best:
            best = candidate

    projected_segment = best[2]
    path_progress = best[3]
    signed_final_distance = (
        to_end_x * final_tangent[0] + to_end_y * final_tangent[1])
    return {
        "remaining_distance": max(0.0, total_length - path_progress),
        "signed_final_distance": signed_final_distance,
        "endpoint_distance": distance,
        "projected_segment": projected_segment,
        "on_final_segment": projected_segment >= final_segment,
    }


class ReferencePathController:
    """Pure-pursuit steering plus speed/headway control using CARLA physics."""

    def __init__(self, path, desired_speed, time_headway=1.4, minimum_gap=2.5,
                 lookahead_time=0.8, minimum_lookahead=4.0, speed_kp=0.45,
                 max_steer_angle=0.65, events=None,
                 endpoint_stop_tolerance=0.75,
                 endpoint_deceleration=3.0):
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
        self.endpoint_stop_tolerance = max(
            0.05, float(endpoint_stop_tolerance))
        self.endpoint_deceleration = max(
            0.1, float(endpoint_deceleration))
        self.endpoint_held = False
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

    @staticmethod
    def _hold_control():
        import carla

        return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)

    def run_step(self, vehicle, other_vehicles, elapsed=0.0,
                 speed_limit=None):
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
        if speed_limit is not None:
            target_speed = min(target_speed, max(0.0, float(speed_limit)))
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

    def run_step_to_endpoint(self, vehicle, other_vehicles, elapsed=0.0):
        """Approach the final recorded position, then latch a straight hold.

        The speed cap follows a constant-deceleration stopping envelope. The
        signed final-segment test prevents pure pursuit from turning back after
        an overshoot, while the latch ensures later physics motion cannot make
        the controller resume and orbit the endpoint.
        """
        if self.endpoint_held:
            return self._hold_control(), True

        location = vehicle.get_location()
        progress = endpoint_progress(
            self.path, location.x, location.y,
            minimum_segment=max(0, self.index - 3))
        projected_segment = progress["projected_segment"]
        if projected_segment is not None:
            self.index = max(self.index, projected_segment)
        if (progress["endpoint_distance"] <= self.endpoint_stop_tolerance or
                (progress["on_final_segment"] and
                 progress["signed_final_distance"] <= 0.0)):
            self.endpoint_held = True
            return self._hold_control(), True

        stopping_distance = max(
            0.0, max(
                progress["remaining_distance"],
                progress["endpoint_distance"]) -
            self.endpoint_stop_tolerance)
        speed_limit = math.sqrt(
            2.0 * self.endpoint_deceleration * stopping_distance)
        return (self.run_step(
            vehicle, other_vehicles, elapsed=elapsed,
            speed_limit=speed_limit), False)
