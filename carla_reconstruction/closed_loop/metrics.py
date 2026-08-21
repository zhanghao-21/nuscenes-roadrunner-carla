"""Safety surrogate metrics shared by the TM and SUMO hybrid runners."""

import csv
import json
import math
import os


def distance_2d(left, right):
    return math.hypot(right[0] - left[0], right[1] - left[1])


def closing_ttc(ego_position, ego_velocity, other_position, other_velocity,
                clearance=0.0):
    """Return line-of-sight closing TTC, or infinity when actors separate."""
    rx = other_position[0] - ego_position[0]
    ry = other_position[1] - ego_position[1]
    distance = math.hypot(rx, ry)
    gap = max(0.0, distance - max(0.0, clearance))
    if gap <= 1.0e-9:
        return 0.0
    rvx = other_velocity[0] - ego_velocity[0]
    rvy = other_velocity[1] - ego_velocity[1]
    closing_speed = -(rx * rvx + ry * rvy) / distance
    if closing_speed <= 1.0e-9:
        return float("inf")
    return gap / closing_speed


class SafetyMetrics:
    """Accumulate ego-centric distance, TTC, speed, and collision metrics."""

    def __init__(self):
        self.rows = []
        self.collisions = []
        self.minimum_distance = float("inf")
        self.minimum_ttc = float("inf")
        self.minimum_distance_actor = None
        self.minimum_ttc_actor = None

    def update(self, time_seconds, ego, others):
        """Update from dictionaries containing position, velocity, and radius."""
        best_distance = float("inf")
        best_distance_id = None
        best_ttc = float("inf")
        best_ttc_id = None
        for actor_id, state in others.items():
            distance = distance_2d(ego["position"], state["position"])
            clearance = float(ego.get("radius", 0.0)) + float(state.get("radius", 0.0))
            ttc = closing_ttc(
                ego["position"], ego["velocity"], state["position"],
                state["velocity"], clearance=clearance)
            if distance < best_distance:
                best_distance, best_distance_id = distance, actor_id
            if ttc < best_ttc:
                best_ttc, best_ttc_id = ttc, actor_id

        if best_distance < self.minimum_distance:
            self.minimum_distance = best_distance
            self.minimum_distance_actor = best_distance_id
        if best_ttc < self.minimum_ttc:
            self.minimum_ttc = best_ttc
            self.minimum_ttc_actor = best_ttc_id

        speed = math.hypot(*ego["velocity"])
        self.rows.append({
            "time_s": float(time_seconds),
            "ego_speed_mps": speed,
            "nearest_actor": best_distance_id or "",
            "distance_m": best_distance,
            "ttc_s": best_ttc,
            "collision_count": len(self.collisions),
        })

    def record_collision(self, time_seconds, other_actor_id, impulse=0.0):
        self.collisions.append({
            "time_s": float(time_seconds),
            "other_actor_id": str(other_actor_id),
            "impulse": float(impulse),
        })

    @staticmethod
    def _finite(value):
        return value if math.isfinite(value) else None

    def summary(self):
        return {
            "samples": len(self.rows),
            "collision_count": len(self.collisions),
            "collisions": list(self.collisions),
            "minimum_distance_m": self._finite(self.minimum_distance),
            "minimum_distance_actor": self.minimum_distance_actor,
            "minimum_ttc_s": self._finite(self.minimum_ttc),
            "minimum_ttc_actor": self.minimum_ttc_actor,
        }

    def write(self, output_dir, run_metadata=None):
        os.makedirs(output_dir, exist_ok=True)
        csv_path = os.path.join(output_dir, "metrics.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as stream:
            fieldnames = ["time_s", "ego_speed_mps", "nearest_actor",
                          "distance_m", "ttc_s", "collision_count"]
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows:
                clean = dict(row)
                if not math.isfinite(clean["distance_m"]):
                    clean["distance_m"] = ""
                if not math.isfinite(clean["ttc_s"]):
                    clean["ttc_s"] = ""
                writer.writerow(clean)

        summary = self.summary()
        if run_metadata is not None:
            summary["run"] = run_metadata
        summary_path = os.path.join(output_dir, "summary.json")
        with open(summary_path, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=False)
            stream.write("\n")
        return csv_path, summary_path
