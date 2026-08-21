#!/usr/bin/env python3
"""Draw CARLA's actual waypoint graph, including junction connectors."""

import argparse

import carla


NORMAL_COLOR = carla.Color(30, 190, 255)
JUNCTION_COLOR = carla.Color(255, 120, 20)
DEAD_END_COLOR = carla.Color(255, 30, 60)


def raised(location, height):
    return carla.Location(location.x, location.y, location.z + height)


def run(args):
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    carla_map = world.get_map()
    waypoints = carla_map.generate_waypoints(args.spacing)
    junction_count = 0
    line_count = 0
    dead_ends = set()

    for waypoint in waypoints:
        successors = waypoint.next(args.spacing)
        touches_junction = waypoint.is_junction or any(
            successor.is_junction for successor in successors)
        if args.junction_only and not touches_junction:
            continue
        location = raised(waypoint.transform.location, args.height)
        color = JUNCTION_COLOR if waypoint.is_junction else NORMAL_COLOR
        if waypoint.is_junction:
            junction_count += 1
        if not successors:
            dead_ends.add((waypoint.road_id, waypoint.lane_id))
            color = DEAD_END_COLOR
        world.debug.draw_point(
            location, size=args.point_size, color=color,
            life_time=args.life_time, persistent_lines=False)
        for successor in successors:
            next_location = raised(successor.transform.location, args.height)
            segment_color = (JUNCTION_COLOR if
                             waypoint.is_junction or successor.is_junction
                             else NORMAL_COLOR)
            world.debug.draw_line(
                location, next_location, thickness=args.thickness,
                color=segment_color, life_time=args.life_time,
                persistent_lines=False)
            line_count += 1

    print("Map:", carla_map.name)
    print("Generated waypoints:", len(waypoints))
    print("Junction waypoints:", sum(item.is_junction for item in waypoints))
    print("Lines drawn:", line_count)
    print("Dead-end road/lane pairs:", sorted(dead_ends))
    print("Legend: cyan=ordinary lane, orange=junction connector, red=dead end")
    print("Drawing remains visible for %.1f seconds." % args.life_time)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--height", type=float, default=0.35)
    parser.add_argument("--life-time", type=float, default=120.0)
    parser.add_argument("--point-size", type=float, default=0.06)
    parser.add_argument("--thickness", type=float, default=0.04)
    parser.add_argument("--junction-only", action="store_true")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
