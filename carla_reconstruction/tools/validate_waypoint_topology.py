#!/usr/bin/env python3
"""Validate CARLA waypoint transitions directly from a persistent-map XODR."""

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

import carla


def _next_roads(carla_map, road, station, distance=1.0):
    waypoint = carla_map.get_waypoint_xodr(road, -1, max(0.01, station))
    return sorted({item.road_id for item in waypoint.next(distance)}) if waypoint else []


def validate(path, spacing=1.0):
    with open(path, "r", encoding="utf-8") as stream:
        xodr = stream.read()
    root = ET.fromstring(xodr)
    roads = {int(item.get("id")): item for item in root.findall("road")}
    carla_map = carla.Map(os.path.basename(path), xodr)
    incoming_failures = []
    connections = []
    for junction in root.findall("junction"):
        for connection in junction.findall("connection"):
            incoming = int(connection.get("incomingRoad"))
            connecting = int(connection.get("connectingRoad"))
            connections.append((incoming, connecting))
            station = float(roads[incoming].get("length")) - 0.25
            found = _next_roads(carla_map, incoming, station)
            if connecting not in found:
                incoming_failures.append({
                    "incoming_road": incoming,
                    "expected_connecting_road": connecting,
                    "found_road_ids": found,
                })

    connector_ids = {
        road_id for road_id, road in roads.items()
        if road.get("junction", "-1") != "-1"
    }
    outgoing_failures = []
    for connector in sorted(connector_ids):
        successor = roads[connector].find("./link/successor")
        if successor is None or successor.get("elementType") != "road":
            continue
        expected = int(successor.get("elementId"))
        station = float(roads[connector].get("length")) - 0.25
        found = _next_roads(carla_map, connector, station)
        if expected not in found:
            outgoing_failures.append({
                "connecting_road": connector,
                "expected_outgoing_road": expected,
                "found_road_ids": found,
            })

    waypoints = carla_map.generate_waypoints(spacing)
    report = {
        "xodr": os.path.abspath(path),
        "roads": len(roads),
        "junctions": len(root.findall("junction")),
        "waypoints": len(waypoints),
        "junction_waypoints": sum(item.is_junction for item in waypoints),
        "topology_edges": len(carla_map.get_topology()),
        "incoming_transitions": len(connections),
        "incoming_failures": incoming_failures,
        "outgoing_transitions": len(connector_ids),
        "outgoing_failures": outgoing_failures,
    }
    report["valid"] = not incoming_failures and not outgoing_failures
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xodr", required=True)
    parser.add_argument("--spacing", type=float, default=1.0)
    parser.add_argument("--output")
    args = parser.parse_args()
    report = validate(os.path.abspath(args.xodr), args.spacing)
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        with open(os.path.abspath(args.output), "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    if not report["valid"]:
        sys.exit(2)


if __name__ == "__main__":
    main()
