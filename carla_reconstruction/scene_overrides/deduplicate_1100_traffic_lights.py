"""Scene-1100-only removal of the extra traffic-light mast on roads 7/8.

Retain physical signal 20001 on road 8. Replace road 7's redundant signals by
a reference to 20001, preserving both lanes' stop-line applicability and their
existing controller. No road geometry, junction links, meshes or other signals
are changed. Default is an offline preview; --apply updates only the installed
Decorated XODR after making an exclusive, byte-for-byte backup.
"""

import argparse
import copy
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ASSET = "Nusc_singapore_hollandvillage_1100"
KEEP = "20001"
REMOVE = {"20004", "20005", "20006"}
EXPECTED_SIGNALS = {"20000", "20001", "20002", "20003", "20004", "20005", "20006", "20007"}
CONTROLLER = "30000"
ROAD_ANCHORS = {
    "7": (10.907781, 64.423578, 61.185299, -2.275048),
    "8": (10.573005, 62.142985, 63.860508, -2.265929),
}
POSITIONS = {"7": (10.3078, -3.2449), "8": (9.9731, -3.1336)}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def structural(node):
    return (node.tag, sorted(node.attrib.items()), (node.text or "").strip(),
            [structural(child) for child in node])


def unrelated_structure(root):
    other = copy.deepcopy(root)
    for road in other.findall("road"):
        if road.get("id") in ROAD_ANCHORS:
            road.remove(road.find("signals"))
    for controller in other.findall("controller"):
        if controller.get("id") == CONTROLLER:
            for control in list(controller):
                if control.tag == "control" and control.get("signalId") in REMOVE:
                    controller.remove(control)
    return structural(other)


def validate_layout(root):
    if root.tag != "OpenDRIVE":
        raise ValueError("Expected an OpenDRIVE document")
    roads = {road.get("id"): road for road in root.findall("road")}
    for road_id, anchor in ROAD_ANCHORS.items():
        road = roads.get(road_id)
        if road is None or road.get("junction") != "-1":
            raise ValueError("Not the expected scene-1100 approach roads")
        geom = road.find("planView/geometry")
        if geom is None or road.find("signals") is None:
            raise ValueError("Scene-1100 approach geometry/signals missing")
        actual = (float(road.get("length")), float(geom.get("x")),
                  float(geom.get("y")), float(geom.get("hdg")))
        if any(not math.isclose(a, b, rel_tol=0, abs_tol=1e-6) for a, b in zip(actual, anchor)):
            raise ValueError("Approach geometry differs from scene-1100; refusing correction")
    controllers = [c for c in root.findall("controller") if c.get("id") == CONTROLLER]
    if len(controllers) != 1:
        raise ValueError("Expected one scene-1100 signal controller")
    return roads, controllers[0]


def _check_signal(element, road_id, signal_id, reference=False):
    expected_tag = "signalReference" if reference else "signal"
    if element.tag != expected_tag or element.get("id") != signal_id:
        raise ValueError("Unexpected approach signal identity")
    if (element.get("orientation") != "+" or
            not math.isclose(float(element.get("s")), POSITIONS[road_id][0], abs_tol=1e-6) or
            not math.isclose(float(element.get("t")), POSITIONS[road_id][1], abs_tol=1e-6)):
        raise ValueError("Approach signal position/orientation changed")
    if [v.attrib for v in element.findall("validity")] != [{"fromLane": "-1", "toLane": "-1"}]:
        raise ValueError("Unexpected signal lane applicability")
    if not reference and (element.get("dynamic") != "yes" or element.get("type") != "1000001"):
        raise ValueError("Expected a dynamic traffic light")


def patch_xodr(data):
    """Return (patched bytes, report); reject any ambiguous or changed approach."""
    text = data.decode("utf-8")
    root = ET.fromstring(text)
    roads, controller = validate_layout(root)
    original_other = unrelated_structure(root)
    signals = root.findall("road/signals/signal")
    ids = [s.get("id") for s in signals]
    control_ids = [c.get("signalId") for c in controller.findall("control")]
    already = not REMOVE.intersection(ids)
    expected = EXPECTED_SIGNALS - REMOVE if already else EXPECTED_SIGNALS
    if len(ids) != len(expected) or set(ids) != expected or set(control_ids) != expected or len(control_ids) != len(expected):
        raise ValueError("Unexpected signal/controller IDs; refusing a partial or unrelated correction")
    # A merged physical signal cannot silently inherit another controller/phase.
    for other in root.findall("controller"):
        if other is not controller and any(c.get("signalId") in REMOVE | {KEEP} for c in other.findall("control")):
            raise ValueError("Approach signals belong to different controllers")
    expected_by_road = {"7": [KEEP] if already else ["20004", "20005"],
                        "8": [KEEP] if already else [KEEP, "20006"]}
    for road_id, expected_ids in expected_by_road.items():
        entries = list(roads[road_id].find("signals"))
        if [s.get("id") for s in entries] != expected_ids:
            raise ValueError("Unexpected signal entries on approach road " + road_id)
        for signal in entries:
            _check_signal(signal, road_id, signal.get("id"), reference=already and road_id == "7")
    if any(r.get("id") in REMOVE for r in root.findall(".//signalReference")):
        raise ValueError("Removed signals have additional references; manual review required")
    report = {"scene": "singapore-hollandvillage_scene-1100", "already_applied": already,
              "retained_signal": KEEP, "removed_signal_ids": sorted(REMOVE),
              "controlled_roads_after": [7, 8], "controller": CONTROLLER,
              "signal_count_before": len(signals), "signal_count_after": len(expected) if already else len(signals) - len(REMOVE),
              "unrelated_geometry_and_signals_unchanged": True}
    if already:
        return data, report

    # Check duplicate attributes too: never merge different lamp types/headings.
    for road_id in ("7", "8"):
        entries = list(roads[road_id].find("signals"))
        normalized = [{k: v for k, v in s.attrib.items() if k not in ("id", "name")} for s in entries]
        if normalized[0] != normalized[1] or [structural(c) for c in entries[0]] != [structural(c) for c in entries[1]]:
            raise ValueError("Approach signals are not exact same-lane duplicates")
    seven = roads["7"].find("signals")
    old = seven[0]
    reference = ET.Element("signalReference", {k: old.get(k) for k in ("s", "t", "orientation")})
    reference.set("id", KEEP)
    for validity in old.findall("validity"):
        reference.append(copy.deepcopy(validity))
    seven[:] = [reference]
    eight = roads["8"].find("signals")
    eight.remove(eight[1])
    for control in list(controller):
        if control.get("signalId") in REMOVE:
            controller.remove(control)
    if unrelated_structure(root) != original_other:
        raise RuntimeError("Correction would change unrelated map data")

    # Replace only the three edited XML blocks, retaining all other file bytes.
    result = text
    for road_id in ("7", "8"):
        block = re.compile(r'(<road\b[^>]*\bid="' + road_id + r'"[^>]*>)(.*?)(</road>)', re.S)
        matches = list(block.finditer(result))
        if len(matches) != 1:
            raise ValueError("Ambiguous road XML block")
        signals_node = roads[road_id].find("signals")
        ET.indent(signals_node, space="    ", level=2)
        replacement = ET.tostring(signals_node, encoding="unicode").rstrip()
        match = matches[0]
        body, count = re.subn(r"<signals>.*?</signals>", lambda _: replacement, match.group(2), count=1, flags=re.S)
        if count != 1:
            raise ValueError("Missing road signals block")
        result = result[:match.start()] + match.group(1) + body + match.group(3) + result[match.end():]
    block = re.compile(r'<controller\b[^>]*\bid="' + CONTROLLER + r'"[^>]*>.*?</controller>', re.S)
    matches = list(block.finditer(result))
    if len(matches) != 1:
        raise ValueError("Ambiguous controller XML block")
    ET.indent(controller, space="    ", level=1)
    replacement = ET.tostring(controller, encoding="unicode").rstrip()
    result = block.sub(lambda _: replacement, result, count=1)
    output = result.encode("utf-8")
    # Verify the actual serialized output, including unchanged road/junction data.
    if unrelated_structure(ET.fromstring(output)) != original_other:
        raise RuntimeError("Serialized correction changed unrelated map data")
    return output, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--carla-root", default="C:/carla")
    parser.add_argument("--output", required=True, help="new/empty preview and audit directory")
    parser.add_argument("--apply", action="store_true", help="back up and update only scene-1100 Decorated XODR")
    args = parser.parse_args()
    target = Path(args.carla_root).resolve() / "Unreal/CarlaUE4/Content" / ASSET / "Maps" / ASSET / "OpenDrive" / (ASSET + "_Decorated.xodr")
    output = Path(args.output).resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        parser.error("--output must be new or empty")
    original = target.read_bytes()
    updated, report = patch_xodr(original)
    output.mkdir(parents=True, exist_ok=True)
    (output / target.name).write_bytes(updated)
    report.update(target=str(target), original_sha256=digest(original), corrected_sha256=digest(updated), applied=False)
    if args.apply and updated != original:
        backup = target.with_name(target.name + ".pre-1100-light-dedup-" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".bak")
        with backup.open("xb") as stream:
            stream.write(original)
        if backup.read_bytes() != original or target.read_bytes() != original:
            raise RuntimeError("Backup verification or source recheck failed; not applying")
        temporary = target.with_name(target.name + ".dedup.tmp")
        with temporary.open("xb") as stream:
            stream.write(updated)
        temporary.replace(target)
        if target.read_bytes() != updated:
            raise RuntimeError("Installed XODR verification failed; restore the backup")
        report.update(applied=True, backup=str(backup))
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
