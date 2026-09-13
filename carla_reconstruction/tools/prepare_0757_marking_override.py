"""Prepare an opt-in visual style override for Boston scene 0757 only.

The five source road-divider curves lack paint metadata. Double-solid yellow
is an explicit reconstruction assumption, not a verified nuScenes annotation.
Their surveyed geometry and all lane-divider annotations remain unchanged.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path


DIVIDERS = (
    "08e3c7cc-e6ea-4b67-aa76-bdea92f7121b",
    "0f925058-4f3b-47c7-a2d5-150e2f05c6db",
    "208bfd41-15bd-4f80-aaab-4c44cdfd641e",
    "7a18040b-f676-4a2a-9e0c-2baf2dc5b191",
    "8b0ff47b-2ba8-4231-acde-fe032098e319",
)


def make_override(manifest, source):
    if (manifest.get("scene") != "boston-seaport_scene-0757" or
            manifest.get("nuscenes_map") != "boston-seaport" or
            manifest.get("map", {}).get("asset_name") != "Nusc_boston_seaport_0757"):
        raise ValueError("This override is restricted to Nusc_boston_seaport_0757")
    result = copy.deepcopy(source)
    records = {record["token"]: record for record in result["road_divider"]}
    lines = {line["token"]: line["node_tokens"] for line in result["line"]}
    for token in DIVIDERS:
        if token not in records:
            raise ValueError("Expected scene-specific divider missing: " + token)
        record = records[token]
        if record.get("road_divider_segments"):
            raise ValueError("Refusing to replace existing source paint annotations: " + token)
        nodes = lines[record["line_token"]]
        if len(nodes) < 2:
            raise ValueError("Expected divider geometry missing: " + token)
        record["road_divider_segments"] = [
            {"node_token": node, "segment_type": "DOUBLE_SOLID_YELLOW"}
            for node in nodes[:-1]]
    result["scene_visual_override"] = {
        "scene": manifest["scene"],
        "scope": "five explicitly listed untyped road-divider curves only",
        "divider_tokens": list(DIVIDERS),
        "assumed_style": "DOUBLE_SOLID_YELLOW",
        "style_verified_against_camera_imagery": False,
        "geometry_changed": False,
        "lane_divider_annotations_changed": False,
        "note": "Visual fallback only; not ground-truth paint or a driving-network change.",
    }
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    args = parser.parse_args()
    manifest_path = Path(args.manifest).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    source_path = Path(manifest["source"]["nuscenes_dataroot"]) / "maps/expansion/boston-seaport.json"
    raw = source_path.read_bytes()
    result = make_override(manifest, json.loads(raw.decode("utf-8")))
    result["scene_visual_override"]["original_source"] = str(source_path)
    result["scene_visual_override"]["original_source_sha256"] = hashlib.sha256(raw).hexdigest()
    output = manifest_path.parent / "visual_overrides" / "0757_assumed_divider_styles.json"
    if output.resolve() == source_path.resolve():
        raise ValueError("Refusing to overwrite the source map")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    audit = output.with_suffix(".audit.json")
    audit.write_text(json.dumps(result["scene_visual_override"], indent=2) + "\n", encoding="utf-8")
    print("Scene-specific override: " + str(output))
    print("Assumed double-yellow style on five curves; original map and geometry unchanged.")


if __name__ == "__main__":
    main()
