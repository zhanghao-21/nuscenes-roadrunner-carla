"""Opt-in interior lane paint for Nusc_boston_seaport_0103 only.

White dashes replace NIL on three surveyed same-direction lane dividers;
double yellow supplies an assumed style for twelve untyped opposing-traffic
dividers. This is a visual reconstruction, NOT ground-truth paint recovery.
No source coordinates, existing painted edges, or driving networks change.
"""

import argparse
import copy
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path


WHITE_DIVIDERS = (
    "03db3216-1485-4d17-ae32-a402b63ef913",
    "3a73e27f-67cc-4e9f-a160-34b7965e70aa",
    "834a0400-3456-41a2-8d83-7cd7af930346",
)
YELLOW_DIVIDERS = (
    "25f50c79-8c21-4f3f-b929-75a02a9cc7a2",
    "74e41be9-370a-40ce-a2ed-886a36d4b819",
    "836203e6-16bd-442f-bd9f-43b00d7d29e8",
    "92013ec7-a35a-4641-8ae4-a2f402cdaa78",
    "9222e62e-07fa-40ac-bccc-5040b9fa18b8",
    "994cb3df-b1ab-4538-a8bd-e848d6072118",
    "9ebe22f5-221a-466e-a92d-de67c1abc61d",
    "b234ec03-6b57-4791-b30f-3cf98fd5964a",
    "b8d8e0ff-f491-4795-88a2-6bc339a46cbf",
    "c0edf4df-21ff-4da9-883f-cbfeb1634488",
    "c9f85440-20fd-4b6a-af07-fe5d31e729af",
    "d34efd77-8f36-4a60-b480-fa40df21157b",
)


def _boundary_owners(source):
    polygons = {p["token"]: p for p in source["polygon"]}
    owners = defaultdict(set)
    # Lane connectors are deliberately excluded: never draw across junctions.
    for lane in source["lane"]:
        tokens = polygons[lane["polygon_token"]]["exterior_node_tokens"]
        for a, b in zip(tokens, tokens[1:] + tokens[:1]):
            owners[tuple(sorted((a, b)))].add(lane["token"])
    return owners


def _validate_boundary(tokens, owners, nodes, arcs, same_direction):
    pairs = set()
    for a, b in zip(tokens, tokens[1:]):
        pair = sorted(owners.get(tuple(sorted((a, b))), ()))
        if len(pair) != 2:
            raise ValueError("Divider must be shared by exactly two non-connector lanes")
        headings = []
        for token in pair:
            curve = arcs[token]
            headings.append((curve[0]["start_pose"][2], curve[-1]["end_pose"][2]))
        expected_sign = 1 if same_direction else -1
        if any(expected_sign * math.cos(h1 - h2) < 0.85
               for h1 in headings[0] for h2 in headings[1]):
            raise ValueError("Lane direction does not match the proposed divider color")
        dx, dy = nodes[b][0] - nodes[a][0], nodes[b][1] - nodes[a][1]
        tangent = math.atan2(dy, dx)
        if (math.hypot(dx, dy) < 0.01 or
                any(abs(math.cos(tangent - h)) < 0.8 for hs in headings for h in hs)):
            raise ValueError("Divider is not a longitudinal lane boundary")
        pairs.add(tuple(pair))
    return [list(pair) for pair in sorted(pairs)]


def make_override(manifest, source):
    if (manifest.get("scene") != "boston-seaport_scene-0103" or
            manifest.get("nuscenes_map") != "boston-seaport" or
            manifest.get("map", {}).get("asset_name") != "Nusc_boston_seaport_0103"):
        raise ValueError("This override is restricted to Nusc_boston_seaport_0103")
    result = copy.deepcopy(source)
    lines = {line["token"]: line["node_tokens"] for line in source["line"]}
    nodes = {node["token"]: (node["x"], node["y"]) for node in source["node"]}
    owners = _boundary_owners(source)
    changes = []
    for layer, selected, style in (
            ("lane_divider", WHITE_DIVIDERS, "SINGLE_DASHED_WHITE"),
            ("road_divider", YELLOW_DIVIDERS, "DOUBLE_SOLID_YELLOW")):
        records = {record["token"]: record for record in result[layer]}
        for token in selected:
            if token not in records:
                raise ValueError("Expected scene-specific divider missing: " + token)
            record = records[token]
            tokens = lines[record["line_token"]]
            if len(tokens) < 2 or len(set(tokens)) != len(tokens):
                raise ValueError("Invalid divider geometry: " + token)
            field = layer + "_segments"
            original = copy.deepcopy(record.get(field, []))
            if layer == "lane_divider":
                annotations = {a["node_token"]: a["segment_type"] for a in original}
                if (len(annotations) != len(original) or
                        any(n not in tokens or label != "NIL" for n, label in annotations.items()) or
                        any(annotations.get(n) != "NIL" for n in tokens[:-1])):
                    raise ValueError("Refusing to replace unexpected source lane paint: " + token)
            elif original:
                raise ValueError("Refusing to replace existing source road paint: " + token)
            pairs = _validate_boundary(tokens, owners, nodes, source["arcline_path_3"],
                                       same_direction=(layer == "lane_divider"))
            if layer == "lane_divider":
                # The last node has no outgoing edge; keep its NIL annotation.
                for annotation in record[field]:
                    if annotation["node_token"] in tokens[:-1]:
                        annotation["segment_type"] = style
            else:
                record[field] = [{"node_token": n, "segment_type": style} for n in tokens[:-1]]
            changes.append({"layer": layer, "token": token, "assumed_style": style,
                            "original_annotations": original, "lane_pairs": pairs,
                            "node_tokens": list(tokens)})
    result["scene_visual_override"] = {
        "scene": manifest["scene"], "scope": "15 explicitly listed interior dividers only",
        "style_verified_against_camera_imagery": False,
        "geometry_changed": False, "existing_painted_edges_changed": False,
        "nil_policy": "Only three listed same-direction boundaries gain assumed white dashes",
        "note": "Visual fallback only; not ground-truth paint or a driving-network change.",
        "changes": changes,
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
    audit_data = result["scene_visual_override"]
    audit_data["original_source"] = str(source_path)
    audit_data["original_source_sha256"] = hashlib.sha256(raw).hexdigest()
    output = manifest_path.parent / "visual_overrides" / "0103_assumed_divider_styles.json"
    if output.resolve() == source_path.resolve():
        raise ValueError("Refusing to overwrite the source map")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    output.with_suffix(".audit.json").write_text(json.dumps(audit_data, indent=2) + "\n", encoding="utf-8")
    print("Scene-specific override: " + str(output))
    print("Assumed paint: 3 white-dashed lane dividers and 12 double-yellow center dividers.")
    print("Original data, existing painted edges, and driving geometry unchanged.")


if __name__ == "__main__":
    main()
