"""Build aligned camera/LiDAR/schematic panels from an indexed replay capture.

This module never connects to CARLA and never reads legacy GIFs as input. Each
panel is joined by its nuScenes sample identity, not by directory sort order.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import GifImagePlugin, Image, ImageDraw, ImageFont, ImageOps

from .scene_data import (CAMERA_CHANNELS, CAMERA_GRID, carla_lidar_points,
                         load_scene, read_manifest_selection, real_lidar_points)


GENERATED_ROOT = Path(__file__).resolve().parents[1] / "generated"
BACKGROUND = (238, 241, 244)
INK = (30, 40, 52)
GUTTER = 8


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _capture_file(root, name):
    """Resolve an indexed asset without allowing path traversal or symlink escape."""
    if not isinstance(name, str) or not name:
        raise ValueError("Capture asset must have a nonempty relative path")
    path = Path(name)
    if path.is_absolute():
        raise ValueError("Capture assets must use relative paths: " + name)
    resolved = (root / path).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as error:
        raise ValueError("Capture asset escapes capture directory: " + name) from error
    if not resolved.is_file():
        raise FileNotFoundError("Missing indexed capture asset: " + str(resolved))
    return resolved


def _verify_image(path):
    try:
        with Image.open(path) as image:
            image.verify()
        with Image.open(path) as image:
            image.load()
            if image.width < 1 or image.height < 1:
                raise ValueError("Image is empty")
    except Exception as error:
        raise ValueError("Unreadable camera image: " + str(path)) from error


def _meta_path(scene):
    if scene.get("meta_path"):
        return Path(scene["meta_path"])
    path = Path(scene["manifest"]["trajectories"]["meta"])
    return path if path.is_absolute() else Path(scene["manifest_path"]).parent / path


def validate_capture(scene, capture_dir, full=False, allow_partial=False):
    """Validate all identities and source assets before creating output files."""
    root = Path(capture_dir).resolve()
    index_path = root / "capture_index.json"
    if not index_path.is_file():
        raise FileNotFoundError("Capture index not found: " + str(index_path))
    with index_path.open(encoding="utf-8") as stream:
        capture = json.load(stream)
    if capture.get("schema_version") != 1:
        raise ValueError("Unsupported capture schema_version")
    if capture.get("scene") != scene["scene"]:
        raise ValueError("Capture scene does not match the selected manifest")
    if capture.get("mode") != "recorded_reconstruction_replay":
        raise ValueError("This builder requires a recorded_reconstruction_replay capture")
    for key, path in (("manifest_sha256", scene["manifest_path"]),
                      ("meta_sha256", _meta_path(scene))):
        if capture.get(key) != _sha256(path):
            raise ValueError("Capture " + key + " does not match current source; recapture")
    if not capture.get("complete") and not allow_partial:
        raise ValueError("Capture is incomplete; recapture or explicitly use --allow-partial")
    source_frames = {frame["index"]: frame for frame in scene["frames"]}
    rows = capture.get("frames", [])
    if not rows:
        raise ValueError("Capture contains no frames")
    seen = set()
    previous_index = -1
    previous_time = None
    checked = []
    for row in rows:
        index = row.get("index")
        if not isinstance(index, int) or index in seen or index <= previous_index:
            raise ValueError("Capture frame indexes must be unique and strictly increasing")
        source = source_frames.get(index)
        if source is None:
            raise ValueError("Capture frame index is absent from source: " + str(index))
        for key in ("sample_token", "timestamp_us"):
            if row.get(key) != source[key]:
                raise ValueError("Frame %s %s mismatch; never align by file order" % (index, key))
        if not math.isclose(float(row.get("relative_time_s", math.nan)),
                            float(source["relative_time_s"]), abs_tol=1e-6):
            raise ValueError("Frame %s relative_time_s mismatch" % index)
        time_us = row["timestamp_us"]
        if previous_time is not None and time_us <= previous_time:
            raise ValueError("Capture timestamps must be strictly increasing")
        seen.add(index)
        previous_index, previous_time = index, time_us
        cameras = row.get("cameras", {})
        for channel in CAMERA_CHANNELS:
            if channel not in cameras or channel not in source.get("cameras", {}):
                raise ValueError("Frame %s missing camera %s" % (index, channel))
            _verify_image(_capture_file(root, cameras[channel]))
            _verify_image(Path(source["cameras"][channel]))
        ego = np.asarray(row.get("ego_carla_xyz", []), dtype=float)
        if ego.shape != (3,) or not np.isfinite(ego).all():
            raise ValueError("Invalid ego_carla_xyz at frame " + str(index))
        for actor in row.get("actors", []):
            for key in ("x", "y", "yaw_rad", "length", "width"):
                if key not in actor or not math.isfinite(float(actor[key])):
                    raise ValueError("Invalid captured actor " + key)
            if actor["length"] <= 0 or actor["width"] <= 0:
                raise ValueError("Captured actor dimensions must be positive")
        if full:
            raw = np.load(_capture_file(root, row.get("lidar_file")), allow_pickle=False)
            if raw.ndim != 2 or raw.shape[1] != 4 or not np.isfinite(raw).all():
                raise ValueError("SIM LiDAR must be a finite Nx4 array")
            carla_lidar_points(raw, row.get("lidar_sensor_world_matrix"), ego)
            real_lidar_points(source)
        checked.append((source, row))
    parameters = capture.get("capture_parameters", {})
    expected = parameters.get("requested_indices", list(source_frames))
    if (not isinstance(expected, list) or not expected or
            any(not isinstance(index, int) or index not in source_frames for index in expected) or
            sorted(set(expected)) != expected):
        raise ValueError("capture_parameters.requested_indices must be unique ordered source indexes")
    if not seen.issubset(set(expected)):
        raise ValueError("Capture contains an index outside its requested_indices")
    if not allow_partial and seen != set(expected):
        raise ValueError("Complete capture is missing source frame indexes; use --allow-partial explicitly")
    geometry_path = _capture_file(root, capture.get("map_geometry_file"))
    with geometry_path.open(encoding="utf-8") as stream:
        geometry = json.load(stream)
    if geometry.get("coordinate_frame") != "opendrive_patch_local_metres":
        raise ValueError("Map geometry has an unsupported coordinate frame")
    if not isinstance(geometry.get("roads"), list) or not geometry["roads"]:
        raise ValueError("Map geometry contains no CARLA lane polygons")
    for road in geometry["roads"]:
        polygon = np.asarray(road.get("polygon", []), dtype=float)
        if polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2 or not np.isfinite(polygon).all():
            raise ValueError("Invalid CARLA lane polygon")
    environment = geometry.get("environment", {})
    if not isinstance(environment, dict):
        raise ValueError("Invalid map environment")
    for building in environment.get("buildings", []):
        for key in ("x", "y", "half_x", "half_y"):
            if not math.isfinite(float(building[key])):
                raise ValueError("Invalid building " + key)
        if (building["half_x"] <= 0 or building["half_y"] <= 0 or
                not math.isfinite(float(building.get("yaw_rad", 0)))):
            raise ValueError("Invalid building size or yaw")
    for tree in environment.get("trees", []):
        radius = float(tree.get("radius", tree.get("crown_radius", 1.5)))
        if not all(math.isfinite(value) for value in (float(tree["x"]), float(tree["y"]), radius)) or radius <= 0:
            raise ValueError("Invalid tree position or radius")
    for key in ("nuscenes_static_objects", "traffic_signs", "traffic_lights"):
        for prop in environment.get(key, []):
            if not all(math.isfinite(float(prop[field])) for field in ("x", "y")):
                raise ValueError("Invalid static prop position")
            if key == "nuscenes_static_objects":
                if (not all(math.isfinite(float(prop[field])) and prop[field] > 0 for field in ("length", "width")) or
                        not math.isfinite(float(prop.get("yaw_rad", 0)))):
                    raise ValueError("Invalid static prop dimensions or yaw")
    return capture, geometry, checked


def _font(size):
    for name in ("arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def _tag(image, label, subtitle=None):
    draw = ImageDraw.Draw(image)
    size = max(12, min(22, image.width // 24))
    height = size + 12 + (size + 2 if subtitle else 0)
    draw.rectangle((0, 0, image.width, height), fill=INK)
    draw.text((8, 5), label, font=_font(size), fill=(255, 255, 255))
    if subtitle:
        draw.text((8, size + 8), subtitle, font=_font(max(10, size - 3)), fill=(201, 213, 225))


def _camera_grid(paths, cell_width, label):
    cell_height = round(cell_width * 9 / 16)
    grid = Image.new("RGB", (3 * cell_width, 2 * cell_height), INK)
    for y, row in enumerate(CAMERA_GRID):
        for x, channel in enumerate(row):
            with Image.open(paths[channel]) as source:
                tile = ImageOps.contain(source.convert("RGB"), (cell_width, cell_height), Image.Resampling.LANCZOS)
            grid.paste(tile, (x * cell_width + (cell_width - tile.width) // 2,
                              y * cell_height + (cell_height - tile.height) // 2))
            draw = ImageDraw.Draw(grid)
            short = channel.replace("CAM_", "")
            draw.text((x * cell_width + 6, y * cell_height + cell_height - 20), short,
                      font=_font(max(10, min(14, cell_width // 20))), fill="white",
                      stroke_width=1, stroke_fill="black")
    _tag(grid, label)
    return grid


def _project(point, center, size, window_m):
    scale = size / window_m
    return ((float(point[0]) - center[0]) * scale + size / 2,
            size / 2 - (float(point[1]) - center[1]) * scale)


def _box(x, y, length, width, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * dx - s * dy, y + s * dx + c * dy)
            for dx, dy in ((length / 2, width / 2), (length / 2, -width / 2),
                           (-length / 2, -width / 2), (-length / 2, width / 2))]


def _actor_color(actor):
    category = actor.get("category", "")
    if actor.get("is_ego"):
        return (224, 65, 62)
    if "pedestrian" in category:
        return (54, 159, 98)
    if "bicycle" in category or "motorcycle" in category:
        return (181, 77, 168)
    return (37, 126, 200)


def render_topdown(geometry, frame, size, window_m=80.0, scene_name=""):
    """Schematic lane area and actors; not a representation of painted markings."""
    image = Image.new("RGB", (size, size), (229, 234, 224))
    draw = ImageDraw.Draw(image)
    center = (frame["ego_carla_xyz"][0], -frame["ego_carla_xyz"][1])
    project = lambda point: _project(point, center, size, window_m)
    for road in geometry["roads"]:
        draw.polygon([project(point) for point in road["polygon"]], fill=(150, 155, 160))
    environment = geometry.get("environment", {})
    for building in environment.get("buildings", []):
        corners = _box(building["x"], building["y"], 2 * building["half_x"],
                       2 * building["half_y"], building.get("yaw_rad", 0))
        draw.polygon([project(point) for point in corners], fill=(199, 190, 175), outline=(143, 132, 119))
    for tree in environment.get("trees", []):
        px, py = project((tree["x"], tree["y"]))
        radius = max(2.0, float(tree.get("radius", tree.get("crown_radius", 1.5))) * size / window_m)
        draw.ellipse((px - radius, py - radius, px + radius, py + radius), fill=(107, 149, 100))
    for prop in environment.get("nuscenes_static_objects", []):
        px, py = project((prop["x"], prop["y"]))
        if "trafficcone" in prop.get("category", ""):
            radius = max(2, prop["width"] * size / window_m / 2)
            draw.polygon(((px, py - radius), (px - radius, py + radius), (px + radius, py + radius)),
                         fill=(230, 145, 45), outline=(140, 81, 21))
        else:
            corners = _box(prop["x"], prop["y"], prop["length"], prop["width"], prop.get("yaw_rad", 0))
            draw.polygon([project(point) for point in corners], fill=(230, 145, 45), outline=(140, 81, 21))
    # Signal markers represent locations only, never an inferred red/green state.
    for key in ("traffic_signs", "traffic_lights"):
        for prop in environment.get(key, []):
            px, py = project((prop["x"], prop["y"]))
            radius = max(2, size / 200)
            draw.ellipse((px - radius, py - radius, px + radius, py + radius),
                         fill=(131, 93, 171), outline=(69, 43, 96))
    for actor in frame.get("actors", []):
        corners = _box(actor["x"], actor["y"], actor["length"], actor["width"], actor["yaw_rad"])
        draw.polygon([project(point) for point in corners], fill=_actor_color(actor), outline=(30, 45, 55))
        nose = (actor["x"] + actor["length"] / 2 * math.cos(actor["yaw_rad"]),
                actor["y"] + actor["length"] / 2 * math.sin(actor["yaw_rad"]))
        draw.line((project((actor["x"], actor["y"])), project(nose)), fill="white", width=2)
    _tag(image, "TOP-DOWN (SCHEMATIC)", "Ego-centred | north-up | lane areas, not painted markings")
    legend = "Ego: red | Vehicle: blue | Pedestrian: green | Props: orange | Signs/lights: purple"
    draw.rectangle((0, size - 54, size, size), fill=INK)
    draw.text((8, size - 47), legend, fill="white", font=_font(max(10, min(13, size // 50))))
    identity = "%s | frame %s | t=%.2f s" % (scene_name, frame["index"], frame["relative_time_s"])
    draw.text((8, size - 24), identity, fill=(204, 216, 228), font=_font(max(10, min(14, size // 45))))
    _scale(draw, size, window_m, dark=False)
    return image


def _scale(draw, size, window_m, dark):
    color = (230, 238, 245) if dark else INK
    meters = 10 if window_m >= 30 else 5 if window_m >= 15 else 1
    length = meters * size / window_m
    bottom = size - (42 if dark else 67)
    draw.line((14, bottom, 14 + length, bottom), fill=color, width=3)
    draw.text((14, bottom - 18), "%g m" % meters, fill=color, font=_font(12))
    x, y = size - 28, max(70, size // 7)
    draw.line((x, y + 23, x, y), fill=color, width=2)
    draw.polygon(((x, y), (x - 4, y + 7), (x + 4, y + 7)), fill=color)
    draw.text((x - 4, y - 17), "N", fill=color, font=_font(12))


def render_lidar(points, size, window_m=80.0, label="LIDAR"):
    """Render ego-centred north-up point cloud with a fixed -2..8 m height scale."""
    cloud = np.asarray(points, dtype=float)
    if cloud.ndim != 2 or cloud.shape[1] < 3:
        raise ValueError("LiDAR points must be Nx3")
    pixels = np.empty((size, size, 3), dtype=np.uint8)
    pixels[:] = (12, 23, 38)
    keep = np.isfinite(cloud[:, :3]).all(axis=1)
    keep &= (np.abs(cloud[:, 0]) < window_m / 2) & (np.abs(cloud[:, 1]) < window_m / 2)
    points = cloud[keep]
    if len(points):
        x = np.clip((size / 2 + points[:, 0] * size / window_m).astype(int), 0, size - 1)
        y = np.clip((size / 2 - points[:, 1] * size / window_m).astype(int), 0, size - 1)
        height = np.clip((points[:, 2] + 2) / 10, 0, 1)
        colors = np.column_stack((60 + 195 * height, 175 + 55 * (1 - height), 245 - 185 * height)).astype(np.uint8)
        order = np.argsort(points[:, 2])
        pixels[y[order], x[order]] = colors[order]
    image = Image.fromarray(pixels)
    _tag(image, label, "North-up | height -2 to +8 m relative to ego")
    draw = ImageDraw.Draw(image)
    center = size // 2
    draw.line((center - 4, center, center + 4, center), fill="white")
    draw.line((center, center - 4, center, center + 4), fill="white")
    _scale(draw, size, window_m, dark=True)
    return image


def frame_durations(frames, fps=None, fallback_s=0.5):
    if fps is not None:
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be positive")
        return [max(10, round(100 / fps) * 10)] * len(frames)
    differences = [(second["timestamp_us"] - first["timestamp_us"]) / 1000
                   for first, second in zip(frames, frames[1:])]
    last = differences[-1] if differences else fallback_s * 1000
    # GIF stores delays in centiseconds; original timestamps remain in the index.
    return [max(10, round(value / 10) * 10) for value in differences + [last]]


def _write_gif(paths, output, durations):
    """Stream one quantized frame at a time instead of retaining all RGB panels."""
    palette = Image.new("P", (1, 1))
    colors = [value for r in range(6) for g in range(6) for b in range(6)
              for value in (r * 51, g * 51, b * 51)]
    colors.extend(value for step in range(40) for value in [round(step * 255 / 39)] * 3)
    palette.putpalette(colors)
    with Path(output).open("wb") as stream:
        for index, (path, duration) in enumerate(zip(paths, durations)):
            with Image.open(path) as source:
                frame = source.convert("RGB").quantize(palette=palette, dither=Image.Dither.FLOYDSTEINBERG)
            if index == 0:
                chunks, _ = GifImagePlugin.getheader(frame, info={"loop": 0})
                for chunk in chunks:
                    stream.write(chunk)
            for chunk in GifImagePlugin.getdata(frame, duration=duration, disposal=2):
                stream.write(chunk)
            frame.close()
        stream.write(b";")


def build_scene(scene, capture_dir, output_dir, full=False, cell_width=320,
                window_m=80.0, fps=None, allow_partial=False):
    if cell_width < 64:
        raise ValueError("cell-width must be at least 64 pixels")
    if not math.isfinite(window_m) or window_m <= 0:
        raise ValueError("window-m must be positive")
    root, output = Path(capture_dir).resolve(), Path(output_dir).resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError("Output is not empty; choose a new --output directory: " + str(output))
    capture, geometry, aligned = validate_capture(scene, root, full, allow_partial)
    durations = frame_durations([row for _, row in aligned], fps, capture.get("frame_dt_s", 0.5))
    (output / "frames").mkdir(parents=True, exist_ok=True)
    paths, metadata = [], []
    for source, row in aligned:
        real_grid = _camera_grid(source["cameras"], cell_width, "REAL | nuScenes")
        sim_paths = {channel: _capture_file(root, row["cameras"][channel]) for channel in CAMERA_CHANNELS}
        sim_grid = _camera_grid(sim_paths, cell_width, "SIM | CARLA recorded-pose replay")
        height = real_grid.height + sim_grid.height + GUTTER
        bev = render_topdown(geometry, row, height, window_m, scene["scene"])
        middle_width = real_grid.height if full else 0
        width = real_grid.width + GUTTER + height + (middle_width + GUTTER if full else 0)
        panel = Image.new("RGB", (width, height), BACKGROUND)
        panel.paste(real_grid, (0, 0))
        panel.paste(sim_grid, (0, real_grid.height + GUTTER))
        x = real_grid.width + GUTTER
        if full:
            raw = np.load(_capture_file(root, row["lidar_file"]), allow_pickle=False)
            sim_points = carla_lidar_points(raw, row["lidar_sensor_world_matrix"], row["ego_carla_xyz"])
            panel.paste(render_lidar(real_lidar_points(source), middle_width, window_m, "REAL LIDAR"), (x, 0))
            panel.paste(render_lidar(sim_points, middle_width, window_m, "SIM LIDAR"), (x, middle_width + GUTTER))
            x += middle_width + GUTTER
        panel.paste(bev, (x, 0))
        path = output / "frames" / ("%03d.png" % row["index"])
        panel.save(path)
        paths.append(path)
        metadata.append({key: row[key] for key in ("index", "sample_token", "timestamp_us", "relative_time_s")})
        metadata[-1]["file"] = path.relative_to(output).as_posix()
        panel.close()
    _write_gif(paths, output / "aligned.gif", durations)
    panel_index = {"schema_version": 1, "scene": scene["scene"], "layout": "aligned_full" if full else "aligned_topdown",
                   "capture_index": str(root / "capture_index.json"), "capture_index_sha256": _sha256(root / "capture_index.json"),
                   "manifest_sha256": capture["manifest_sha256"], "meta_sha256": capture["meta_sha256"],
                   "camera_rig": capture.get("camera_rig"), "mode": capture["mode"],
                   "topdown": "schematic CARLA lane polygons, manifest environment and captured actor poses; not painted markings",
                   "window_m": window_m, "cell_width": cell_width, "fps_override": fps,
                   "gif_duration_ms": durations,
                   "partial_capture": (not capture.get("complete") or
                                       len(aligned) != len(capture.get("capture_parameters", {}).get("requested_indices", scene["frames"]))),
                   "allow_partial": allow_partial,
                   "frames": metadata}
    with (output / "panel_index.json").open("w", encoding="utf-8") as stream:
        json.dump(panel_index, stream, indent=2)
    return panel_index


def main(full=False, argv=None):
    parser = argparse.ArgumentParser(description="Build aligned REAL/SIM camera%s and schematic top-down panels, without CARLA." % ("/LiDAR" if full else ""))
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--manifest", help="Path to a reconstructed scene_manifest.json")
    selection.add_argument("--scene", help="Generated scene name, or all")
    parser.add_argument("--generated-root", type=Path, default=GENERATED_ROOT)
    parser.add_argument("--capture-dir", type=Path, help="Override indexed capture directory (one scene only)")
    parser.add_argument("--output", type=Path, help="New/empty output directory (one scene only)")
    parser.add_argument("--dataroot", help="Override nuScenes dataset root")
    parser.add_argument("--version", help="Override nuScenes dataset version")
    parser.add_argument("--cell-width", type=int, default=320)
    parser.add_argument("--window-m", type=float, default=80.0)
    parser.add_argument("--fps", type=float, help="Override playback rate; default follows source timestamps")
    parser.add_argument("--allow-partial", action="store_true", help="Allow explicitly incomplete/missing keyframes; identities still must match")
    args = parser.parse_args(argv)
    if args.scene and args.scene.lower() == "all" and (args.capture_dir or args.output):
        parser.error("--capture-dir and --output require a single scene")
    try:
        manifests = read_manifest_selection(manifest=args.manifest, scene=args.scene, generated_root=args.generated_root)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    failures = []
    for manifest in manifests:
        try:
            scene = load_scene(manifest, dataroot=args.dataroot, version=args.version)
            base = args.generated_root / "visualizations" / scene["scene"]
            capture_dir = args.capture_dir or base / "capture"
            output_dir = args.output or base / ("aligned_full" if full else "aligned_topdown")
            result = build_scene(scene, capture_dir, output_dir, full, args.cell_width,
                                 args.window_m, args.fps, args.allow_partial)
            print("[%s] %s frames -> %s" % (scene["scene"], len(result["frames"]), output_dir))
        except (ValueError, OSError, KeyError) as error:
            failures.append(str(manifest))
            print("ERROR [%s]: %s" % (manifest, error))
    if failures:
        raise SystemExit("Panel build failed for %d scene(s); no frames were silently dropped." % len(failures))


if __name__ == "__main__":
    main()
