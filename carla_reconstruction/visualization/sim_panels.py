"""Offline six-camera plus LiDAR figures for continuous TM sensor captures."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageOps

from .capture import write_json
from .panels import (_capture_file, _font, _sha256, _verify_image, _write_gif,
                     frame_durations, render_lidar)
from .scene_data import CAMERA_CHANNELS, CAMERA_GRID, carla_lidar_points


def validate_sim_capture(root):
    root = Path(root).resolve()
    capture = json.loads((root / "capture_index.json").read_text(encoding="utf-8"))
    if capture.get("schema_version") != 1 or capture.get("mode") != "traffic_manager_sensor_capture":
        raise ValueError("requires a traffic_manager_sensor_capture index")
    if not capture.get("complete") or not capture.get("frames"):
        raise ValueError("TM sensor capture is incomplete or empty")
    previous_frame, previous_time = -1, -1.0
    expected = set(CAMERA_CHANNELS) | {"LIDAR_TOP"}
    for index, row in enumerate(capture["frames"]):
        frame, elapsed = row["carla_frame"], row["relative_time_s"]
        if row["index"] != index or frame <= previous_frame or not math.isfinite(elapsed) or elapsed <= previous_time:
            raise ValueError("capture frame identities/times must be strictly increasing")
        ids = row.get("sensor_frame_ids", {})
        if set(ids) != expected or any(value != frame for value in ids.values()):
            raise ValueError("all six cameras and LiDAR must share the captured CARLA frame")
        stamps = row.get("sensor_timestamps_s", {})
        if set(stamps) != expected or any(
                not math.isfinite(value) or abs(value - row["world_timestamp_s"]) > 1e-4
                for value in stamps.values()):
            raise ValueError("sensor timestamps disagree with the captured world snapshot")
        for channel in CAMERA_CHANNELS:
            _verify_image(_capture_file(root, row["cameras"][channel]))
        raw = np.load(_capture_file(root, row["lidar_file"]), allow_pickle=False)
        if raw.ndim != 2 or raw.shape[1] != 4 or not len(raw) or not np.isfinite(raw).all():
            raise ValueError("LiDAR must contain finite nonempty Nx4 returns")
        carla_lidar_points(raw, row["lidar_sensor_world_matrix"], row["ego_carla_xyz"])
        previous_frame, previous_time = frame, elapsed
    return capture


def case_title(scenario):
    weather = scenario.get("weather", "default")
    label = {"rain": "HEAVY RAIN", "clear": "CLEAR DAY", "default": "LOADED WEATHER"}[weather]
    if scenario.get("obstacle"):
        return "LANE OBSTACLE | " + label
    return label


def render_panel(root, capture, row, cell_width=400, window_m=60.0):
    margin, gap, label_h, header, footer = 16, 8, 26, 70, 38
    cell_h = round(cell_width * 9 / 16)
    grid_w = 3 * cell_width + 2 * gap
    grid_h = 2 * (cell_h + label_h) + gap
    width = 2 * margin + grid_w + gap + grid_h
    image = Image.new("RGB", (width, header + grid_h + footer), (16, 25, 37))
    draw = ImageDraw.Draw(image)
    title = case_title(capture.get("scenario", {}))
    draw.text((margin, 10), "CARLA  |  " + title, fill=(240, 246, 252), font=_font(23))
    draw.text((margin, 41), capture["scene"] + "  |  Traffic Manager closed loop",
              fill=(170, 189, 206), font=_font(15))
    status = "t = %.2f s   |   ego %.1f km/h" % (row["relative_time_s"], row["ego_speed_kmh"])
    draw.text((width - margin - draw.textbbox((0, 0), status, font=_font(18))[2], 16),
              status, fill=(100, 213, 227), font=_font(18))
    for y, channels in enumerate(CAMERA_GRID):
        for x, channel in enumerate(channels):
            left, top = margin + x * (cell_width + gap), header + y * (cell_h + label_h + gap)
            draw.rectangle((left, top, left + cell_width - 1, top + label_h - 1), fill=(31, 47, 64))
            draw.text((left + 9, top + 5), "SIM  " + channel.replace("CAM_", "").replace("_", " "),
                      fill="white", font=_font(13))
            with Image.open(_capture_file(root, row["cameras"][channel])) as source:
                tile = ImageOps.contain(source.convert("RGB"), (cell_width, cell_h), Image.Resampling.LANCZOS)
            image.paste(tile, (left + (cell_width - tile.width) // 2, top + label_h + (cell_h - tile.height) // 2))
    raw = np.load(_capture_file(root, row["lidar_file"]), allow_pickle=False)
    points = carla_lidar_points(raw, row["lidar_sensor_world_matrix"], row["ego_carla_xyz"])
    lidar = render_lidar(points, grid_h, window_m, "SIM  LIDAR TOP")
    image.paste(lidar, (margin + grid_w + gap, header))
    note = "Six RGB cameras + one LiDAR  |  synchronized frame %d  |  approximate nuScenes sensor rig" % row["carla_frame"]
    draw.text((margin, header + grid_h + 11), note, fill=(170, 189, 206), font=_font(13))
    return image


def build_sim_panels(capture_dir, output_dir, cell_width=400, window_m=60.0, fps=None):
    if cell_width < 64 or not math.isfinite(window_m) or window_m <= 0:
        raise ValueError("require cell-width >=64 and finite positive window-m")
    root, output = Path(capture_dir).resolve(), Path(output_dir).resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError("panel output is not empty; choose a new output directory")
    capture = validate_sim_capture(root)
    durations = frame_durations(capture["frames"], fps, capture["frame_dt_s"])
    (output / "frames").mkdir(parents=True, exist_ok=True)
    paths = []
    for row in capture["frames"]:
        panel = render_panel(root, capture, row, cell_width, window_m)
        path = output / "frames" / ("%04d.png" % row["index"])
        panel.save(path)
        panel.close()
        paths.append(path)
    _write_gif(paths, output / "simulation.gif", durations)
    # A deterministic presentation still; every frame is also available separately.
    still_time = 2.5 if capture.get("scenario", {}).get("obstacle") else 4.0
    representative = min(range(len(paths)), key=lambda i: abs(capture["frames"][i]["relative_time_s"] - still_time))
    with Image.open(paths[representative]) as still:
        still.save(output / "figure.png")
    index = {
        "schema_version": 1, "layout": "sim_six_cameras_one_lidar", "mode": capture["mode"],
        "scene": capture["scene"], "scenario": capture.get("scenario", {}),
        "capture_index": str(root / "capture_index.json"),
        "capture_index_sha256": _sha256(root / "capture_index.json"),
        "gif_duration_ms": durations, "cell_width": cell_width, "window_m": window_m,
        "representative_index": representative,
        "frames": [{"index": row["index"], "carla_frame": row["carla_frame"],
                    "relative_time_s": row["relative_time_s"], "file": path.relative_to(output).as_posix()}
                   for row, path in zip(capture["frames"], paths)],
    }
    write_json(output / "panel_index.json", index)
    print("Simulation figure:", output / "figure.png")
    print("Simulation GIF:", output / "simulation.gif")
    return index


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cell-width", type=int, default=400)
    parser.add_argument("--window-m", type=float, default=60.0)
    parser.add_argument("--fps", type=float)
    args = parser.parse_args(argv)
    build_sim_panels(args.capture_dir, args.output, args.cell_width, args.window_m, args.fps)


if __name__ == "__main__":
    main()
