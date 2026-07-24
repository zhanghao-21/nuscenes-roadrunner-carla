"""Build the 07/24 progress deck (v2, 20 slides):
nuScenes -> OpenDRIVE -> CARLA / RoadRunner."""
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

import os
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = BASE + r"\progress_report\project_nuscenes_072026.pptx"
IMG = BASE + r"\nuscenes2xodr\output"

NAVY = RGBColor(0x1F, 0x38, 0x64)
NAVY2 = RGBColor(0x2E, 0x4E, 0x87)
ACCENT = RGBColor(0xC0, 0x50, 0x4D)
DARK = RGBColor(0x33, 0x33, 0x33)
GREY = RGBColor(0x59, 0x59, 0x59)
LIGHT = RGBColor(0xF2, 0xF2, 0xF2)
BLUEBG = RGBColor(0xE8, 0xED, 0xF5)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

prs = Presentation()
prs.slide_width = Emu(12192000)
prs.slide_height = Emu(6858000)
BLANK = prs.slide_layouts[6]

page = [0]
FOOTER = "nuScenes → OpenDRIVE → CARLA / RoadRunner   ·   progress 07/24/2026"


def _noline(shp):
    shp.line.fill.background()
    shp.shadow.inherit = False


def add_slide(title, stage=None):
    s = prs.slides.add_slide(BLANK)
    page[0] += 1
    tb = s.shapes.add_textbox(Inches(0.45), Inches(0.2), Inches(10.2), Inches(0.72))
    p = tb.text_frame.paragraphs[0]
    r = p.add_run(); r.text = title
    r.font.size = Pt(29); r.font.bold = True; r.font.color.rgb = NAVY
    bar = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.5), Inches(0.96),
                             Inches(2.2), Pt(3))
    bar.fill.solid(); bar.fill.fore_color.rgb = ACCENT; _noline(bar)
    if stage:
        chip = s.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(11.0),
                                  Inches(0.3), Inches(1.85), Inches(0.42))
        chip.adjustments[0] = 0.5
        chip.fill.solid(); chip.fill.fore_color.rgb = NAVY; _noline(chip)
        tfc = chip.text_frame; tfc.word_wrap = False
        pc = tfc.paragraphs[0]; pc.alignment = PP_ALIGN.CENTER
        rc = pc.add_run(); rc.text = stage
        rc.font.size = Pt(12); rc.font.bold = True; rc.font.color.rgb = WHITE
    ft = s.shapes.add_textbox(Inches(0.45), Inches(7.08), Inches(9.0), Inches(0.35))
    q = ft.text_frame.paragraphs[0]
    r = q.add_run(); r.text = FOOTER
    r.font.size = Pt(10); r.font.color.rgb = RGBColor(0xA6, 0xA6, 0xA6)
    pn = s.shapes.add_textbox(Inches(12.5), Inches(7.02), Inches(0.6), Inches(0.4))
    q = pn.text_frame.paragraphs[0]; q.alignment = PP_ALIGN.RIGHT
    r = q.add_run(); r.text = str(page[0])
    r.font.size = Pt(12); r.font.color.rgb = GREY
    return s


def textbox(slide, x, y, w, h):
    tb = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = tb.text_frame; tf.word_wrap = True
    return tf


def para(tf, text, size=14, bold=False, color=DARK, first=False, space=4,
         caps=False, align=None, italic=False):
    p = tf.paragraphs[0] if first and not tf.paragraphs[0].runs else tf.add_paragraph()
    p.space_after = Pt(space)
    if align is not None:
        p.alignment = align
    r = p.add_run(); r.text = text.upper() if caps else text
    f = r.font; f.size = Pt(size); f.bold = bold; f.color.rgb = color
    f.italic = italic
    return p


def bullet(tf, text, size=13.5, level=0, bold=False, color=DARK, first=False,
           space=5):
    prefix = "• " if level == 0 else "– "
    return para(tf, prefix + text, size=size, bold=bold, color=color,
                first=first, space=space)


def section_label(tf, text, first=False):
    return para(tf, text, size=13.5, bold=True, color=ACCENT, caps=True,
                first=first, space=6)


def card(slide, x, y, w, h, fill=LIGHT, rounded=True):
    shp = slide.shapes.add_shape(
        MSO_SHAPE.ROUNDED_RECTANGLE if rounded else MSO_SHAPE.RECTANGLE,
        Inches(x), Inches(y), Inches(w), Inches(h))
    if rounded:
        shp.adjustments[0] = 0.055
    shp.fill.solid(); shp.fill.fore_color.rgb = fill; _noline(shp)
    tf = shp.text_frame; tf.word_wrap = True
    tf.margin_left = Inches(0.15); tf.margin_right = Inches(0.15)
    tf.margin_top = Inches(0.1); tf.margin_bottom = Inches(0.1)
    tf.vertical_anchor = MSO_ANCHOR.TOP
    return tf


def arrow(slide, x, y, ln=0.4, th=0.3, down=False, color=ACCENT):
    shp = slide.shapes.add_shape(
        MSO_SHAPE.DOWN_ARROW if down else MSO_SHAPE.RIGHT_ARROW,
        Inches(x), Inches(y), Inches(th if down else ln),
        Inches(ln if down else th))
    shp.fill.solid(); shp.fill.fore_color.rgb = color; _noline(shp)
    return shp


def hflow(slide, items, x, y, h, box_w, gap=0.5, tsize=13, dsize=11,
          fill=LIGHT, tcolor=NAVY, rev=False):
    """Horizontal flowchart: boxes with arrows between (rev: point left)."""
    for i, (t, d) in enumerate(items):
        bx = x + i * (box_w + gap)
        tf = card(slide, bx, y, box_w, h, fill=fill)
        para(tf, t, size=tsize, bold=True, color=tcolor, first=True, space=3,
             align=PP_ALIGN.CENTER)
        if d:
            para(tf, d, size=dsize, color=DARK, align=PP_ALIGN.CENTER)
        if i < len(items) - 1:
            ax = bx + box_w + (gap - 0.36) / 2
            shp = slide.shapes.add_shape(
                MSO_SHAPE.LEFT_ARROW if rev else MSO_SHAPE.RIGHT_ARROW,
                Inches(ax), Inches(y + h / 2 - 0.13), Inches(0.36),
                Inches(0.26))
            shp.fill.solid(); shp.fill.fore_color.rgb = ACCENT; _noline(shp)


def picture(slide, path, x, y, w=None, h=None, caption=None, cap_w=None):
    pic = slide.shapes.add_picture(path, Inches(x), Inches(y),
                                   Inches(w) if w else None,
                                   Inches(h) if h else None)
    pic.line.color.rgb = RGBColor(0xD9, 0xD9, 0xD9)
    pic.line.width = Pt(0.75)
    if caption:
        cy = (pic.top + pic.height) / 914400 + 0.04
        tf = textbox(slide, x, cy, cap_w or (pic.width / 914400), 0.3)
        para(tf, caption, size=10.5, color=GREY, first=True,
             align=PP_ALIGN.CENTER)
    return pic


def stat_row(tf, v, k, vsize=22, ksize=13):
    p = tf.add_paragraph(); p.space_after = Pt(5)
    r = p.add_run(); r.text = v + "  "
    r.font.size = Pt(vsize); r.font.bold = True; r.font.color.rgb = NAVY
    r2 = p.add_run(); r2.text = k
    r2.font.size = Pt(ksize); r2.font.color.rgb = GREY


# ================================================================ 1 · TITLE
s = prs.slides.add_slide(BLANK)
page[0] += 1
bg = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, prs.slide_width,
                        prs.slide_height)
bg.fill.solid(); bg.fill.fore_color.rgb = NAVY; _noline(bg)
band = s.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, Inches(4.62),
                          prs.slide_width, Pt(3))
band.fill.solid(); band.fill.fore_color.rgb = ACCENT; _noline(band)
tf = textbox(s, 0.9, 1.55, 11.5, 2.4)
para(tf, "Rebuilding Real Driving Scenes in Simulation", size=40, bold=True,
     color=WHITE, first=True, space=10)
para(tf, "nuScenes  →  OpenDRIVE  →  CARLA 0.9.15  /  RoadRunner R2026a",
     size=22, color=RGBColor(0xC9, 0xD6, 0xEA), space=8)
tf = textbox(s, 0.9, 4.85, 11.5, 1.6)
para(tf, "Progress Update — July 24, 2026", size=18, bold=True,
     color=WHITE, first=True, space=6)
para(tf, "All 10 nuScenes-mini scenes: georeferenced road networks, "
         "calibrated sensor replay, and collision-safe scenario "
         "reconstruction in two simulators", size=15,
     color=RGBColor(0xC9, 0xD6, 0xEA))

# ================================================================ 2 · OVERVIEW
s = add_slide("Task Overview")
tf = textbox(s, 0.5, 1.25, 6.0, 5.4)
section_label(tf, "Objective", first=True)
para(tf, "Rebuild all 10 nuScenes-mini scenes as simulation-ready OpenDRIVE "
         "road networks and replay the recorded traffic — ego + 703 annotated "
         "agents — in CARLA and MathWorks RoadRunner, for repeatable, "
         "closed-loop scenario evaluation.", size=15, space=10)
section_label(tf, "Design principle")
para(tf, "One simulator-neutral intermediate representation feeds both "
         "back-ends: an OpenDRIVE 1.4 map plus a JSON sidecar carrying "
         "everything OpenDRIVE cannot (ego trajectory, 2 Hz agent frames, "
         "origin, buildings, crosswalks).", size=14, space=10)
section_label(tf, "This round vs. May (ScenarioNet replay)")
bullet(tf, "From log replay to true road networks generated from the "
           "nuScenes HD vector map, georeferenced to WGS84")
bullet(tf, "Two simulator back-ends instead of one")
bullet(tf, "Systematic real-vs-sim validation: cameras, LiDAR, BEV, OSM")

tf = card(s, 6.9, 1.3, 5.9, 5.35, fill=BLUEBG)
para(tf, "STATUS: ALL 10 SCENES COMPLETE, END TO END", size=14, bold=True,
     color=ACCENT, first=True, space=10)
for t in ["OpenDRIVE map + metadata sidecar (2 flavors)",
          "CARLA replay with calibrated 6-camera rig + LiDAR",
          "5,000+ rendered frames · 60 real-vs-sim GIFs",
          "RoadRunner city scene (markings, signals, buildings)",
          "RoadRunner replay scenario, collision-safe"]:
    p = tf.add_paragraph(); p.space_after = Pt(7)
    r = p.add_run(); r.text = "✓  "
    r.font.size = Pt(15); r.font.bold = True; r.font.color.rgb = ACCENT
    r2 = p.add_run(); r2.text = t
    r2.font.size = Pt(14); r2.font.color.rgb = DARK
para(tf, "", size=4)
para(tf, "Boston Seaport · One North · Queenstown · Holland Village", size=12.5,
     color=GREY, italic=True)

# ================================================================ 3 · ROADMAP
s = add_slide("Roadmap — Four Work Packages")
steps = [
    ("1 · Convert", "Scene region → OpenDRIVE 1.4 + metadata sidecar; real "
     "markings, junctions, traffic lights with real OSM counts"),
    ("2 · Replay in CARLA", "Four driving modes; deterministic synchronous "
     "playback; calibrated nuScenes sensor rig"),
    ("3 · Validate", "Real-vs-sim cameras, LiDAR, BEV and OSM overlays — "
     "per keyframe, per scene"),
    ("4 · Rebuild in RoadRunner", "City scenes from five data layers + "
     "collision-safe scenario replay of every agent"),
]
for i, (t, d) in enumerate(steps):
    x = 0.5 + i * 3.12
    pent = s.shapes.add_shape(
        MSO_SHAPE.PENTAGON if i == 0 else MSO_SHAPE.CHEVRON,
        Inches(x), Inches(1.5), Inches(3.35), Inches(0.85))
    pent.adjustments[0] = 0.32
    pent.fill.solid()
    pent.fill.fore_color.rgb = NAVY if i % 2 == 0 else NAVY2
    _noline(pent)
    ptf = pent.text_frame; ptf.word_wrap = False
    pp = ptf.paragraphs[0]; pp.alignment = PP_ALIGN.CENTER
    rr = pp.add_run(); rr.text = t
    rr.font.size = Pt(15); rr.font.bold = True; rr.font.color.rgb = WHITE
    tf = textbox(s, x + 0.15, 2.55, 2.95, 1.9)
    para(tf, d, size=12.5, color=DARK, first=True)
tf = card(s, 0.5, 4.75, 12.3, 1.9, fill=LIGHT)
para(tf, "MILESTONES", size=13, bold=True, color=ACCENT, first=True, space=6)
para(tf, "Jun 30 – Jul 4:  converter, CARLA loader, validation notebooks, all "
         "CARLA renders      ·      Jul 20:  RoadRunner import + OSM "
         "co-registration      ·      Jul 21:  all 10 city scenes      ·      "
         "Jul 21 – 22:  all 10 replay scenarios", size=13.5)

# ================================================================ 4 · ARCHITECTURE
s = add_slide("System Architecture")
from_boxes = [
    ("nuScenes v1.0-mini", "10 scenes · 4 maps\n404 keyframes @ 2 Hz\nHD "
     "vector map, ego poses, 3D boxes"),
]
tf = card(s, 0.45, 1.7, 2.5, 2.15, fill=BLUEBG)
para(tf, "nuScenes v1.0-mini", size=13.5, bold=True, color=NAVY, first=True,
     space=3, align=PP_ALIGN.CENTER)
para(tf, "10 scenes · 4 maps · 404 keyframes · HD vector map, ego poses, "
         "3D boxes", size=11, color=DARK, align=PP_ALIGN.CENTER)
tf = card(s, 3.55, 1.7, 3.0, 2.15)
para(tf, "Converter", size=13.5, bold=True, color=NAVY, first=True, space=1,
     align=PP_ALIGN.CENTER)
para(tf, "convert_nuscenes_to_xodr.py · Py 3.12", size=9.5, color=ACCENT,
     space=4, align=PP_ALIGN.CENTER)
para(tf, "OpenDRIVE 1.4  +  _meta.json sidecar (ego trajectory, per-frame "
         "agents, buildings, crosswalks)", size=11, color=DARK,
     align=PP_ALIGN.CENTER)
tf = card(s, 7.15, 1.7, 3.0, 2.15)
para(tf, "CARLA 0.9.15", size=13.5, bold=True, color=NAVY, first=True, space=1,
     align=PP_ALIGN.CENTER)
para(tf, "load_xodr_in_carla.py · Py 3.7", size=9.5, color=ACCENT, space=4,
     align=PP_ALIGN.CENTER)
para(tf, "replay / follow / manual / autopilot · camera + LiDAR renders",
     size=11, color=DARK, align=PP_ALIGN.CENTER)
tf = card(s, 10.75, 1.7, 2.05, 2.15, fill=BLUEBG)
para(tf, "Validation", size=13.5, bold=True, color=NAVY, first=True, space=3,
     align=PP_ALIGN.CENTER)
para(tf, "montages, GIFs, BEV, LiDAR, OSM overlays", size=11, color=DARK,
     align=PP_ALIGN.CENTER)
tf = card(s, 3.55, 4.6, 3.0, 1.95)
para(tf, "RoadRunner fixer", size=13.5, bold=True, color=NAVY, first=True,
     space=1, align=PP_ALIGN.CENTER)
para(tf, "fix_xodr_for_roadrunner.py", size=9.5, color=ACCENT, space=4,
     align=PP_ALIGN.CENTER)
para(tf, "snap · simplify · bounds · geoReference · lane links", size=11,
     color=DARK, align=PP_ALIGN.CENTER)
tf = card(s, 7.15, 4.6, 3.0, 1.95)
para(tf, "RoadRunner R2026a", size=13.5, bold=True, color=NAVY, first=True,
     space=1, align=PP_ALIGN.CENTER)
para(tf, "MATLAB automation", size=9.5, color=ACCENT, space=4,
     align=PP_ALIGN.CENTER)
para(tf, "city scene: roads + markings + signals + ground + buildings",
     size=11, color=DARK, align=PP_ALIGN.CENTER)
tf = card(s, 10.75, 4.6, 2.05, 1.95, fill=BLUEBG)
para(tf, "RR Scenario", size=13.5, bold=True, color=NAVY, first=True, space=3,
     align=PP_ALIGN.CENTER)
para(tf, "collision-safe agent replay", size=11, color=DARK,
     align=PP_ALIGN.CENTER)
arrow(s, 3.05, 2.62)   # nuscenes -> converter
arrow(s, 6.65, 2.62)   # converter -> carla
arrow(s, 10.25, 2.62)  # carla -> validation
arrow(s, 4.9, 3.98, down=True, ln=0.5, th=0.28)   # converter -> fixer
arrow(s, 6.65, 5.45)   # fixer -> roadrunner
arrow(s, 10.25, 5.45)  # roadrunner -> scenario
tf = textbox(s, 0.45, 4.75, 2.6, 1.7)
para(tf, "Two Python envs:", size=12, bold=True, color=NAVY, first=True, space=2)
para(tf, "nusc2xodr (3.12) — converter + validation", size=11.5, color=GREY,
     space=2)
para(tf, "carla915 (3.7) — CARLA client wheel", size=11.5, color=GREY)

# ================================================================ 5 · FRAMES
s = add_slide("Data and Coordinate Frames")
tf = textbox(s, 0.5, 1.2, 5.9, 2.9)
section_label(tf, "One local frame, four projections", first=True)
bullet(tf, "Patch-local frame: everything shifted to the region's lower-left "
           "corner — no rotation, no flip")
bullet(tf, "WGS84 via equirectangular linearization about each map's "
           "published origin (R⊕ = 6,378,137 m)")
bullet(tf, "CARLA is left-handed: every pose converted (x, y, ψ) → (x, −y, −ψ)")
bullet(tf, "RoadRunner: transverse-mercator geoReference + world origin at "
           "the patch corner → layers register by projection, deterministically")
hf_items = [
    ("nuScenes map frame", "metres, x east / y north"),
    ("Patch-local", "shift by (x_min, y_min)"),
    ("WGS84", "equirectangular about map origin"),
]
hflow(s, hf_items, 0.5, 4.35, 1.15, 3.2, gap=0.55, dsize=10.5)
arrow(s, 6.6, 5.75, down=True, ln=0.42, th=0.26)
arrow(s, 9.85, 5.75, down=True, ln=0.42, th=0.26)
tf = card(s, 4.4, 6.25, 3.35, 0.62)
para(tf, "CARLA: (x, −y, −ψ)", size=11.5, bold=True, color=NAVY, first=True,
     align=PP_ALIGN.CENTER)
tf = card(s, 8.0, 6.25, 4.0, 0.62)
para(tf, "RoadRunner: tmerc FullProjection", size=11.5, bold=True, color=NAVY,
     first=True, align=PP_ALIGN.CENTER)
tf = card(s, 6.65, 1.3, 6.15, 2.75, fill=BLUEBG)
para(tf, "PUBLISHED MAP ORIGINS (used everywhere)", size=12.5, bold=True,
     color=ACCENT, first=True, space=6)
for m, la, lo in [("boston-seaport", "42.3368492", "−71.0578537"),
                  ("singapore-onenorth", "1.2882101", "103.7847519"),
                  ("singapore-hollandvillage", "1.2993652", "103.7821770"),
                  ("singapore-queenstown", "1.2782562", "103.7674141")]:
    p = tf.add_paragraph(); p.space_after = Pt(4)
    r = p.add_run(); r.text = m
    r.font.size = Pt(12.5); r.font.bold = True; r.font.color.rgb = NAVY
    r2 = p.add_run(); r2.text = "   lat₀ %s, lon₀ %s" % (la, lo)
    r2.font.size = Pt(12); r2.font.color.rgb = DARK

# ================================================================ 6 · CONVERTER FLOW
s = add_slide("Stage 1 — Conversion Algorithm", stage="STAGE 1 / 6")
hflow(s, [
    ("1 · Region patch", "ego-trajectory bbox + 50 m padding"),
    ("2 · Select lanes", "lane + lane_connector records intersecting patch"),
    ("3 · Geometry", "centerlines discretized at 0.5 m; width = area / length "
     "(2–6 m)"),
], 0.5, 1.45, 1.55, 3.85, gap=0.45, dsize=11)
arrow(s, 11.15, 3.1, down=True, ln=0.4, th=0.26)
hflow(s, [
    ("6 · Emit OpenDRIVE 1.4", "roads, junctions, links, markings, signals, "
     "objects"),
    ("5 · Single-lane roads", "driving lane id −1; laneOffset = w/2 straddles "
     "the true centerline"),
    ("4 · Topology", "union-find over connectors sharing lanes → junctions; "
     "1-in / 1-out ⇒ unique links"),
], 0.5, 3.7, 1.55, 3.85, gap=0.45, dsize=11, rev=True)
tf = textbox(s, 0.5, 5.6, 12.3, 1.3)
section_label(tf, "Real lane markings", first=True)
para(tf, "Dominant nuScenes divider segment type per lane → OpenDRIVE "
         "roadMark:  DASHED → broken · SOLID/ZIGZAG → solid · DOUBLE → "
         "doubled · YELLOW → yellow.  Junction interiors left unmarked, "
         "matching the source map.", size=13.5)

# ================================================================ 7 · OPTIONAL LAYERS
s = add_slide("Stage 1 — Optional Map Layers", stage="STAGE 1 / 6")
tf = card(s, 0.5, 1.3, 3.95, 4.2)
para(tf, "CROSSWALKS", size=13, bold=True, color=ACCENT, first=True, space=5)
para(tf, "ped_crossing polygons exported to the sidecar and as OpenDRIVE "
         "crosswalk objects, attached to the nearest road via (s, t) "
         "projection.", size=12.5, space=8)
para(tf, "CARLA paints none in standalone mode → the loader draws zebra "
         "stripes (0.5 m bars / 0.6 m gaps).", size=12.5, color=GREY)
tf = card(s, 4.65, 1.3, 3.95, 4.2)
para(tf, "BUILDINGS (SYNTHESIZED)", size=13, bold=True, color=ACCENT,
     first=True, space=5)
para(tf, "nuScenes has no building layer. Occupied layers (drivable, walkway, "
         "carpark, …) buffered 2 m and subtracted from the patch; leftover "
         "blocks ≥ 120 m² tiled on a 14 m grid with 11.2 m boxes.", size=12.5,
     space=8)
para(tf, "Deterministic pseudo-random heights 8–24 m; sidecar only.",
     size=12.5, color=GREY)
tf = card(s, 8.8, 1.3, 3.95, 4.2)
para(tf, "TRAFFIC LIGHTS", size=13, bold=True, color=ACCENT, first=True,
     space=5)
para(tf, "OpenDRIVE signals (dynamic, type 1000001) at each approach-lane "
         "end, grouped into per-phase controllers by approach axis → CARLA "
         "builds functional cycling light groups.", size=12.5, space=8)
para(tf, "--osm-signals: one signal per real OSM traffic_signals node "
         "(Overpass API) → correct light counts, e.g. scene-0061: 4 lights, "
         "not 26.", size=12.5, color=GREY)
tf = textbox(s, 0.5, 5.75, 12.3, 1.1)
para(tf, "All layers are per-scene reproducible from one command:  "
         "python convert_nuscenes_to_xodr.py --all-scenes --traffic-lights "
         "--osm-signals --crosswalks", size=13, color=GREY, first=True,
     italic=True)

# ================================================================ 8 · SIDECAR
s = add_slide("Stage 1 — The Metadata Sidecar", stage="STAGE 1 / 6")
tf = card(s, 0.5, 1.3, 6.4, 5.3, fill=BLUEBG)
para(tf, "<map>_<scene>_meta.json", size=14, bold=True, color=NAVY,
     first=True, space=8)
rows = [
    ("origin, patch", "map-frame corner + region bounds"),
    ("ego_start_local", "first ego pose {x, y, yaw}"),
    ("ego_trajectory_local", "one pose per keyframe"),
    ("frame_dt", "0.5 s (nuScenes 2 Hz keyframes)"),
    ("agent_frames", "per keyframe: {id, category, x, y, yaw, wlh} for every "
     "dynamic agent"),
    ("buildings, crosswalks", "synthesized footprints, polygons"),
    ("counts", "lanes / connectors / junctions"),
]
for k, v in rows:
    p = tf.add_paragraph(); p.space_after = Pt(5)
    r = p.add_run(); r.text = k + "   "
    r.font.size = Pt(13); r.font.bold = True; r.font.color.rgb = NAVY
    r2 = p.add_run(); r2.text = v
    r2.font.size = Pt(12.5); r2.font.color.rgb = DARK
tf = textbox(s, 7.35, 1.35, 5.4, 2.6)
section_label(tf, "Why it matters", first=True)
para(tf, "The ego trajectory and agent frames are aligned 1:1 with keyframes "
         "— the same frame index identifies the same instant in every "
         "downstream artifact: camera frames, LiDAR sweeps, BEV renders, "
         "RoadRunner trajectories.", size=14)
tf = card(s, 7.35, 4.0, 5.4, 2.6)
para(tf, "EXPORTED ACROSS 10 SCENES", size=13, bold=True, color=ACCENT,
     first=True, space=6)
stat_row(tf, "404", "keyframes (39–41 per scene)")
stat_row(tf, "703", "unique annotated agents")
stat_row(tf, "783 / 150", "roads / junctions")

# ================================================================ 9 · FIXER
s = add_slide("Stage 2 — Hardening OpenDRIVE for RoadRunner", stage="STAGE 2 / 6")
tf = textbox(s, 0.5, 1.2, 12.3, 0.6)
para(tf, "Raw converter output loads in CARLA but RoadRunner rejects or "
         "mishandles it — six automated fixes:", size=14.5, color=DARK,
     first=True)
fixes = [
    ("Snap endpoints", "connector ends within 2 m snapped exactly onto linked "
     "lanes — closes disconnected roads"),
    ("Simplify plan view", "Douglas–Peucker at 3 cm tolerance — hundreds of "
     "points per road → a handful"),
    ("Default markings", "NIL-divider lanes get broken-white center + "
     "solid-white edge"),
    ("Lane-level links", "connectors get lane predecessor/successor — "
     "RoadRunner's maneuver topology"),
    ("Real header bounds", "recomputed from geometry (±10 m) — fixes "
     "“Bounds of file invalid”"),
    ("Real geoReference", "transverse-mercator at the scene origin's true "
     "lat/lon — GIS layers line up"),
]
for i, (t, d) in enumerate(fixes):
    x = 0.5 + (i % 3) * 4.15
    y = 2.0 + (i // 3) * 2.0
    tf = card(s, x, y, 3.95, 1.8)
    para(tf, "%d · %s" % (i + 1, t), size=13.5, bold=True, color=NAVY,
         first=True, space=4)
    para(tf, d, size=11.5, color=DARK)
tf = card(s, 0.5, 6.1, 12.3, 0.75, fill=BLUEBG)
para(tf, "Result: files shrink 3–7× (scene-0103: 912 kB → 199 kB) and import "
         "cleanly.  Batch: nuscenes2xodr/output/*.xodr → output_roadrunner/ "
         "with sidecars copied along.", size=13, color=DARK, first=True)

# ================================================================ 10 · CARLA LOADER
s = add_slide("Stage 3 — CARLA World Generation and Driving Modes",
              stage="STAGE 3 / 6")
hflow(s, [
    ("Load .xodr + sidecar", "scenario by id / scene / path; batch "
     "--all-scenarios"),
    ("generate_opendrive_world", "vertex 2 m · extra width 0.6 m · smoothed "
     "junctions"),
    ("Convert + snap ego", "(x, y, ψ) → (x, −y, −ψ); start pose snapped to "
     "nearest drivable lane"),
    ("Drive", "one of four modes; weather preset; scenery drawn"),
], 0.5, 1.45, 1.6, 2.85, gap=0.45, dsize=11)
modes = [
    ("REPLAY (default)", "Deterministic synchronous playback — teleported ego "
     "on a Catmull–Rom-smoothed path, 10 substeps per keyframe"),
    ("FOLLOW", "Ego driven physically by VehiclePIDController over recorded "
     "waypoints; per-segment speed ≤ 60 km/h; agents keyed to ego progress"),
    ("MANUAL", "pygame chase view + HUD; keyboard or wheel/gamepad; recorded "
     "traffic replays around you, looped"),
    ("AUTOPILOT", "Traffic Manager on the converted map (speed offset 30%)"),
]
for i, (t, d) in enumerate(modes):
    x = 0.5 + (i % 2) * 6.2
    y = 3.5 + (i // 2) * 1.6
    tf = card(s, x, y, 5.95, 1.45, fill=LIGHT)
    para(tf, t, size=12.5, bold=True, color=ACCENT, first=True, space=3)
    para(tf, d, size=11.5, color=DARK)
tf = textbox(s, 0.5, 6.7, 12.3, 0.5)
para(tf, "Every mode restores async settings and destroys spawned actors on "
         "exit; per-scenario error isolation in batch runs.", size=12,
     color=GREY, first=True)

# ================================================================ 11 · REPLAY INTERNALS
s = add_slide("Stage 3 — Replay Engine Internals", stage="STAGE 3 / 6")
tf = textbox(s, 0.5, 1.2, 6.9, 5.6)
section_label(tf, "Determinism and smoothness", first=True)
bullet(tf, "Synchronous mode, fixed Δt = frame_dt / substeps (0.05 s) — no "
           "camera flicker, even velocity")
bullet(tf, "Catmull–Rom interpolation across 2 Hz keyframes; shortest-arc "
           "yaw blending")
bullet(tf, "Ground height cached on a 1 m grid — even per-frame timing")
bullet(tf, "Every actor grounded by its own bounding box: pedestrians don't "
           "sink, vehicles don't float")
para(tf, "", size=3)
section_label(tf, "Recorded agents")
bullet(tf, "Spawned on first appearance; physics off, teleported per tick; "
           "hidden 200 m underground when absent")
bullet(tf, "Category → blueprint pools (walkers, bikes, motorcycles, bus, "
           "trucks, 7 car models)")
bullet(tf, "Model + color keyed on a stable hash of the instance token — "
           "consistent across frames and runs; red reserved for ego")
para(tf, "", size=3)
section_label(tf, "Trajectory overlay")
bullet(tf, "2 s history (dimmed) + 6 s future per actor at 2 Hz — ego red, "
           "vehicles blue, pedestrians green, cyclists magenta")
picture(s, IMG + r"\cameras\scene-0103\CAM_FRONT\020.png", 7.7, 1.5, w=5.1,
        caption="Simulated CAM_FRONT — recorded agents replayed on the "
                "generated map")
picture(s, IMG + r"\topdown\scene-0103\frames\020.png", 8.85, 4.85, h=1.95,
        caption="Same instant in BEV with overlay")

# ================================================================ 12 · SENSORS
s = add_slide("Stage 3 — Calibrated Sensor Rig", stage="STAGE 3 / 6")
tf = textbox(s, 0.5, 1.2, 6.9, 5.6)
section_label(tf, "Six cameras, real projection", first=True)
bullet(tf, "Free-floating sensors (not attached): CARLA's vehicle pivot ≠ "
           "nuScenes ego origin")
bullet(tf, "Each tick teleported to  ego_pose ∘ real extrinsic  (matrix "
           "composition, converted back to CARLA Euler)")
bullet(tf, "Native 1600×900; per-camera FOV = 2·atan(W / 2fx) from real "
           "intrinsics — CAM_FRONT 64.56°, not the approximate 70°")
bullet(tf, "Saved views share the real cameras' projection → directly usable "
           "for calibration-sensitive perception evaluation (BEVFormer-class)")
bullet(tf, "Fallback approximate rig when no calibration sidecar exists")
para(tf, "", size=3)
section_label(tf, "LiDAR (HDL-32E-like)")
bullet(tf, "Mounted at the nuScenes position (x 0.90, z 1.84 m); 32 channels, "
           "+10°/−30° FOV, 70 m range")
bullet(tf, "Rotation locked to the sync tick → exactly one 360° sweep per "
           "keyframe, saved as .ply")
tf = card(s, 7.7, 1.35, 5.1, 1.7, fill=BLUEBG)
para(tf, "OUTPUT LAYOUT (MIRRORS NUSCENES)", size=12.5, bold=True,
     color=ACCENT, first=True, space=5)
para(tf, "cameras[_follow]/<scene>/<CAM_NAME>/NNN.png", size=12, color=DARK,
     space=3)
para(tf, "lidar_carla/<scene>/NNN.ply — one per keyframe", size=12, color=DARK)
picture(s, IMG + r"\lidar_carla\scene-0103\frames\020.png", 8.05, 3.3, h=3.1,
        caption="CARLA LiDAR sweep, BEV-rendered (scene-0103)")

# ================================================================ 13 · VALIDATION FLOW
s = add_slide("Stage 4 — Validation Pipeline", stage="STAGE 4 / 6")
hflow(s, [
    ("CARLA renders", "6-cam frames + LiDAR sweeps per keyframe"),
    ("Montage", "labelled 2×3 camera grids: real over sim"),
    ("Animate", "per-scene GIFs at the native 2 Hz"),
    ("Composite", "cameras | LiDAR pair | top-down — one panel per frame"),
], 0.5, 1.45, 1.7, 2.85, gap=0.45, dsize=11)
tf = card(s, 0.5, 3.6, 6.0, 3.0)
para(tf, "FOUR INDEPENDENT CHECKS", size=13, bold=True, color=ACCENT,
     first=True, space=6)
bullet(tf, "Cameras: real nuScenes vs. CARLA, same keyframe", size=12.5)
bullet(tf, "LiDAR: real LIDAR_TOP vs. CARLA sweep, identical ego-centred "
           "north-up frame", size=12.5)
bullet(tf, "BEV: scene re-rendered from the exported .xodr + sidecar alone",
       size=12.5)
bullet(tf, "OSM: lanes + trajectory + signals over real map data (Overpass) — "
           "validates georeferencing and light counts", size=12.5)
tf = card(s, 6.8, 3.6, 6.0, 3.0, fill=BLUEBG)
para(tf, "COVERAGE (ALL 10 SCENES)", size=13, bold=True, color=ACCENT,
     first=True, space=6)
stat_row(tf, "5,082", "camera PNGs (replay + follow)", vsize=19, ksize=12.5)
stat_row(tf, "404 + 404", "LiDAR sweeps (.ply) + BEV renders", vsize=19,
         ksize=12.5)
stat_row(tf, "60", "comparison GIFs (real / sim / stacked)", vsize=19,
         ksize=12.5)
stat_row(tf, "1,616", "composite panel frames", vsize=19, ksize=12.5)

# ================================================================ 14 · VALIDATION EXAMPLE
s = add_slide("Stage 4 — Real vs. Simulated, One Glance", stage="STAGE 4 / 6")
picture(s, IMG + r"\aligned_full\scene-0103\frames\020.png", 1.15, 1.6, w=11.0,
        caption="scene-0103, frame 20 — REAL six-camera view (top) vs. SIM "
                "(bottom) · real-over-sim LiDAR · reconstructed top-down")
tf = textbox(s, 0.9, 6.05, 11.5, 0.9)
para(tf, "Every keyframe of every scene exists in this form; the animated "
         "versions (nuscenes_real / carla_sim / comparison GIFs) play the "
         "full 20 s scene at 2 Hz.", size=13.5, color=GREY, first=True,
     align=PP_ALIGN.CENTER)

# ================================================================ 15 · RR CITY FLOW
s = add_slide("Stage 5 — RoadRunner City Scenes", stage="STAGE 5 / 6")
tf = textbox(s, 0.5, 1.2, 12.3, 0.55)
para(tf, "Five data layers assembled per scene — one OpenDRIVE import + one "
         "HD-map (.rrhd) import, placed by projection:", size=14.5,
     first=True)
layers = [
    ("Roads", "hardened .xodr, FullProjection, world origin at patch corner"),
    ("Real markings", "nuScenes lane/road dividers as HD-map curve markings — "
     "aligned by construction"),
    ("Traffic signals", "surveyed positions → assembled post + mast arm + "
     "3-light heads"),
    ("Ground", "level slab (patch +120 m) of flat lanes at z = −0.15 m"),
    ("OSM buildings", "footprints → oriented boxes, shrunk to clear lanes, "
     "real heights"),
]
for i, (t, d) in enumerate(layers):
    x = 0.5 + i * 2.51
    tf = card(s, x, 2.0, 2.31, 2.15)
    para(tf, t, size=12.5, bold=True, color=NAVY, first=True, space=4,
         align=PP_ALIGN.CENTER)
    para(tf, d, size=10.5, color=DARK, align=PP_ALIGN.CENTER)
    if i < 4:
        pass
ar = arrow(s, 5.9, 4.3, down=True, ln=0.4, th=0.28)
tf = card(s, 3.55, 4.85, 5.2, 0.7, fill=BLUEBG)
para(tf, "roadrunnerHDMap  →  <scene>_city.rrscene  (× 10)", size=13,
     bold=True, color=NAVY, first=True, align=PP_ALIGN.CENTER)
tf = textbox(s, 0.5, 5.85, 12.3, 1.0)
bullet(tf, "OSM extract auto-downloaded per scene (~200 m padded bbox); "
           "markings parsed once and cached; preview figure for visual QA "
           "before import", size=12.5, first=True)
bullet(tf, "Generic scene selection (scene-01…10 or real ids) — all 10 built "
           "with the same script; per-scene manualShift calibration hook for "
           "residual OSM offset", size=12.5)

# ================================================================ 16 · SIGNALS + BUILDINGS
s = add_slide("Stage 5 — Signal Assembly and Building Decomposition",
              stage="STAGE 5 / 6")
tf = textbox(s, 0.5, 1.2, 6.0, 5.5)
section_label(tf, "Traffic-signal assembly", first=True)
para(tf, "The recorded point marks the light over the road — not the pole. "
         "Per signal:", size=13, space=6)
hf = [("Snap facing", "to the nearest lane travelling toward the light"),
      ("Find roadside", "probe which side of the road is off-road"),
      ("Walk pole out", "step outward until clear of the drivable area"),
      ("Assemble", "9.2 m post + 7.8 m mast arm + two 3-light heads")]
for i, (t, d) in enumerate(hf):
    p = tf.add_paragraph(); p.space_after = Pt(5)
    r = p.add_run(); r.text = "%d · %s   " % (i + 1, t)
    r.font.size = Pt(13); r.font.bold = True; r.font.color.rgb = NAVY
    r2 = p.add_run(); r2.text = d
    r2.font.size = Pt(12.5); r2.font.color.rgb = DARK
para(tf, "Measured FBX half-extents keep the stock assets' natural "
         "proportions (assembly copied from FourWaySignal.rrscene).",
     size=12, color=GREY, space=8)
para(tf, "Signal source configurable: .xodr <signal> entries · OSM nodes · "
         "nuScenes traffic_light layer.", size=12, color=GREY)
tf = textbox(s, 6.9, 1.2, 5.9, 5.5)
section_label(tf, "Building footprints → oriented boxes", first=True)
hf = [("Min-area rect", "of the OSM footprint (convex-hull rotating "
       "calipers)"),
      ("Split if concave", "fill ratio < 0.55 → halve across the long axis, "
       "recurse (depth ≤ 3) — L/U shapes become tight boxes"),
      ("Shrink off roads", "scale about center until 2 m clear of lane "
       "centerlines; floor at 80% — buildings shrink but survive"),
      ("Realize", "boxes cycle 3 downtown meshes; heights from OSM tags "
       "(levels × 3.2 m, default 10 m)")]
for i, (t, d) in enumerate(hf):
    p = tf.add_paragraph(); p.space_after = Pt(6)
    r = p.add_run(); r.text = "%d · %s   " % (i + 1, t)
    r.font.size = Pt(13); r.font.bold = True; r.font.color.rgb = NAVY
    r2 = p.add_run(); r2.text = d
    r2.font.size = Pt(12.5); r2.font.color.rgb = DARK
para(tf, "Everything travels in one .rrhd file — requires the Scene Builder "
         "license on import.", size=12, color=GREY)

# ================================================================ 17 · RR REPLAY
s = add_slide("Stage 6 — Collision-Safe Scenario Replay", stage="STAGE 6 / 6")
hflow(s, [
    ("Agent frames", "sidecar tracks per instance, 0.5 s keyframes"),
    ("Pre-prune", "0.1 s timeline · OBB separating-axis tests · inflated "
     "footprints"),
    ("CSV export", "time, x, y, yaw + spawn/remove times; 2 mm waypoint creep"),
    ("importScenario", "per-category actor assets; ego = green sedan"),
    ("Save", "<scene>_replay .rrscenario × 10"),
], 0.5, 1.45, 1.75, 2.28, gap=0.35, dsize=10.5)
tf = textbox(s, 0.5, 3.55, 6.6, 3.2)
section_label(tf, "Why pre-prune?", first=True)
para(tf, "RoadRunner Scenario fails the simulation on any actor contact — and "
         "real annotated boxes plus substitute assets of different sizes do "
         "touch.", size=13, space=6)
bullet(tf, "Replay resampled at 0.1 s with linear interpolation — exactly how "
           "RoadRunner moves actors, so contacts between keyframes are caught")
bullet(tf, "Vehicle footprints ×1.25 + 0.3 m, pedestrians +0.15 m — strictly "
           "more conservative than RoadRunner's own test")
bullet(tf, "First contact → truncate just before it (RemoveTime); contact at "
           "first frame → drop.  Ego untouchable; waiting actors modelled as "
           "parked from t = 0")
tf = card(s, 7.4, 3.55, 5.35, 1.85, fill=BLUEBG)
para(tf, "SURVIVAL", size=13, bold=True, color=ACCENT, first=True, space=5)
stat_row(tf, "504 / 703", "agent trajectories imported (plus ego ×10)",
         vsize=20, ksize=12.5)
para(tf, "Rest: truncated away, < 2 frames, or unmapped categories.",
     size=12, color=GREY)
tf = card(s, 7.4, 5.6, 5.35, 1.15)
para(tf, "ASSETS", size=12.5, bold=True, color=ACCENT, first=True, space=4)
para(tf, "Sedan/SUV/Compact (cycled) · PickupTruck · SchoolBus · Ambulance · "
         "NCAP bike + motorcycle · .rrchar pedestrians", size=11.5)

# ================================================================ 18 · RESULTS
s = add_slide("Results — Per-Scene Statistics")
rows = [
    ("scene-0061", "onenorth",       "39", "89",  "73",  "18", "57"),
    ("scene-0103", "boston-seaport", "40", "113", "93",  "18", "109"),
    ("scene-0553", "boston-seaport", "41", "40",  "54",  "9",  "29"),
    ("scene-0655", "boston-seaport", "41", "117", "77",  "14", "73"),
    ("scene-0757", "boston-seaport", "41", "24",  "30",  "5",  "12"),
    ("scene-0796", "queenstown",     "40", "50",  "130", "18", "35"),
    ("scene-0916", "queenstown",     "41", "94",  "50",  "15", "63"),
    ("scene-1077", "hollandvillage", "41", "59",  "137", "30", "38"),
    ("scene-1094", "hollandvillage", "40", "85",  "94",  "18", "65"),
    ("scene-1100", "hollandvillage", "40", "32",  "45",  "5",  "23"),
    ("Total", "",                    "404", "703", "783", "150", "504"),
]
hdr = ("Scene", "Map", "Frames", "Agents", "Roads", "Junctions", "RR traj.")
tbl = s.shapes.add_table(len(rows) + 1, len(hdr), Inches(0.5), Inches(1.35),
                         Inches(8.4), Inches(5.3)).table
for j, h in enumerate(hdr):
    c = tbl.cell(0, j); c.text = h
    pr = c.text_frame.paragraphs[0]
    pr.runs[0].font.size = Pt(12.5); pr.runs[0].font.bold = True
for i, row in enumerate(rows, start=1):
    for j, v in enumerate(row):
        c = tbl.cell(i, j); c.text = v
        pr = c.text_frame.paragraphs[0]
        if pr.runs:
            pr.runs[0].font.size = Pt(11.5)
            pr.runs[0].font.bold = (row[0] == "Total")
tf = card(s, 9.25, 1.35, 3.55, 3.1, fill=BLUEBG)
para(tf, "AGENT POPULATION", size=12.5, bold=True, color=ACCENT, first=True,
     space=6)
for k, v in [("cars", "390"), ("adult pedestrians", "213"), ("trucks", "28"),
             ("motorcycles", "20"), ("bicycles", "15"), ("buses", "15"),
             ("other", "32")]:
    p = tf.add_paragraph(); p.space_after = Pt(3)
    r = p.add_run(); r.text = v + "  "
    r.font.size = Pt(14); r.font.bold = True; r.font.color.rgb = NAVY
    r2 = p.add_run(); r2.text = k
    r2.font.size = Pt(12); r2.font.color.rgb = GREY
tf = textbox(s, 9.25, 4.65, 3.55, 2.2)
para(tf, "Signals per scene 0–9 (39 total); two scenes have none in their "
         "patch.", size=12, color=GREY, first=True, space=6)
para(tf, "RR trajectories = agents surviving collision pruning; ego adds one "
         "per scene.", size=12, color=GREY)

# ================================================================ 19 · EXAMPLES
s = add_slide("Scenario Examples — scene-0103 (Boston Seaport)")
picture(s, IMG + r"\osm_aligned\scene-0103.png", 0.55, 1.45, h=2.6,
        caption="OSM overlay: lanes, ego, signals")
picture(s, IMG + r"\topdown\scene-0103\frames\020.png", 4.9, 1.45, h=2.6,
        caption="BEV from exported map + sidecar")
picture(s, IMG + r"\lidar\scene-0103\frames\020.png", 0.55, 4.55, h=2.25,
        caption="Real LIDAR_TOP, frame 20")
picture(s, IMG + r"\lidar_carla\scene-0103\frames\020.png", 4.9, 4.55, h=2.25,
        caption="CARLA LiDAR, same frame")
tf = textbox(s, 8.9, 1.5, 3.9, 5.2)
para(tf, "Same frame index everywhere — sidecar keyframes align cameras, "
         "LiDAR, BEV, and RoadRunner trajectories 1:1.", size=13.5,
     first=True, space=10)
para(tf, "113 agents in this scene; 109 trajectories survive into the "
         "RoadRunner scenario — the densest of the ten.", size=13, space=10)
para(tf, "Animated versions per scene: real / sim / comparison camera GIFs, "
         "LiDAR GIFs, full-panel GIFs.", size=12.5, color=GREY)

# ================================================================ 20 · LIMITS + NEXT
s = add_slide("Limitations and Next Steps")
tf = card(s, 0.5, 1.3, 6.0, 4.3)
para(tf, "KNOWN LIMITATIONS", size=13.5, bold=True, color=ACCENT, first=True,
     space=7)
for t in ["Straight-segment geometry; flat elevation (z = 0)",
          "One driving lane per road; LHT emitted as right-side lanes",
          "Small single-connector junctions not consolidated",
          "OSM building shift calibrated for scene-0103 only",
          "Scripted replay ignores traffic lights (matches recorded reality)",
          "RoadRunner replay trades fidelity for its fail-on-collision rule"]:
    bullet(tf, t, size=13)
tf = card(s, 6.8, 1.3, 6.0, 4.3, fill=BLUEBG)
para(tf, "NEXT STEPS (07/24 →)", size=13.5, bold=True, color=ACCENT,
     first=True, space=7)
for t in ["Calibrate manualShift for the remaining 9 scenes",
          "Add elevation from LiDAR ground fits",
          "Export RR scenarios to OpenSCENARIO / back into CARLA — close the "
          "loop between back-ends",
          "Perception evaluation (BEVFormer-class) on calibrated CARLA "
          "renders vs. real frames — domain-gap study",
          "Scale beyond v1.0-mini to trainval maps"]:
    bullet(tf, t, size=13)
tf = textbox(s, 0.5, 5.9, 12.3, 1.0)
para(tf, "Everything is reproducible: one converter command + one loader "
         "command (Python) and two MATLAB scripts per scene.", size=14,
     bold=True, color=NAVY, first=True, align=PP_ALIGN.CENTER)

prs.save(OUT)
print("saved", OUT, "slides:", len(prs.slides._sldIdLst))
