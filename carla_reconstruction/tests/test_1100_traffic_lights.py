"""Narrow traffic-light correction; no CARLA/Unreal server required."""

import unittest
import xml.etree.ElementTree as ET

from scene_overrides.deduplicate_1100_traffic_lights import (
    EXPECTED_SIGNALS, POSITIONS, ROAD_ANCHORS, patch_xodr, unrelated_structure)


def fixture():
    root = ET.Element("OpenDRIVE")
    ET.SubElement(root, "header", name="scene-1100")
    for road_id, ids in (("2", ["20000", "20002", "20003", "20007"]),
                         ("7", ["20004", "20005"]), ("8", ["20001", "20006"])):
        length, x, y, heading = ROAD_ANCHORS.get(road_id, (74.057273, -4.96, -24.08, .939))
        road = ET.SubElement(root, "road", id=road_id, junction="-1", length=str(length))
        plan = ET.SubElement(road, "planView")
        ET.SubElement(plan, "geometry", x=str(x), y=str(y), hdg=str(heading))
        ET.SubElement(road, "lanes", sentinel="preserve lane geometry and markings")
        signals = ET.SubElement(road, "signals")
        s, t = POSITIONS.get(road_id, (73.4584, -2.8301))
        for signal_id in ids:
            signal = ET.SubElement(signals, "signal", id=signal_id, name="TL_" + signal_id,
                                   s=str(s), t=str(t), orientation="+", dynamic="yes", type="1000001")
            ET.SubElement(signal, "validity", fromLane="-1", toLane="-1")
    controller = ET.SubElement(root, "controller", id="30000", sequence="0")
    for signal_id in sorted(EXPECTED_SIGNALS):
        ET.SubElement(controller, "control", signalId=signal_id, type="")
    ET.SubElement(root, "junction", id="46", sentinel="preserve connections and phases")
    return ET.tostring(root)


class TrafficLight1100Tests(unittest.TestCase):
    def test_one_physical_signal_controls_both_approach_lanes(self):
        result, report = patch_xodr(fixture())
        root = ET.fromstring(result)
        self.assertEqual(len(root.findall("road/signals/signal")), 5)
        signal = root.find("road[@id='8']/signals/signal")
        reference = root.find("road[@id='7']/signals/signalReference")
        self.assertEqual(signal.get("id"), "20001")
        self.assertEqual(reference.get("id"), "20001")
        self.assertEqual(reference.get("s"), "10.3078")
        self.assertEqual(reference.find("validity").attrib, {"fromLane": "-1", "toLane": "-1"})
        self.assertEqual(report["controlled_roads_after"], [7, 8])

    def test_geometry_other_approach_and_controller_attributes_unchanged(self):
        before = fixture()
        after, _ = patch_xodr(before)
        self.assertEqual(unrelated_structure(ET.fromstring(before)), unrelated_structure(ET.fromstring(after)))
        self.assertEqual(ET.fromstring(after).find("controller").get("sequence"), "0")

    def test_no_dangling_controller_entries(self):
        result, _ = patch_xodr(fixture())
        root = ET.fromstring(result)
        ids = {s.get("id") for s in root.findall("road/signals/signal")}
        self.assertEqual(ids, {c.get("signalId") for c in root.findall("controller/control")})
        self.assertEqual(ids, {"20000", "20001", "20002", "20003", "20007"})

    def test_idempotent(self):
        first, _ = patch_xodr(fixture())
        second, report = patch_xodr(first)
        self.assertEqual(first, second)
        self.assertTrue(report["already_applied"])

    def test_rejects_different_scene_geometry(self):
        data = fixture().replace(b'x="64.423578"', b'x="65.0"')
        with self.assertRaisesRegex(ValueError, "geometry differs"):
            patch_xodr(data)

    def test_rejects_changed_lane_applicability(self):
        with self.assertRaisesRegex(ValueError, "lane applicability"):
            patch_xodr(fixture().replace(b'fromLane="-1"', b'fromLane="-2"'))

    def test_rejects_partial_previous_change(self):
        with self.assertRaisesRegex(ValueError, "signal/controller IDs"):
            patch_xodr(fixture().replace(b'id="20005"', b'id="21005"'))

    def test_rejects_additional_reference_to_removed_light(self):
        root = ET.fromstring(fixture())
        ET.SubElement(root.find("road[@id='2']/signals"), "signalReference", id="20004")
        with self.assertRaisesRegex(ValueError, "additional references"):
            patch_xodr(ET.tostring(root))

    def test_rejects_conflicting_controller(self):
        root = ET.fromstring(fixture())
        other = ET.SubElement(root, "controller", id="other")
        ET.SubElement(other, "control", signalId="20004")
        with self.assertRaisesRegex(ValueError, "different controllers"):
            patch_xodr(ET.tostring(root))

    def test_rejects_distinct_headings_for_same_lane(self):
        root = ET.fromstring(fixture())
        root.find("road[@id='7']/signals/signal[@id='20005']").set("hOffset", "1.0")
        with self.assertRaisesRegex(ValueError, "not exact"):
            patch_xodr(ET.tostring(root))


if __name__ == "__main__":
    unittest.main()
