import json
import os
import shutil
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from nusc_carla.xodr import write_carla_compatible_xodr
from tools.prepare_import_package import stage_package


class PrepareImportPackageTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).parents[1] / "generated" / "test_prepare_package"
        if self.root.exists():
            shutil.rmtree(self.root)
        self.root.mkdir(parents=True)

    def tearDown(self):
        if self.root.exists():
            shutil.rmtree(self.root)

    def test_stages_rrdata_and_keeps_manifest_xodr(self):
        root = self.root
        source_xodr = root / "authoritative.xodr"
        source_xodr.write_text("<OpenDRIVE>authoritative</OpenDRIVE>", encoding="utf-8")
        fbx = root / "Map.fbx"
        fbx.write_bytes(b"fbx")
        (root / "Map.rrdata.xml").write_text("<Materials/>", encoding="utf-8")
        manifest = root / "manifest.json"
        manifest.write_text(json.dumps({
            "map": {
                "asset_name": "Map",
                "package_name": "Package",
                "unreal_source_level": "/Game/Package/Maps/Map/Map",
            },
            "source": {"xodr": os.fspath(source_xodr)},
        }), encoding="utf-8")

        package, _ = stage_package(manifest, fbx, root / "stage")
        package = Path(package)
        self.assertEqual((package / "Map.fbx").read_bytes(), b"fbx")
        self.assertEqual((package / "Map.rrdata.xml").read_text(encoding="utf-8"),
                         "<Materials/>")
        self.assertIn("authoritative", (package / "Map.xodr").read_text(encoding="utf-8"))

    def test_requires_carla_filmbox_metadata(self):
        root = self.root
        xodr = root / "Map.xodr"
        xodr.write_text("<OpenDRIVE/>", encoding="utf-8")
        fbx = root / "Map.fbx"
        fbx.write_bytes(b"fbx")
        manifest = root / "manifest.json"
        manifest.write_text(json.dumps({
            "map": {"asset_name": "Map"},
            "source": {"xodr": os.fspath(xodr)},
        }), encoding="utf-8")
        with self.assertRaisesRegex(FileNotFoundError, "CARLA material metadata"):
            stage_package(manifest, fbx, root / "stage")

    def test_remaps_road_junction_id_collision_and_all_references(self):
        source = self.root / "collision.xodr"
        target = self.root / "fixed.xodr"
        source.write_text("""<?xml version="1.0"?>
<OpenDRIVE>
  <road id="1" junction="-1"><link><successor elementType="junction" elementId="1"/></link></road>
  <road id="2" junction="1"><link><predecessor elementType="road" elementId="1"/></link></road>
  <junction id="1"><connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start"/></junction>
</OpenDRIVE>""", encoding="utf-8")
        mapping = write_carla_compatible_xodr(source, target)
        self.assertEqual(set(mapping), {1})
        root = ET.parse(target).getroot()
        new_id = str(mapping[1])
        self.assertNotIn(int(new_id), {1, 2})
        self.assertEqual(root.find("junction").get("id"), new_id)
        self.assertEqual(root.find('./road[@id="2"]').get("junction"), new_id)
        self.assertEqual(
            root.find('./road[@id="1"]/link/successor').get("elementId"), new_id)


if __name__ == "__main__":
    unittest.main()
