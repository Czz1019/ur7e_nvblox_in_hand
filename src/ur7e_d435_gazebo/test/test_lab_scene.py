"""Geometry checks for the procedural lab fixture, independent of ROS."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from lab_scene import build_lab_scene


def test_fixture_geometry():
    cfg = yaml.safe_load((ROOT/'config/lab_scene.yaml').read_text())
    xml, manifest = build_lab_scene(cfg, ROOT/'meshes/lab')
    root = ET.fromstring(xml)
    visuals, collisions = root.findall('.//visual'), root.findall('.//collision')
    assert len(visuals) == len(collisions) == len(manifest['fixtures'])
    assert manifest['tube_count'] == 19 and manifest['rack_count'] == 2
    assert manifest['bench_top_z'] == 0 and manifest['floor_z'] == -.82
    for v, c in zip(visuals, collisions):
        assert ET.tostring(v.find('geometry')) == ET.tostring(c.find('geometry'))
        assert v.findtext('pose') == c.findtext('pose')
    for rack in cfg['racks']:
        mesh = trimesh.load(ROOT/'meshes/lab'/(rack['name']+'_ring.stl'))
        assert mesh.is_watertight and mesh.is_winding_consistent
        assert mesh.euler_number == 0 and mesh.volume > 0
        for vertex in mesh.vertices:
            assert (vertex[0]**2 + vertex[1]**2)**.5 >= rack['hole_radius']-1e-8
