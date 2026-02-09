"""Tests for the SDF parser."""

import os
import sys
import math

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sdf_to_usd.parser import SdfParser


TEST_DATA_DIR = os.path.join(os.path.dirname(__file__), "test_data")


def test_parse_simple_box():
    """Test parsing a simple SDF with primitive geometry."""
    sdf_path = os.path.join(TEST_DATA_DIR, "simple_box.sdf")
    parser = SdfParser(sdf_path)
    model = parser.parse()

    # Model
    assert model.name == "simple_box_robot"
    assert len(model.links) == 3
    assert len(model.joints) == 2

    # Base link
    base = model.get_link("base_link")
    assert base is not None
    assert abs(base.pose.z - 0.5) < 1e-6
    assert base.inertial is not None
    assert abs(base.inertial.mass - 5.0) < 1e-6
    assert len(base.visuals) == 1
    assert len(base.collisions) == 1

    # Box geometry
    vis = base.visuals[0]
    assert vis.geometry.box is not None
    assert abs(vis.geometry.box.size[0] - 0.5) < 1e-6
    assert abs(vis.geometry.box.size[1] - 0.3) < 1e-6
    assert abs(vis.geometry.box.size[2] - 0.2) < 1e-6

    # Material
    assert vis.material is not None
    assert abs(vis.material.diffuse[0] - 0.8) < 1e-6

    # Wheel (degrees="true" pose)
    wheel = model.get_link("wheel_left")
    assert wheel is not None
    assert abs(wheel.pose.roll - math.radians(90)) < 1e-6

    # Cylinder visual
    wvis = wheel.visuals[0]
    assert wvis.geometry.cylinder is not None
    assert abs(wvis.geometry.cylinder.radius - 0.1) < 1e-6

    # Sphere collision
    wcol = wheel.collisions[0]
    assert wcol.geometry.sphere is not None
    assert abs(wcol.geometry.sphere.radius - 0.1) < 1e-6

    # Joints
    j1 = model.joints[0]
    assert j1.name == "wheel_left_joint"
    assert j1.type == "revolute"
    assert j1.parent == "base_link"
    assert j1.child == "wheel_left"
    assert abs(j1.axis.damping - 0.01) < 1e-6

    print("All parser tests passed!")


def test_parse_mini_talon():
    """Test parsing the mini_talon_vtail SDF model."""
    sdf_path = "/home/emin/İndirilenler/mini_talon_vtail/model.sdf"
    if not os.path.exists(sdf_path):
        print("SKIP: mini_talon model not found")
        return

    parser = SdfParser(sdf_path)
    model = parser.parse()

    assert model.name == "mini_talon_vtail"
    assert len(model.links) == 7  # base_link + 6 child links
    assert len(model.joints) == 6

    # Check base_link has many visuals
    base = model.get_link("base_link")
    assert base is not None
    assert len(base.visuals) >= 8  # fuselage, wings, tails, etc.
    assert len(base.collisions) >= 4  # fuselage, wings, tails

    # Check mesh URIs are resolved
    for vis in base.visuals:
        if vis.geometry and vis.geometry.mesh:
            assert vis.geometry.mesh.resolved_path != ""
            assert os.path.exists(vis.geometry.mesh.resolved_path), \
                f"Mesh not found: {vis.geometry.mesh.resolved_path}"

    # Check PBR material
    has_pbr = any(v.material and v.material.pbr for v in base.visuals)
    assert has_pbr, "Expected PBR material on base_link visuals"

    # Check joints
    joint_names = {j.name for j in model.joints}
    assert "left_aileron_joint" in joint_names
    assert "motor_joint" in joint_names

    print("Mini talon parser test passed!")


if __name__ == "__main__":
    test_parse_simple_box()
    test_parse_mini_talon()
