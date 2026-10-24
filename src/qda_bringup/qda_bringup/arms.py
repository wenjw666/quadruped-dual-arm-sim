"""Attach the Aubo A3 left and right arms, each with a parallel jaw."""

from __future__ import annotations

import os
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def _prefix(elem: ET.Element, prefix: str) -> None:
    name = elem.get("name")
    if name:
        elem.set("name", prefix + name)
    for child in elem:
        if child.tag in ("parent", "child") and child.get("link"):
            child.set("link", prefix + child.get("link"))


def _arm(prefix: str, mount: str) -> list[ET.Element]:
    path = os.path.join(get_package_share_directory("aubo_description"), "urdf", "aubo_A3.urdf")
    root = ET.parse(path).getroot()
    kept = []
    for elem in list(root):
        if elem.tag not in ("link", "joint"):
            continue
        if elem.get("name") in ("world", "world_joint"):
            continue
        for extra in list(elem.iter("property")):
            parent = elem if extra in list(elem) else None
            if parent is not None:
                parent.remove(extra)
        _prefix(elem, prefix)
        kept.append(elem)
    attach = ET.Element("joint", {"name": f"{prefix}arm_attach", "type": "fixed"})
    origin = ET.SubElement(attach, "origin")
    origin.set("xyz", "0 0 0")
    origin.set("rpy", "0 0 0")
    parent = ET.SubElement(attach, "parent")
    parent.set("link", mount)
    child = ET.SubElement(attach, "child")
    child.set("link", f"{prefix}base_link")
    kept.append(attach)
    kept.extend(_gripper(prefix))
    return kept


def _inertial(link: ET.Element, mass: str = "0.02") -> None:
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").set("value", mass)
    inertia = ET.SubElement(inertial, "inertia")
    for key in ("ixx", "iyy", "izz"):
        inertia.set(key, "1e-5")
    for key in ("ixy", "ixz", "iyz"):
        inertia.set(key, "0")


def _origin(parent: ET.Element, xyz: str, rpy: str = "0 0 0") -> ET.Element:
    origin = ET.SubElement(parent, "origin")
    origin.set("xyz", xyz)
    origin.set("rpy", rpy)
    return origin


_material_seq = 0


def _paint(visual: ET.Element, rgba: str) -> None:
    # Gazebo drops the whole visual if <material> has no name.
    global _material_seq
    _material_seq += 1
    material = ET.SubElement(visual, "material")
    material.set("name", f"gripper_mat_{_material_seq}")
    color = ET.SubElement(material, "color")
    color.set("rgba", rgba)


def _box(
    link: ET.Element,
    size: str,
    xyz: str,
    rgba: str,
    collision: bool = False,
    collision_size: str | None = None,
    collision_xyz: str | None = None,
) -> None:
    visual = ET.SubElement(link, "visual")
    _origin(visual, xyz)
    ET.SubElement(ET.SubElement(visual, "geometry"), "box").set("size", size)
    _paint(visual, rgba)
    if collision:
        collide = ET.SubElement(link, "collision")
        _origin(collide, collision_xyz or xyz)
        ET.SubElement(ET.SubElement(collide, "geometry"), "box").set("size", collision_size or size)


def _cylinder(link: ET.Element, radius: str, length: str, xyz: str, rgba: str, rpy: str = "0 0 0") -> None:
    visual = ET.SubElement(link, "visual")
    _origin(visual, xyz, rpy)
    cyl = ET.SubElement(ET.SubElement(visual, "geometry"), "cylinder")
    cyl.set("radius", radius)
    cyl.set("length", length)
    _paint(visual, rgba)


def _gripper(prefix: str) -> list[ET.Element]:
    """Parallel two-finger jaw on the A3 flange.

    Flange +Z is the tool axis. The fingers run along that axis and open
    along Y. Each finger is one prismatic joint, 0 closed to 0.038 open,
    so a stock trajectory controller can drive it without a vendor driver.
    Closed opening is 8 mm; fully open is 84 mm.
    """
    gripper = ET.Element("link", {"name": f"{prefix}gripper"})
    _inertial(gripper, "0.25")
    _cylinder(gripper, "0.0315", "0.016", "0 0 0.008", "0.55 0.56 0.58 1")
    _box(gripper, "0.078 0.100 0.044", "0 0 0.040", "0.22 0.23 0.25 1", collision=True)
    _box(gripper, "0.062 0.028 0.014", "0 0 0.066", "0.16 0.17 0.18 1")
    joint = ET.Element("joint", {"name": f"{prefix}gripper_joint", "type": "fixed"})
    _origin(joint, "0 0 0.002")
    ET.SubElement(joint, "parent").set("link", f"{prefix}flange")
    ET.SubElement(joint, "child").set("link", f"{prefix}gripper")

    parts = [gripper, joint]
    fingers = (
        ("left", "0 1 0", "0.004", "0.007", "0.002"),
        ("right", "0 -1 0", "-0.004", "-0.007", "-0.002"),
    )
    for side, axis, joint_y, bar_y, pad_y in fingers:
        link = ET.Element("link", {"name": f"{prefix}finger_{side}"})
        _inertial(link, "0.04")
        # Visual tips stop at the bench; collision stops at the pad so a
        # top grasp does not stab the table.
        _box(
            link,
            "0.020 0.014 0.060",
            f"0 {bar_y} 0.042",
            "0.72 0.73 0.75 1",
            collision=True,
            collision_size="0.020 0.014 0.040",
            collision_xyz=f"0 {bar_y} 0.032",
        )
        _box(link, "0.016 0.005 0.036", f"0 {pad_y} 0.034", "0.06 0.06 0.07 1")
        finger = ET.Element("joint", {"name": f"{prefix}finger_{side}_joint", "type": "prismatic"})
        _origin(finger, f"0 {joint_y} 0.058")
        ET.SubElement(finger, "parent").set("link", f"{prefix}gripper")
        ET.SubElement(finger, "child").set("link", f"{prefix}finger_{side}")
        ET.SubElement(finger, "axis").set("xyz", axis)
        limit = ET.SubElement(finger, "limit")
        limit.set("lower", "0")
        limit.set("upper", "0.038")
        limit.set("effort", "40")
        limit.set("velocity", "0.15")
        parts.extend([link, finger])
    return parts


def inject_aubo(robot_xml: str, refined: bool = True) -> str:
    root = ET.fromstring(robot_xml)
    for prefix, mount in (("left_", "left_arm_mount"), ("right_", "right_arm_mount")):
        for elem in _arm(prefix, mount):
            root.append(elem)
    if refined:
        from .refined_assembly import refine_assembly
        root = refine_assembly(root)
    return ET.tostring(root, encoding="unicode")
