#!/usr/bin/env python3
"""Generate the six display robot forms from the exported full URDF."""

from copy import deepcopy
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np


URDF_DIR = Path(__file__).resolve().parents[1] / "urdf"
SOURCE = URDF_DIR / "std_auto.urdf"
MESH_DIR = URDF_DIR.parent / "meshes"


def name(element):
    return element.attrib["name"]


def joint_links(joint):
    return joint.find("parent").attrib["link"], joint.find("child").attrib["link"]


def mesh_link(link_name, mesh_name):
    link = ET.Element("link", {"name": link_name})
    for element_name in ("visual", "collision"):
        element = ET.SubElement(link, element_name)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(
            geometry,
            "mesh",
            {"filename": f"package://bw_std_description/meshes/{mesh_name}"},
        )
        if element_name == "visual":
            material = ET.SubElement(element, "material", {"name": ""})
            ET.SubElement(material, "color", {"rgba": "1 1 1 1"})
    return link


def fixed_joint(joint_name, parent, child):
    joint = ET.Element("joint", {"name": joint_name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def add_v2_chest_cap(robot):
    cap = ET.Element("link", {"name": "chest_top_cap"})
    for element_name in ("visual", "collision"):
        element = ET.SubElement(cap, element_name)
        ET.SubElement(element, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(
            geometry,
            "mesh",
            {"filename": "package://bw_std_description/meshes/chest_top_cap.STL"},
        )
        if element_name == "visual":
            material = ET.SubElement(element, "material", {"name": ""})
            ET.SubElement(
                material,
                "color",
                {"rgba": "0.752941176470588 0.752941176470588 0.752941176470588 1"},
            )
    robot.append(cap)

    joint = fixed_joint("chest_top_cap_joint", "C_Link", "chest_top_cap")
    joint.insert(0, ET.Element("origin", {"xyz": "0.092 0 0.0746", "rpy": "0 0 0"}))
    robot.append(joint)


def write_variant(
    filename,
    included_links,
    reparent_c_joint=False,
    chassis_only=False,
    base_mesh="base_chassis.STL",
    show_slider=False,
    slider_mesh="linear_slide.STL",
    hide_chest=False,
):
    source_root = ET.parse(SOURCE).getroot()
    robot = ET.Element("robot", {"name": Path(filename).stem})

    for element in source_root:
        if element.tag == "link" and name(element) in included_links:
            if name(element) == "base_link" and chassis_only:
                robot.append(mesh_link("base_link", base_mesh))
            elif name(element) == "C_Link" and hide_chest:
                robot.append(ET.Element("link", {"name": "C_Link"}))
            else:
                robot.append(deepcopy(element))
        elif element.tag == "joint":
            parent, child = joint_links(element)
            if parent in included_links and child in included_links:
                robot.append(deepcopy(element))
            elif reparent_c_joint and name(element) == "C_joint":
                joint = deepcopy(element)
                joint.find("parent").attrib["link"] = "base_footprint"
                robot.append(joint)

    if show_slider:
        robot.append(mesh_link("linear_slide_link", slider_mesh))
        slider_parent = "base_link" if "base_link" in included_links else "base_footprint"
        robot.append(fixed_joint("linear_slide_joint", slider_parent, "linear_slide_link"))

    if filename == "std_v2.urdf":
        add_v2_chest_cap(robot)

    variant = Path(filename).stem.removeprefix("std_")
    namespace_robot_meshes(robot, variant)

    ET.indent(robot, space="  ")
    output = URDF_DIR / filename
    ET.ElementTree(robot).write(output, encoding="utf-8", xml_declaration=True)
    with output.open("a", encoding="utf-8") as stream:
        stream.write("\n")


def namespace_robot_meshes(robot, variant):
    """Give every variant private mesh paths so equal filenames cannot collide."""
    target_dir = MESH_DIR / variant
    target_dir.mkdir(parents=True, exist_ok=True)
    prefix = "package://bw_std_description/meshes/"
    for mesh in robot.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        if not uri.startswith(prefix):
            continue
        relative = uri.removeprefix(prefix)
        source = MESH_DIR / relative
        target = target_dir / Path(relative).name
        if source.resolve() != target.resolve():
            shutil.copy2(source, target)
        mesh.attrib["filename"] = f"{prefix}{variant}/{target.name}"


def write_v4_subset(filename, included_links, reparent_arm=False):
    """Create arm-only and chassis-only forms from the imported v4 model."""
    source_root = ET.parse(URDF_DIR / "std_v4.urdf").getroot()
    robot = ET.Element("robot", {"name": Path(filename).stem})
    for element in source_root:
        if element.tag == "link" and name(element) in included_links:
            link = deepcopy(element)
            if filename == "std_v6.urdf" and name(link) == "base_link":
                for mesh in link.findall(".//mesh"):
                    mesh.attrib["filename"] = (
                        "package://bw_std_description/meshes/v4_base_flat_top.STL"
                    )
            robot.append(link)
        elif element.tag == "joint":
            parent, child = joint_links(element)
            if parent in included_links and child in included_links:
                robot.append(deepcopy(element))
            elif reparent_arm and name(element) == "A_right_Degree1_joint":
                joint = deepcopy(element)
                joint.find("parent").attrib["link"] = "base_footprint"
                robot.append(joint)

    if filename == "std_v5.urdf":
        robot.append(mesh_link("arm_base_plate_link", "v5_solid_base.STL"))
        robot.append(
            fixed_joint("arm_base_plate_joint", "base_footprint", "arm_base_plate_link")
        )
    variant = Path(filename).stem.removeprefix("std_")
    namespace_robot_meshes(robot, variant)
    ET.indent(robot, space="  ")
    output = URDF_DIR / filename
    ET.ElementTree(robot).write(output, encoding="utf-8", xml_declaration=True)
    with output.open("a", encoding="utf-8") as stream:
        stream.write("\n")


def write_v4_subsets():
    source_root = ET.parse(URDF_DIR / "std_v4.urdf").getroot()
    all_links = {name(link) for link in source_root.findall("link")}
    arm_links = {link for link in all_links if link.startswith("A_right_")}
    write_v4_subset(
        "std_v5.urdf",
        {"base_footprint"} | arm_links,
        reparent_arm=True,
    )
    chassis_links = {
        "base_footprint",
        "base_link",
        "D_left_link",
        "D_right_link",
        "D_behind_link",
    }
    write_v4_subset("std_v6.urdf", chassis_links)


def split_base_mesh():
    """Split the tall linear slide components from the combined base STL."""
    source = MESH_DIR / "base_link.STL"
    raw = source.read_bytes()
    triangle_count = struct.unpack_from("<I", raw, 80)[0]
    triangle_dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    triangles = np.frombuffer(raw, dtype=triangle_dtype, count=triangle_count, offset=84)

    vertices = triangles["vertices"].reshape(-1, 3)
    quantized = np.round(vertices * 1_000_000).astype(np.int64)
    _, inverse = np.unique(quantized, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    parent = np.arange(triangle_count)

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(first, second):
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    order = np.argsort(faces.ravel())
    sorted_vertices = faces.ravel()[order]
    sorted_faces = np.repeat(np.arange(triangle_count), 3)[order]
    boundaries = np.r_[0, np.flatnonzero(np.diff(sorted_vertices)) + 1, len(order)]
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        for adjacent_face in sorted_faces[start + 1 : end]:
            union(sorted_faces[start], adjacent_face)

    roots = np.array([find(index) for index in range(triangle_count)])
    slide_roots = set()
    for root in np.unique(roots):
        component = triangles["vertices"][roots == root]
        if component[..., 2].max() > 0.8:
            slide_roots.add(root)

    slide_triangles = triangles[np.isin(roots, list(slide_roots))]
    write_stl(MESH_DIR / "linear_slide.STL", slide_triangles)
    # V2 is mounted without the chassis, so omit the four lower mounting corners.
    v2_slide = slide_triangles[slide_triangles["vertices"].mean(axis=1)[:, 2] >= 0.15]
    write_stl(MESH_DIR / "linear_slide_v2.STL", v2_slide)
    centers_z = slide_triangles["vertices"].mean(axis=1)[:, 2]
    v2_base_plate = slide_triangles[(centers_z >= 0.15) & (centers_z < 0.205)]
    write_stl(MESH_DIR / "v2_base_plate.STL", v2_base_plate)
    write_stl(MESH_DIR / "base_chassis.STL", triangles[~np.isin(roots, list(slide_roots))])
    component_roots, component_counts = np.unique(roots, return_counts=True)
    outer_body_root = component_roots[np.argmax(component_counts)]
    outer_body = clip_mesh_above_height(triangles[roots == outer_body_root], 0.205)
    write_stl(MESH_DIR / "base_body_no_lidar.STL", outer_body)


def write_v3_base_chassis_mesh():
    """Remove v3's complete slide/mount component from its combined base mesh."""
    source = MESH_DIR / "v3" / "base_link.STL"
    raw = source.read_bytes()
    triangle_count = struct.unpack_from("<I", raw, 80)[0]
    triangle_dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    triangles = np.frombuffer(raw, dtype=triangle_dtype, count=triangle_count, offset=84)

    vertices = triangles["vertices"].reshape(-1, 3)
    quantized = np.round(vertices * 1_000_000).astype(np.int64)
    _, inverse = np.unique(quantized, axis=0, return_inverse=True)
    faces = inverse.reshape(-1, 3)
    parent = np.arange(triangle_count)

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(first, second):
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    order = np.argsort(faces.ravel())
    sorted_vertices = faces.ravel()[order]
    sorted_faces = np.repeat(np.arange(triangle_count), 3)[order]
    boundaries = np.r_[0, np.flatnonzero(np.diff(sorted_vertices)) + 1, len(order)]
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        for adjacent_face in sorted_faces[start + 1 : end]:
            union(sorted_faces[start], adjacent_face)

    roots = np.array([find(index) for index in range(triangle_count)])
    slide_roots = {
        root
        for root in np.unique(roots)
        if triangles["vertices"][roots == root][..., 2].max() > 0.8
    }
    write_stl(
        MESH_DIR / "v3_base_chassis.STL",
        triangles[~np.isin(roots, list(slide_roots))],
    )


def write_stl(output, triangles):
    header = b"Standard robot generated mesh".ljust(80, b" ")
    output.write_bytes(header + struct.pack("<I", len(triangles)) + triangles.tobytes())


def box_faces(minimum, maximum):
    x0, y0, z0 = minimum
    x1, y1, z1 = maximum
    vertices = [
        [x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1],
    ]
    indexes = (
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
        (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7),
    )
    return [(vertices[a], vertices[b], vertices[c]) for a, b, c in indexes]


def write_v5_solid_base_mesh():
    """Create a hole-free plate and a pedestal touching the arm's lower face."""
    faces = box_faces((-0.233, -0.1105, 0.154375), (-0.005, 0.1105, 0.18))
    faces += box_faces((-0.144, -0.025, 0.18), (-0.094, 0.025, 0.208))
    write_stl(MESH_DIR / "v5_solid_base.STL", triangles_from_faces(faces))


def write_v4_flat_top_base():
    """Cut v4's complete upper chassis to one plane and close its outer boundary."""
    source = MESH_DIR / "v4" / "base_link.STL"
    raw = source.read_bytes()
    triangle_count = struct.unpack_from("<I", raw, 80)[0]
    triangle_dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    triangles = np.frombuffer(raw, dtype=triangle_dtype, count=triangle_count, offset=84)
    write_stl(MESH_DIR / "v4_base_flat_top.STL", flatten_mesh_top(triangles, 0.175))


def flatten_mesh_top(triangles, height):
    """Clip a mesh at height and close the cut with one solid, horizontal face."""
    vertices = triangles["vertices"]
    entirely_below = vertices[..., 2].max(axis=1) <= height
    entirely_above = vertices[..., 2].min(axis=1) > height
    crossing = triangles[~(entirely_below | entirely_above)]
    faces = []
    cut_points = []
    for triangle in crossing["vertices"]:
        clipped = []
        intersections = []
        for index, current in enumerate(triangle):
            following = triangle[(index + 1) % len(triangle)]
            current_inside = current[2] <= height
            following_inside = following[2] <= height
            if current_inside:
                clipped.append(current)
            if current_inside != following_inside:
                ratio = (height - current[2]) / (following[2] - current[2])
                intersection = current + ratio * (following - current)
                clipped.append(intersection)
                intersections.append(intersection[:2])
        if len(intersections) == 2:
            cut_points.extend(intersections)
        for index in range(1, len(clipped) - 1):
            faces.append((clipped[0], clipped[index], clipped[index + 1]))

    points = sorted(set(map(tuple, np.round(cut_points, 6))))

    def cross(origin, first, second):
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    lower = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    center = [
        np.mean([point[0] for point in hull]),
        np.mean([point[1] for point in hull]),
        height,
    ]
    for index, point in enumerate(hull):
        following = hull[(index + 1) % len(hull)]
        faces.append(
            (center, [point[0], point[1], height], [following[0], following[1], height])
        )

    generated = triangles_from_faces(faces)
    return np.concatenate((triangles[entirely_below], generated))


def triangles_from_faces(faces):
    triangle_dtype = np.dtype(
        [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    triangles = np.zeros(len(faces), dtype=triangle_dtype)
    triangles["vertices"] = np.asarray(faces, dtype=np.float32)
    first = triangles["vertices"][:, 1] - triangles["vertices"][:, 0]
    second = triangles["vertices"][:, 2] - triangles["vertices"][:, 0]
    normals = np.cross(first, second)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    triangles["normal"] = normals / np.maximum(lengths, 1e-12)
    return triangles


def clip_mesh_above_height(triangles, height):
    """Remove the integrated tower above the chassis and close the cut face."""
    faces = []
    cut_points = []
    for triangle in triangles["vertices"]:
        clipped = []
        for index, current in enumerate(triangle):
            following = triangle[(index + 1) % len(triangle)]
            current_inside = current[2] <= height
            following_inside = following[2] <= height
            if current_inside:
                clipped.append(current)
            if current_inside != following_inside:
                ratio = (height - current[2]) / (following[2] - current[2])
                intersection = current + ratio * (following - current)
                clipped.append(intersection)
                cut_points.append(intersection[:2])
        for index in range(1, len(clipped) - 1):
            faces.append((clipped[0], clipped[index], clipped[index + 1]))

    points = sorted(set(map(tuple, np.round(cut_points, 6))))
    if points:
        def cross(origin, first, second):
            return (first[0] - origin[0]) * (second[1] - origin[1]) - (
                first[1] - origin[1]
            ) * (second[0] - origin[0])

        lower = []
        for point in points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
                lower.pop()
            lower.append(point)
        upper = []
        for point in reversed(points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
                upper.pop()
            upper.append(point)
        hull = lower[:-1] + upper[:-1]
        center = [
            np.mean([point[0] for point in hull]),
            np.mean([point[1] for point in hull]),
            height,
        ]
        for index, point in enumerate(hull):
            following = hull[(index + 1) % len(hull)]
            faces.append(
                (center, [point[0], point[1], height], [following[0], following[1], height])
            )
    return triangles_from_faces(faces)


def write_chest_cap_mesh():
    """Create the rounded-square cover matching the 80 mm chest opening."""
    # The plug overlaps the 80 mm opening by 1 mm on every side and extends
    # into the cavity, preventing a visible gap along the curved chest rim.
    width, depth, radius, thickness = 0.082, 0.082, 0.010, 0.040
    points = []
    for center_x, center_y, start_angle in (
        (width / 2 - radius, depth / 2 - radius, 0),
        (-width / 2 + radius, depth / 2 - radius, 90),
        (-width / 2 + radius, -depth / 2 + radius, 180),
        (width / 2 - radius, -depth / 2 + radius, 270),
    ):
        for angle in np.linspace(start_angle, start_angle + 90, 9, endpoint=False):
            radians = np.deg2rad(angle)
            points.append(
                [center_x + radius * np.cos(radians), center_y + radius * np.sin(radians)]
            )
    points = np.asarray(points, dtype=np.float32)
    bottom_z, top_z = -thickness / 2, thickness / 2
    faces = []
    for index in range(len(points)):
        following = (index + 1) % len(points)
        bottom = [points[index, 0], points[index, 1], bottom_z]
        bottom_next = [points[following, 0], points[following, 1], bottom_z]
        top = [points[index, 0], points[index, 1], top_z]
        top_next = [points[following, 0], points[following, 1], top_z]
        faces.extend(
            (
                ([0, 0, bottom_z], bottom_next, bottom),
                ([0, 0, top_z], top, top_next),
                (bottom, bottom_next, top_next),
                (bottom, top_next, top),
            )
        )

    write_stl(MESH_DIR / "chest_top_cap.STL", triangles_from_faces(faces))


def main():
    split_base_mesh()
    write_v3_base_chassis_mesh()
    write_chest_cap_mesh()
    write_v5_solid_base_mesh()
    write_v4_flat_top_base()
    root = ET.parse(SOURCE).getroot()
    all_links = {name(link) for link in root.findall("link")}
    chest = {"C_Link"}
    arms = {link for link in all_links if link.startswith("A_")}

    write_variant("std_v1.urdf", all_links)
    write_variant(
        "std_v2.urdf",
        {"base_footprint"} | chest | arms,
        reparent_c_joint=True,
        show_slider=True,
        slider_mesh="linear_slide_v2.STL",
    )
    write_v4_subsets()
    v2_root = ET.parse(URDF_DIR / "std_v2.urdf").getroot()
    forbidden_v2_links = {
        "base_link",
        "D_left_Link",
        "D_right_Link",
        "D_behind_Link",
    }
    actual_v2_links = {name(link) for link in v2_root.findall("link")}
    assert not forbidden_v2_links & actual_v2_links, "v2 must not contain chassis links"
    assert not any(
        mesh.attrib["filename"].endswith(("base_link.STL", "base_chassis.STL"))
        for mesh in v2_root.findall(".//mesh")
    ), "v2 must not contain a chassis mesh"


if __name__ == "__main__":
    main()
