#!/usr/bin/env python3
"""Build the Sydney coastline mesh and matching Nav2 occupancy map."""

import argparse
import os
import xml.etree.ElementTree as ET

MODEL_SCALE = 0.15
EXIT_RECTS = (
    (-205.0, -126.0, -10.0, 10.0),
    (126.0, 205.0, -10.0, 10.0),
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", required=True)
    parser.add_argument("--target-dir", required=True)
    return parser.parse_args()


def namespace(root):
    uri = root.tag.split("}", 1)[0].lstrip("{")
    ET.register_namespace("", uri)
    return uri


def tag(ns, name):
    return f"{{{ns}}}{name}"


def parse_matrix(node, ns):
    matrix_node = node.find(tag(ns, "matrix"))
    if matrix_node is None:
        return (
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
    values = [float(value) for value in matrix_node.text.split()]
    return tuple(tuple(values[row * 4 + col] for col in range(4)) for row in range(4))


def transform_point(point, matrix, unit_scale):
    vector = (point[0], point[1], point[2], 1.0)
    transformed = [
        sum(matrix[row][col] * vector[col] for col in range(4))
        for row in range(3)
    ]
    return (
        transformed[0] * unit_scale * MODEL_SCALE,
        transformed[1] * unit_scale * MODEL_SCALE,
        transformed[2] * unit_scale * MODEL_SCALE,
    )


def geometry_transforms(root, ns):
    transforms = {}
    for node in root.findall(f".//{tag(ns, 'library_visual_scenes')}//{tag(ns, 'node')}"):
        instance = node.find(tag(ns, "instance_geometry"))
        if instance is None:
            continue
        transforms[instance.get("url").lstrip("#")] = parse_matrix(node, ns)
    return transforms


def position_data(mesh, ns):
    vertices = {
        node.get("id"): node for node in mesh.findall(tag(ns, "vertices"))
    }
    sources = {
        node.get("id"): node for node in mesh.findall(tag(ns, "source"))
    }
    result = {}
    for vertices_id, vertices_node in vertices.items():
        position_input = next(
            node
            for node in vertices_node.findall(tag(ns, "input"))
            if node.get("semantic") == "POSITION"
        )
        source = sources[position_input.get("source").lstrip("#")]
        float_array = source.find(tag(ns, "float_array"))
        accessor = source.find(f".//{tag(ns, 'accessor')}")
        stride = int(accessor.get("stride", "3"))
        values = [float(value) for value in float_array.text.split()]
        result[vertices_id] = [
            tuple(values[index:index + 3])
            for index in range(0, len(values), stride)
        ]
    return result


def point_in_rect(point, rect):
    return rect[0] <= point[0] <= rect[1] and rect[2] <= point[1] <= rect[3]


def orientation(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a, b, c, d):
    return (
        orientation(a, b, c) * orientation(a, b, d) <= 0.0
        and orientation(c, d, a) * orientation(c, d, b) <= 0.0
        and max(min(a[0], b[0]), min(c[0], d[0]))
        <= min(max(a[0], b[0]), max(c[0], d[0]))
        and max(min(a[1], b[1]), min(c[1], d[1]))
        <= min(max(a[1], b[1]), max(c[1], d[1]))
    )


def point_in_triangle(point, triangle):
    signs = [
        orientation(triangle[index], triangle[(index + 1) % 3], point)
        for index in range(3)
    ]
    return not (any(value < 0.0 for value in signs) and any(value > 0.0 for value in signs))


def triangle_intersects_rect(triangle, rect):
    if any(point_in_rect(point, rect) for point in triangle):
        return True
    corners = (
        (rect[0], rect[2]),
        (rect[1], rect[2]),
        (rect[1], rect[3]),
        (rect[0], rect[3]),
    )
    if any(point_in_triangle(corner, triangle) for corner in corners):
        return True
    triangle_edges = [
        (triangle[index], triangle[(index + 1) % 3])
        for index in range(3)
    ]
    rectangle_edges = [
        (corners[index], corners[(index + 1) % 4])
        for index in range(4)
    ]
    return any(
        segments_intersect(*triangle_edge, *rectangle_edge)
        for triangle_edge in triangle_edges
        for rectangle_edge in rectangle_edges
    )


def carve_mesh(source_path, target_path):
    tree = ET.parse(source_path)
    root = tree.getroot()
    ns = namespace(root)
    unit_node = root.find(f".//{tag(ns, 'asset')}/{tag(ns, 'unit')}")
    unit_scale = float(unit_node.get("meter", "1.0")) if unit_node is not None else 1.0
    transforms = geometry_transforms(root, ns)
    removed = 0

    for geometry in root.findall(f".//{tag(ns, 'library_geometries')}/{tag(ns, 'geometry')}"):
        geometry_id = geometry.get("id")
        mesh = geometry.find(tag(ns, "mesh"))
        positions_by_vertices = position_data(mesh, ns)
        matrix = transforms.get(geometry_id, parse_matrix(geometry, ns))

        for triangles in mesh.findall(tag(ns, "triangles")):
            inputs = triangles.findall(tag(ns, "input"))
            stride = max(int(node.get("offset", "0")) for node in inputs) + 1
            vertex_input = next(
                node for node in inputs if node.get("semantic") == "VERTEX"
            )
            vertex_offset = int(vertex_input.get("offset", "0"))
            vertices_id = vertex_input.get("source").lstrip("#")
            positions = positions_by_vertices[vertices_id]
            index_node = triangles.find(tag(ns, "p"))
            indices = [int(value) for value in index_node.text.split()]
            face_stride = stride * 3
            kept = []

            for start in range(0, len(indices), face_stride):
                face = indices[start:start + face_stride]
                vertex_indices = [
                    face[corner * stride + vertex_offset]
                    for corner in range(3)
                ]
                local_triangle = [
                    transform_point(positions[index], matrix, unit_scale)[:2]
                    for index in vertex_indices
                ]
                if any(
                    triangle_intersects_rect(local_triangle, rect)
                    for rect in EXIT_RECTS
                ):
                    removed += 1
                    continue
                kept.extend(face)

            triangles.set("count", str(len(kept) // face_stride))
            index_node.text = " ".join(str(value) for value in kept)

    os.makedirs(os.path.dirname(target_path), exist_ok=True)
    tree.write(target_path, encoding="UTF-8", xml_declaration=True)
    return removed


def main():
    args = parse_args()
    source_meshes = os.path.join(args.source_dir, "meshes")
    target_meshes = os.path.join(args.target_dir, "meshes")
    os.makedirs(target_meshes, exist_ok=True)

    visual_removed = carve_mesh(
        os.path.join(source_meshes, "sydney_regatta.dae"),
        os.path.join(target_meshes, "sydney_regatta.dae"),
    )
    collision_removed = carve_mesh(
        os.path.join(source_meshes, "sydney_regatta_shore.dae"),
        os.path.join(target_meshes, "sydney_regatta_shore.dae"),
    )
    print(
        f"Carved two coastline exits: {visual_removed} visual triangles and "
        f"{collision_removed} collision triangles removed."
    )


if __name__ == "__main__":
    main()
