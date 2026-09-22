#!/usr/bin/env python3
"""Generate the glTF files the loader is tested against.

models/test_arm.glb
    A small four-axis arm built to exercise the loader's mapping: a tree of
    nodes, a node rotation given as a quaternion, principal and non-principal
    joint axes, a default angle, one mesh shared by two nodes, one primitive
    without normals, 16-bit indices and base colour materials. It doubles as a
    demo model:  pixi run viewer --model models/test_arm.glb

tests/gltf_load/edge_cases.gltf
    Articulation data the loader has to repair or refuse: swapped limits, a
    zero axis, a default outside its limits, an unsupported joint type, and a
    string value that spells a key name.

tests/gltf_load/no_geometry.gltf
    Nodes but nothing to draw, which the loader must refuse.

The output is deterministic, so running this again leaves git clean.
Standard library only.
"""

import base64
import json
import math
import pathlib
import struct

ROOT = pathlib.Path(__file__).resolve().parent.parent

GL_FLOAT = 5126
GL_UNSIGNED_SHORT = 5123
GL_ARRAY_BUFFER = 34962
GL_ELEMENT_ARRAY_BUFFER = 34963


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def box(size_x, size_y, size_z, y0=0.0):
    """A box standing on y = y0, four vertices per face so each face keeps its
    own normal. Returns positions, normals and triangle indices, wound
    counter-clockwise seen from outside."""
    x = (-size_x / 2, size_x / 2)
    y = (y0, y0 + size_y)
    z = (-size_z / 2, size_z / 2)
    faces = [
        ((1, 0, 0),  [(x[1], y[0], z[0]), (x[1], y[1], z[0]), (x[1], y[1], z[1]), (x[1], y[0], z[1])]),
        ((-1, 0, 0), [(x[0], y[0], z[0]), (x[0], y[1], z[0]), (x[0], y[1], z[1]), (x[0], y[0], z[1])]),
        ((0, 1, 0),  [(x[0], y[1], z[0]), (x[1], y[1], z[0]), (x[1], y[1], z[1]), (x[0], y[1], z[1])]),
        ((0, -1, 0), [(x[0], y[0], z[0]), (x[1], y[0], z[0]), (x[1], y[0], z[1]), (x[0], y[0], z[1])]),
        ((0, 0, 1),  [(x[0], y[0], z[1]), (x[1], y[0], z[1]), (x[1], y[1], z[1]), (x[0], y[1], z[1])]),
        ((0, 0, -1), [(x[0], y[0], z[0]), (x[1], y[0], z[0]), (x[1], y[1], z[0]), (x[0], y[1], z[0])]),
    ]
    positions, normals, indices = [], [], []
    for normal, quad in faces:
        # Flip the quad if its winding disagrees with the outward normal.
        if _dot(_cross(_sub(quad[1], quad[0]), _sub(quad[2], quad[0])), normal) < 0:
            quad = quad[::-1]
        base = len(positions)
        positions += quad
        normals += [normal] * 4
        indices += [base, base + 1, base + 2, base, base + 2, base + 3]
    return positions, normals, indices


class GltfBuilder:
    """Accumulates one binary buffer and the JSON that describes it."""

    def __init__(self, generator):
        self.bin = bytearray()
        self.doc = {
            "asset": {"version": "2.0", "generator": generator},
            "scene": 0,
            "scenes": [{"nodes": []}],
            "nodes": [],
            "meshes": [],
            "materials": [],
            "accessors": [],
            "bufferViews": [],
        }

    def _view(self, data, target):
        while len(self.bin) % 4:
            self.bin.append(0)
        offset = len(self.bin)
        self.bin += data
        self.doc["bufferViews"].append(
            {"buffer": 0, "byteOffset": offset, "byteLength": len(data), "target": target})
        return len(self.doc["bufferViews"]) - 1

    def _accessor(self, view, component_type, count, kind, **extra):
        accessor = {"bufferView": view, "componentType": component_type, "count": count, "type": kind}
        accessor.update(extra)
        self.doc["accessors"].append(accessor)
        return len(self.doc["accessors"]) - 1

    def material(self, name, linear_rgb):
        self.doc["materials"].append({
            "name": name,
            "pbrMetallicRoughness": {
                "baseColorFactor": [*linear_rgb, 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 0.8,
            },
        })
        return len(self.doc["materials"]) - 1

    def mesh(self, name, positions, normals, indices, material):
        """normals may be None, which the loader has to cope with."""
        packed = b"".join(struct.pack("<3f", *p) for p in positions)
        lo = [min(p[i] for p in positions) for i in range(3)]
        hi = [max(p[i] for p in positions) for i in range(3)]
        attributes = {"POSITION": self._accessor(self._view(packed, GL_ARRAY_BUFFER), GL_FLOAT,
                                                 len(positions), "VEC3", min=lo, max=hi)}
        if normals is not None:
            packed = b"".join(struct.pack("<3f", *n) for n in normals)
            attributes["NORMAL"] = self._accessor(self._view(packed, GL_ARRAY_BUFFER), GL_FLOAT,
                                                  len(normals), "VEC3")
        index_view = self._view(struct.pack(f"<{len(indices)}H", *indices), GL_ELEMENT_ARRAY_BUFFER)
        self.doc["meshes"].append({
            "name": name,
            "primitives": [{
                "attributes": attributes,
                "indices": self._accessor(index_view, GL_UNSIGNED_SHORT, len(indices), "SCALAR"),
                "material": material,
            }],
        })
        return len(self.doc["meshes"]) - 1

    def node(self, name, parent=None, translation=None, rotation=None, mesh=None, extras=None):
        node = {"name": name}
        if translation is not None:
            node["translation"] = list(translation)
        if rotation is not None:
            node["rotation"] = list(rotation)
        if mesh is not None:
            node["mesh"] = mesh
        if extras is not None:
            node["extras"] = extras
        self.doc["nodes"].append(node)
        index = len(self.doc["nodes"]) - 1
        if parent is None:
            self.doc["scenes"][0]["nodes"].append(index)
        else:
            self.doc["nodes"][parent].setdefault("children", []).append(index)
        return index

    def glb(self):
        self.doc["buffers"] = [{"byteLength": len(self.bin)}]
        json_chunk = json.dumps(self.doc, separators=(",", ":")).encode()
        json_chunk += b" " * (-len(json_chunk) % 4)
        bin_chunk = bytes(self.bin) + b"\0" * (-len(self.bin) % 4)
        total = 12 + 8 + len(json_chunk) + 8 + len(bin_chunk)
        return (struct.pack("<4sII", b"glTF", 2, total)
                + struct.pack("<I4s", len(json_chunk), b"JSON") + json_chunk
                + struct.pack("<I4s", len(bin_chunk), b"BIN\0") + bin_chunk)

    def gltf_embedded(self):
        if self.bin:
            self.doc["buffers"] = [{
                "byteLength": len(self.bin),
                "uri": "data:application/octet-stream;base64," + base64.b64encode(bytes(self.bin)).decode(),
            }]
        else:
            for key in ("accessors", "bufferViews", "meshes", "materials"):
                if not self.doc[key]:
                    del self.doc[key]
        return json.dumps(self.doc, indent=2) + "\n"


def revolute(axis, min_deg=None, max_deg=None, default_deg=None):
    extras = {"rbt_joint": "revolute", "rbt_axis": list(axis)}
    if min_deg is not None:
        extras["rbt_min_deg"] = min_deg
    if max_deg is not None:
        extras["rbt_max_deg"] = max_deg
    if default_deg is not None:
        extras["rbt_default_deg"] = default_deg
    return extras


def make_test_arm():
    g = GltfBuilder("robot_opengl_imgui tools/make-test-models.py")

    dark = g.material("pedestal", (0.05, 0.05, 0.06))
    orange = g.material("turntable", (0.80, 0.20, 0.02))
    blue = g.material("upper arm", (0.03, 0.20, 0.80))
    green = g.material("forearm", (0.03, 0.45, 0.10))
    yellow = g.material("wrist", (0.70, 0.55, 0.02))
    grey = g.material("finger", (0.60, 0.60, 0.60))

    base_mesh = g.mesh("pedestal", *box(0.36, 0.10, 0.36), dark)
    turntable_mesh = g.mesh("turntable", *box(0.24, 0.15, 0.24), orange)
    upper_mesh = g.mesh("upper arm", *box(0.10, 0.50, 0.10), blue)
    positions, _, indices = box(0.08, 0.40, 0.08)
    forearm_mesh = g.mesh("forearm, no normals", positions, None, indices, green)
    wrist_mesh = g.mesh("wrist", *box(0.07, 0.08, 0.07), yellow)
    finger_mesh = g.mesh("finger, shared", *box(0.015, 0.07, 0.03), grey)

    # 90 degrees about +Y, as glTF writes it: x, y, z, w.
    quarter_turn_y = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))

    base = g.node("base", mesh=base_mesh)
    turntable = g.node("turntable", base, translation=(0, 0.10, 0), mesh=turntable_mesh,
                       extras=revolute((0, 1, 0), -170, 170))
    shoulder = g.node("shoulder", turntable, translation=(0, 0.15, 0), mesh=upper_mesh,
                      extras=revolute((0, 0, 1), -90, 120, 30))
    elbow = g.node("elbow", shoulder, translation=(0, 0.50, 0), rotation=quarter_turn_y,
                   mesh=forearm_mesh, extras=revolute((1, 0, 1), -180, 180))
    wrist = g.node("wrist", elbow, translation=(0, 0.40, 0), mesh=wrist_mesh,
                   extras=revolute((-1, 0, 0), -120, 120))
    tool = g.node("tool", wrist, translation=(0, 0.08, 0))
    g.node("finger_left", tool, translation=(-0.025, 0, 0), mesh=finger_mesh)
    g.node("finger_right", tool, translation=(0.025, 0, 0), mesh=finger_mesh)
    return g.glb()


def make_edge_cases():
    g = GltfBuilder("robot_opengl_imgui tools/make-test-models.py, edge cases")
    triangle = g.mesh("triangle", [(0, 0, 0), (1, 0, 0), (0, 1, 0)], [(0, 0, 1)] * 3, [0, 1, 2],
                      g.material("plain", (0.5, 0.5, 0.5)))

    root = g.node("root", mesh=triangle)
    g.node("swapped", root, extras={"rbt_joint": "revolute", "rbt_min_deg": 90, "rbt_max_deg": -90})
    g.node("zero_axis", root, extras={"rbt_joint": "revolute", "rbt_axis": [0, 0, 0]})
    g.node("clamped_default", root, extras={"rbt_joint": "revolute", "rbt_min_deg": -10,
                                            "rbt_max_deg": 10, "rbt_default_deg": 45})
    g.node("prismatic", root, extras={"rbt_joint": "prismatic"})
    # The first "rbt_axis" in the text is a value, not a key.
    g.node("decoy", root, extras={"label": "rbt_axis", "rbt_joint": "revolute", "rbt_axis": [1, 0, 0]})
    g.node("defaults", root, extras={"rbt_joint": "revolute"})
    g.node("no_extras", root)
    return g.gltf_embedded()


def make_no_geometry():
    g = GltfBuilder("robot_opengl_imgui tools/make-test-models.py, no geometry")
    root = g.node("root")
    g.node("child", root, translation=(0, 1, 0))
    return g.gltf_embedded()


def write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = "wb" if isinstance(data, bytes) else "w"
    with open(path, mode) as f:
        f.write(data)
    print(f"wrote {path.relative_to(ROOT)} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    write(ROOT / "models" / "test_arm.glb", make_test_arm())
    write(ROOT / "tests" / "gltf_load" / "edge_cases.gltf", make_edge_cases())
    write(ROOT / "tests" / "gltf_load" / "no_geometry.gltf", make_no_geometry())
