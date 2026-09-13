#!/usr/bin/env -S blender --background --python
# ruff: noqa: D100, D103
import math
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import bmesh
import bpy
import mathutils

# BAD. HACK. EVIL. BAD.
sys.path.append(str(Path(__file__).parent.resolve()))

from aruco_tags import get_aruco_tags

aruco_tags = get_aruco_tags("4x4")

# I originally determined these values in mm, and blender uses m
marker_depth = 5 / 1000
marker_size = 150 / 1000
marker_border = 50 / 1000
plate_thickness = 10 / 1000

post_height = 1
post_radius = 25 / 1000
post_base_extra = 5 / 1000

plate_position = 0.85

base_thickness = 10 / 1000
base_radius = 120 / 1000

cells = len(aruco_tags[0])

cell_size = marker_size / (cells + 2)

id_range = range(9)  # tags 0-8


def make_material(name: str, color: tuple[float, float, float]) -> bpy.types.Material:
    mat: bpy.types.Material = bpy.data.materials.new(name)
    mat.diffuse_color = (color[0], color[1], color[2], 0.0)
    # what does this do? dunno, but it seems to work.
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (color[0], color[1], color[2], 0.0)
    return mat


white = make_material("White", (1, 1, 1))
black = make_material("Black", (0, 0, 0))
gray = make_material("Gray", (0.5, 0.5, 0.5))
darkgray = make_material("Dark Gray", (0.2, 0.2, 0.2))


def get_active() -> Any:
    return bpy.context.active_object


def filter_none_args(**kwargs: object) -> dict[str, Any]:
    return {k: v for k, v in kwargs.items() if v is not None}


def add_cube(
        size: float | None = 1.0,
        location: Sequence[float] | mathutils.Vector = (0.0, 0.0, 0.0),
        rotation: Sequence[float] | mathutils.Euler = (0.0, 0.0, 0.0),
        scale: Sequence[float] | mathutils.Vector = (0.0, 0.0, 0.0),
        material: bpy.types.Material | None = None,
) -> Any:
    bpy.ops.mesh.primitive_cube_add(**filter_none_args(size=size, location=location, rotation=rotation, scale=scale))

    cube = get_active()

    if material is not None:
        cube.data.materials.append(material)

    return cube


def add_plane(
        size: float | None = 1.0,
        location: Sequence[float] | mathutils.Vector = None,
        rotation: Sequence[float] | mathutils.Euler = None,
        scale: Sequence[float] | mathutils.Vector = None,
) -> Any:
    bpy.ops.mesh.primitive_plane_add(**filter_none_args(size=size, location=location, rotation=rotation, scale=scale))

    return get_active()


def add_grid(
        x_subdivisions: int | None = None,
        y_subdivisions: int | None = None,
        size: float | None = 1.0,
        location: Sequence[float] | mathutils.Vector = None,
        rotation: Sequence[float] | mathutils.Euler = None,
        scale: Sequence[float] | mathutils.Vector = None,
) -> Any:
    bpy.ops.mesh.primitive_grid_add(
        **filter_none_args(
            x_subdivisions=x_subdivisions,
            y_subdivisions=y_subdivisions,
            size=size,
            location=location,
            rotation=rotation,
            scale=scale,
        ),
    )

    return get_active()


def add_cylinder(
        vertices: int | None = 32,
        radius: float | None = 1.0,
        depth: float | None = 2.0,
        location: Sequence[float] | mathutils.Vector = None,
        rotation: Sequence[float] | mathutils.Euler = None,
        scale: Sequence[float] | mathutils.Vector = None,
        material: bpy.types.Material | None = None,
) -> Any:
    bpy.ops.mesh.primitive_cylinder_add(
        **filter_none_args(
            vertices=vertices,
            radius=radius,
            depth=depth,
            location=location,
            rotation=rotation,
            scale=scale,
        ),
    )

    cylinder = get_active()

    if material is not None:
        cylinder.data.materials.append(material)

    return cylinder


def generate_model(tag_id: int):
    # clear everything
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)

    add_cylinder(
        radius=post_radius,
        depth=post_height,
        location=(0, 0, base_thickness + (post_height / 2)),
        material=gray,
    )

    add_cylinder(
        radius=base_radius,
        depth=base_thickness,
        location=(0, 0, base_thickness / 2),
        material=darkgray,
    )

    add_cube(
        location=(0, post_radius - plate_thickness / 2, base_thickness + post_height * plate_position),
        rotation=(math.pi / 2, math.pi, 0),
        scale=(marker_size + marker_border, marker_size + marker_border, plate_thickness),
        material=white,
    )

    aruco_plane = add_plane(
        size=marker_size,
        location=(0, post_radius + plate_thickness / 2, base_thickness + post_height * plate_position),
        rotation=(math.pi / 2, math.pi, 0),
    )

    aruco_plane.data.materials.append(black)
    aruco_plane.data.materials.append(white)

    bpy.ops.object.mode_set(mode="EDIT")
    plane_mesh = bmesh.from_edit_mesh(aruco_plane.data)

    bmesh.ops.subdivide_edges(
        plane_mesh,
        edges=list(plane_mesh.edges),
        cuts=(cells + 1),
        use_grid_fill=True,
    )

    bmesh.update_edit_mesh(aruco_plane.data)
    bpy.ops.object.mode_set(mode="OBJECT")

    faces = []
    for face in aruco_plane.data.polygons:
        center = face.center
        faces.append((face, center.x, center.y))

    faces.sort(key=lambda t: (t[2], t[1]))

    bits = aruco_tags[tag_id]
    for row in range(cells + 2):
        for col in range(cells + 2):
            face, _x, _y = faces[row * (cells + 2) + col]

            if (row == 0 or row == cells + 1) or (col == 0 or col == cells + 1):
                face.material_index = 0
            else:
                bit = bits[row - 1][col - 1]
                face.material_index = bit

    bpy.ops.object.mode_set(mode="EDIT")
    plane_mesh = bmesh.from_edit_mesh(aruco_plane.data)

    # noinspection PyTypeChecker
    geom = bmesh.ops.extrude_face_region(plane_mesh, geom=list(plane_mesh.faces))
    verts = [v for v in geom["geom"] if isinstance(v, bmesh.types.BMVert)]

    bmesh.ops.translate(
        plane_mesh,
        verts=verts,
        vec=(0, 0, plate_thickness),
    )

    bmesh.update_edit_mesh(aruco_plane.data)
    bpy.ops.object.mode_set(mode="OBJECT")

    bpy.ops.object.select_all(action="SELECT")

    bpy.ops.object.join()

    def optimize_mesh(obj: Any, merge_distance: float = 1e-6) -> None:
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.remove_doubles(threshold=merge_distance)
        bpy.ops.mesh.normals_make_consistent(inside=False)
        bpy.ops.mesh.delete_loose()
        bpy.ops.mesh.dissolve_degenerate()
        bpy.ops.object.mode_set(mode="OBJECT")

    optimize_mesh(get_active())

    base = Path(f"aruco_post_{tag_id}")

    (base / "meshes").mkdir(parents=True, exist_ok=True)

    (base / "model.config").write_text(
        f"""
<?xml version='1.0'?>
<model>
    <name>ArUco tag id {tag_id}</name>
    <version>1.0</version>
    <sdf version='1.6'>model.sdf</sdf>

    <description>ArUco tag id {tag_id}</description>
</model>
        """.strip() + "\n",
    )

    (base / "model.sdf").write_text(
        f"""
<?xml version='1.0' ?>
<sdf version='1.6'>
    <model name='ArUco tag id {tag_id}'>
        <static>true</static>
        <link name='link'>
            <pose>0 0 0 {math.pi / 2} 0 {-math.pi / 2}</pose>
            <collision name='collision'>
                <geometry>
                    <mesh>
                        <uri>model://aruco_post_{tag_id}/meshes/post.obj</uri>
                    </mesh>
                </geometry>
            </collision>
            <visual name='visual'>
                <geometry>
                    <mesh>
                        <uri>model://aruco_post_{tag_id}/meshes/post.obj</uri>
                    </mesh>
                </geometry>
            </visual>
        </link>
    </model>
</sdf>
        """.strip() + "\n",
    )

    # bpy.ops.export_scene.dae(filepath="aruco_post.dae")
    bpy.ops.export_scene.gltf(
        filepath=str(base / "meshes" / "post.glb"),
        export_format="GLB",
        export_materials="EXPORT",
    )

    bpy.ops.wm.obj_export(
        filepath=str(base / "meshes" / "post.obj"),
        export_colors=True,
        export_materials=True,
    )


for i in id_range:
    generate_model(i)
