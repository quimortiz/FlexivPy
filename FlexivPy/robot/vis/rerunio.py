import argparse
import os
import pathlib
from typing import Optional

import numpy as np
import rerun as rr  # pip install rerun-sdk
import scipy.spatial.transform as st
import trimesh
import trimesh.visual
from PIL import Image
from urdf_parser_py import urdf as urdf_parser

import pinocchio as pin
import hppfcl
import time

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper





def scene_to_trimeshes(scene: trimesh.Scene) -> list[trimesh.Trimesh]:
    """
    Convert a trimesh.Scene to a list of trimesh.Trimesh.

    Skips objects that are not an instance of trimesh.Trimesh.
    """
    trimeshes = []
    scene_dump = scene.dump()
    geometries = [scene_dump] if not isinstance(scene_dump, list) else scene_dump
    for geometry in geometries:
        if isinstance(geometry, trimesh.Trimesh):
            trimeshes.append(geometry)
        elif isinstance(geometry, trimesh.Scene):
            trimeshes.extend(scene_to_trimeshes(geometry))
    return trimeshes



def log_trimesh(entity_path: str, mesh: trimesh.Trimesh) -> None:
    vertex_colors = albedo_texture = vertex_texcoords = None

    if isinstance(mesh.visual, trimesh.visual.color.ColorVisuals):
        vertex_colors = mesh.visual.vertex_colors
    elif isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals):
        trimesh_material = mesh.visual.material

        if mesh.visual.uv is not None:
            vertex_texcoords = mesh.visual.uv
            # Trimesh uses the OpenGL convention for UV coordinates, so we need to flip the V coordinate
            # since Rerun uses the Vulkan/Metal/DX12/WebGPU convention.
            vertex_texcoords[:, 1] = 1.0 - vertex_texcoords[:, 1]

        if isinstance(trimesh_material, trimesh.visual.material.PBRMaterial):
            if trimesh_material.baseColorTexture is not None:
                albedo_texture = pil_image_to_albedo_texture(trimesh_material.baseColorTexture)
            elif trimesh_material.baseColorFactor is not None:
                vertex_colors = trimesh_material.baseColorFactor
        elif isinstance(trimesh_material, trimesh.visual.material.SimpleMaterial):
            if trimesh_material.image is not None:
                albedo_texture = pil_image_to_albedo_texture(trimesh_material.image)
            else:
                vertex_colors = mesh.visual.to_color().vertex_colors

    rr.log(
        entity_path,
        rr.Mesh3D(
            vertex_positions=mesh.vertices,
            triangle_indices=mesh.faces,
            vertex_normals=mesh.vertex_normals,
            vertex_colors=vertex_colors,
            albedo_texture=albedo_texture,
            vertex_texcoords=vertex_texcoords,
        ),
        timeless=True,
    )



class Robot_logger_rerunio:
    """Class to log a URDF to Rerun."""

    def __init__(self, 
                 robot, entity_path_prefix: Optional[str]) -> None:
        self.robot = robot
        self.entity_path_prefix = entity_path_prefix

    def log(self, q: Optional[np.ndarray] = None, log_meshes: bool = False) -> None:
        """
        """

        # if geometry_type == pin.GeometryType.VISUAL:
        geom_model = self.robot.visual_model
        geom_data = self.robot.visual_data

        if q is None:
            pin.updateGeometryPlacements(self.robot.model, self.robot.data, geom_model, geom_data)

        else:
            pin.forwardKinematics(self.robot.model, self.robot.data, q)
            pin.updateGeometryPlacements(self.robot.model, self.robot.data, geom_model, geom_data)

        for visual in geom_model.geometryObjects:

            T = geom_data.oMg[geom_model.getGeometryId(visual.name)]
            # Manage scaling: force scaling even if this should be normally handled by MeshCat (but there is a bug here)
            geom = visual.geometry
            # if WITH_HPP_FCL_BINDINGS and isinstance(
            #     geom, (hppfcl.Plane, hppfcl.Halfspace)
            # ):
            #     T = M.copy()
            #     T.translation += M.rotation @ (geom.d * geom.n)
            #     T = T.homogeneous
            # else:
            #     T = M.homogeneous

            # Update viewer configuration.




            visual_name = visual.name

            # self.viewer[visual_name].set_transform(T)
            rr.log(visual_name, rr.Transform3D(translation=T.translation, mat3x3=T.rotation))


            # add the mesh

            # i just have to log the mesh!

            if log_meshes:


            # self.log_visual(visual_name) 
                if type(visual.geometry) == hppfcl.Cylinder:
                    mesh_or_scene = trimesh.creation.cylinder(
                        radius=visual.geometry.radius,
                        height= 2 * visual.geometry.halfLength # visual.geometry.length,
                    )
                elif type(visual.geometry) == hppfcl.Box:
                    mesh_or_scene = trimesh.creation.box(extents=2. * visual.geometry.halfSide)
                elif type(visual.geometry) == hppfcl.Sphere:
                    mesh_or_scene = trimesh.creation.icosphere(
                        radius=visual.geometry.radius,
                    )
                elif type(visual.geometry) == hppfcl.BVHModelOBBRSS:
                    mesh_or_scene = trimesh.load_mesh(visual.meshPath)

                else: 
                    raise ValueError('unsupported geometry type')

                if isinstance(mesh_or_scene, trimesh.Scene):
                    scene = mesh_or_scene
                    for i, mesh in enumerate(scene_to_trimeshes(scene)):
                        mesh.visual = trimesh.visual.ColorVisuals()
                        mesh.visual.vertex_colors = visual.meshColor
                        log_trimesh(visual_name + f"/v0{i}", mesh)


                else:
                    mesh_or_scene.visual = trimesh.visual.ColorVisuals()
                    mesh_or_scene.visual.vertex_colors = visual.meshColor
                    log_trimesh(visual_name + f"/v0", mesh_or_scene)
