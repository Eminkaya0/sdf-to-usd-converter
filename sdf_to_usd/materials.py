"""Material handling: SDF materials -> USD UsdPreviewSurface shaders."""

import os
import logging

from pxr import Sdf, UsdShade, Gf

from .parser import SdfMaterial, SdfVisual
from .utils import sanitize_usd_name

logger = logging.getLogger("sdf2usd")


class MaterialBuilder:
    """Create and assign USD materials from SDF material definitions."""

    def __init__(self, stage, looks_scope_path: str, mesh_converter=None):
        """
        Args:
            stage: The USD stage.
            looks_scope_path: Path to the /Model/Looks scope for materials.
            mesh_converter: MeshConverter instance for texture copying.
        """
        self.stage = stage
        self.looks_path = looks_scope_path
        self.mesh_converter = mesh_converter
        self._material_cache: dict[str, str] = {}  # hash -> material_path
        self._counter = 0

    def create_material(self, sdf_mat: SdfMaterial, name_hint: str = "") -> str:
        """Create a USD material from SDF material data.

        Returns:
            The USD prim path of the created material.
        """
        mat_key = self._material_hash(sdf_mat)
        if mat_key in self._material_cache:
            return self._material_cache[mat_key]

        mat_name = sanitize_usd_name(name_hint) if name_hint else f"material_{self._counter}"
        self._counter += 1
        mat_path = f"{self.looks_path}/{mat_name}"

        material = UsdShade.Material.Define(self.stage, mat_path)
        shader = UsdShade.Shader.Define(self.stage, f"{mat_path}/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")

        # Diffuse color
        r, g, b = sdf_mat.diffuse
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(r, g, b))

        # Metallic and roughness from PBR
        if sdf_mat.pbr:
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(sdf_mat.pbr.metalness)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(sdf_mat.pbr.roughness)

            # Albedo texture map
            if sdf_mat.pbr.albedo_map_resolved and os.path.exists(sdf_mat.pbr.albedo_map_resolved):
                tex_path = self._setup_texture(
                    mat_path, sdf_mat.pbr.albedo_map_resolved, shader
                )
        else:
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)

        # Specular (as IOR approximation)
        spec_avg = sum(sdf_mat.specular) / 3.0
        if spec_avg > 0.5:
            shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)

        # Connect shader outputs to material
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        self._material_cache[mat_key] = mat_path
        logger.debug(f"  Created material: {mat_name}")
        return mat_path

    def assign_material(self, prim_path: str, material_path: str):
        """Bind a material to a prim."""
        prim = self.stage.GetPrimAtPath(prim_path)
        material = UsdShade.Material.Get(self.stage, material_path)
        if prim and material:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)

    def _setup_texture(self, mat_path: str, texture_file: str, shader) -> str:
        """Create a texture reader shader and connect to diffuseColor."""
        # Copy texture to output
        if self.mesh_converter:
            texture_file = self.mesh_converter.copy_texture(texture_file)

        tex_reader = UsdShade.Shader.Define(self.stage, f"{mat_path}/diffuse_texture")
        tex_reader.CreateIdAttr("UsdUVTexture")
        tex_reader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_file)
        tex_reader.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex_reader.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex_reader.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

        # Connect texture output to shader diffuseColor
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            tex_reader.ConnectableAPI(), "rgb"
        )

        # UV reader
        uv_reader = UsdShade.Shader.Define(self.stage, f"{mat_path}/uv_reader")
        uv_reader.CreateIdAttr("UsdPrimvarReader_float2")
        uv_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        uv_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        tex_reader.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
            uv_reader.ConnectableAPI(), "result"
        )

        return f"{mat_path}/diffuse_texture"

    def _material_hash(self, mat: SdfMaterial) -> str:
        """Create a simple hash key for deduplication."""
        parts = [f"d={mat.diffuse}", f"s={mat.specular}"]
        if mat.pbr:
            parts.append(f"m={mat.pbr.metalness}")
            parts.append(f"r={mat.pbr.roughness}")
            parts.append(f"a={mat.pbr.albedo_map}")
        return "|".join(parts)
