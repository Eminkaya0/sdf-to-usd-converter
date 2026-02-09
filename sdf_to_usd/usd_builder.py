"""USD stage builder: creates the prim hierarchy from parsed SDF data."""

import os
import logging

from pxr import Usd, UsdGeom, Sdf, Gf

from .parser import SdfModel, SdfLink, SdfVisual, SdfCollision, SdfGeometry
from .utils import Pose, sanitize_usd_name
from .mesh_converter import MeshConverter
from .materials import MaterialBuilder
from .physics import PhysicsBuilder

logger = logging.getLogger("sdf2usd")


class UsdBuilder:
    """Build a USD stage from a parsed SDF model."""

    def __init__(self, output_path: str, mesh_converter: MeshConverter,
                 up_axis: str = "Z", scale: float = 1.0,
                 include_physics: bool = True, include_collision: bool = True):
        self.output_path = output_path
        self.output_dir = os.path.dirname(os.path.abspath(output_path))
        self.mesh_converter = mesh_converter
        self.up_axis = up_axis
        self.scale = scale
        self.include_physics = include_physics
        self.include_collision = include_collision
        self.stage = None
        self.material_builder = None
        self.physics_builder = None

    def build(self, model: SdfModel):
        """Build the complete USD stage from an SdfModel."""
        logger.info(f"Building USD stage: {self.output_path}")

        # Create stage
        self.stage = Usd.Stage.CreateNew(self.output_path)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z if self.up_axis == "Z" else UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)

        model_name = sanitize_usd_name(model.name)
        model_path = f"/{model_name}"
        self.stage.SetDefaultPrim(self.stage.DefinePrim(model_path))

        # Create model root Xform
        model_xform = UsdGeom.Xform.Define(self.stage, model_path)
        self._set_xform(model_xform, model.pose)

        # Looks scope for materials
        looks_path = f"{model_path}/Looks"
        UsdGeom.Scope.Define(self.stage, looks_path)
        self.material_builder = MaterialBuilder(
            self.stage, looks_path, self.mesh_converter)

        # Physics builder
        self.physics_builder = PhysicsBuilder(self.stage)
        if self.include_physics:
            self.physics_builder.setup_articulation_root(model_path)

        # Build links
        for link in model.links:
            self._build_link(model_path, link)

        # Build joints
        if self.include_physics:
            for joint in model.joints:
                self.physics_builder.create_joint(model_path, joint)

        # Save
        self.stage.GetRootLayer().Save()
        file_size = os.path.getsize(self.output_path)
        logger.info(f"USD saved: {self.output_path} ({file_size / 1024:.1f} KB)")

    def _build_link(self, model_path: str, link: SdfLink):
        """Build a link prim with its visuals and collisions."""
        link_name = sanitize_usd_name(link.name)
        link_path = f"{model_path}/{link_name}"

        link_xform = UsdGeom.Xform.Define(self.stage, link_path)
        self._set_xform(link_xform, link.pose)

        # Apply physics (rigid body + mass)
        if self.include_physics:
            self.physics_builder.apply_rigid_body(link_path, link)

        # Visuals scope
        if link.visuals:
            visuals_path = f"{link_path}/visuals"
            UsdGeom.Scope.Define(self.stage, visuals_path)

            for visual in link.visuals:
                self._build_visual(visuals_path, visual)

        # Collisions scope
        if self.include_collision and link.collisions:
            collisions_path = f"{link_path}/collisions"
            UsdGeom.Scope.Define(self.stage, collisions_path)

            for collision in link.collisions:
                self._build_collision(collisions_path, collision)

        logger.debug(f"  Built link: {link_name}")

    def _build_visual(self, parent_path: str, visual: SdfVisual):
        """Build a visual geometry prim."""
        vis_name = sanitize_usd_name(visual.name)
        vis_path = f"{parent_path}/{vis_name}"

        if visual.geometry is None:
            return

        geom_path = self._build_geometry(vis_path, visual.geometry, visual.pose)
        if geom_path is None:
            return

        # Apply material
        if visual.material:
            mat_path = self.material_builder.create_material(
                visual.material, name_hint=vis_name)
            self.material_builder.assign_material(geom_path, mat_path)

    def _build_collision(self, parent_path: str, collision: SdfCollision):
        """Build a collision geometry prim."""
        col_name = sanitize_usd_name(collision.name)
        col_path = f"{parent_path}/{col_name}"

        if collision.geometry is None:
            return

        geom_path = self._build_geometry(col_path, collision.geometry, collision.pose)
        if geom_path:
            self.physics_builder.apply_collision(geom_path)

    def _build_geometry(self, prim_path: str, geometry: SdfGeometry, pose: Pose) -> str:
        """Build geometry (mesh reference or primitive shape). Returns the prim path."""
        if geometry.mesh:
            return self._build_mesh_reference(prim_path, geometry, pose)
        elif geometry.box:
            return self._build_box(prim_path, geometry.box, pose)
        elif geometry.cylinder:
            return self._build_cylinder(prim_path, geometry.cylinder, pose)
        elif geometry.sphere:
            return self._build_sphere(prim_path, geometry.sphere, pose)
        elif geometry.capsule:
            return self._build_capsule(prim_path, geometry.capsule, pose)
        return None

    def _build_mesh_reference(self, prim_path: str, geometry: SdfGeometry, pose: Pose) -> str:
        """Convert a mesh file and add a USD reference to it."""
        mesh_geom = geometry.mesh
        if not mesh_geom.resolved_path or not os.path.exists(mesh_geom.resolved_path):
            logger.warning(f"  Mesh file not found: {mesh_geom.resolved_path}")
            return None

        try:
            usd_mesh_path = self.mesh_converter.convert(mesh_geom.resolved_path)
        except Exception as e:
            logger.error(f"  Mesh conversion failed: {e}")
            return None

        # Create Xform with reference to the converted mesh USD
        xform = UsdGeom.Xform.Define(self.stage, prim_path)
        self._set_xform(xform, pose, scale=mesh_geom.scale)

        # Add reference to the converted USD file
        rel_path = os.path.relpath(usd_mesh_path, self.output_dir)
        prim = self.stage.GetPrimAtPath(prim_path)
        prim.GetReferences().AddReference(f"./{rel_path}")

        return prim_path

    def _build_box(self, prim_path: str, box, pose: Pose) -> str:
        """Create a UsdGeom.Cube scaled to box dimensions."""
        xform = UsdGeom.Xform.Define(self.stage, prim_path)
        self._set_xform(xform, pose)

        cube_path = f"{prim_path}/Box"
        cube = UsdGeom.Cube.Define(self.stage, cube_path)
        cube.CreateSizeAttr().Set(1.0)

        # Scale to box dimensions
        sx, sy, sz = box.size
        scale_op = UsdGeom.Xform.Define(self.stage, cube_path)
        scale_op.AddScaleOp().Set(Gf.Vec3f(float(sx), float(sy), float(sz)))

        return cube_path

    def _build_cylinder(self, prim_path: str, cyl, pose: Pose) -> str:
        """Create a UsdGeom.Cylinder."""
        xform = UsdGeom.Xform.Define(self.stage, prim_path)
        self._set_xform(xform, pose)

        cyl_path = f"{prim_path}/Cylinder"
        usd_cyl = UsdGeom.Cylinder.Define(self.stage, cyl_path)
        usd_cyl.CreateRadiusAttr().Set(float(cyl.radius))
        usd_cyl.CreateHeightAttr().Set(float(cyl.length))
        usd_cyl.CreateAxisAttr().Set("Z")

        return cyl_path

    def _build_sphere(self, prim_path: str, sph, pose: Pose) -> str:
        """Create a UsdGeom.Sphere."""
        xform = UsdGeom.Xform.Define(self.stage, prim_path)
        self._set_xform(xform, pose)

        sph_path = f"{prim_path}/Sphere"
        usd_sph = UsdGeom.Sphere.Define(self.stage, sph_path)
        usd_sph.CreateRadiusAttr().Set(float(sph.radius))

        return sph_path

    def _build_capsule(self, prim_path: str, cap, pose: Pose) -> str:
        """Create a UsdGeom.Capsule."""
        xform = UsdGeom.Xform.Define(self.stage, prim_path)
        self._set_xform(xform, pose)

        cap_path = f"{prim_path}/Capsule"
        usd_cap = UsdGeom.Capsule.Define(self.stage, cap_path)
        usd_cap.CreateRadiusAttr().Set(float(cap.radius))
        usd_cap.CreateHeightAttr().Set(float(cap.length))
        usd_cap.CreateAxisAttr().Set("Z")

        return cap_path

    def _set_xform(self, xform: UsdGeom.Xform, pose: Pose,
                    scale: tuple = None):
        """Set translate + orient + scale on a UsdGeom.Xform."""
        # Translate
        xform.AddTranslateOp().Set(Gf.Vec3d(
            float(pose.x), float(pose.y), float(pose.z)))

        # Orient (quaternion wxyz)
        quat = pose.to_quaternion_wxyz()
        xform.AddOrientOp().Set(Gf.Quatf(
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))

        # Scale
        if scale and scale != (1.0, 1.0, 1.0):
            xform.AddScaleOp().Set(Gf.Vec3f(
                float(scale[0]), float(scale[1]), float(scale[2])))
        elif self.scale != 1.0:
            s = self.scale
            xform.AddScaleOp().Set(Gf.Vec3f(s, s, s))
