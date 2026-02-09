"""SDF XML parser: extracts model structure into Python dataclasses."""

import os
import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional

from .utils import Pose, parse_pose_string, resolve_model_uri

logger = logging.getLogger("sdf2usd")


# ─── Dataclasses ───────────────────────────────────────────────────

@dataclass
class SdfInertia:
    ixx: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 0.0
    iyz: float = 0.0
    izz: float = 0.0


@dataclass
class SdfInertial:
    mass: float = 0.0
    pose: Pose = field(default_factory=Pose.identity)
    inertia: SdfInertia = field(default_factory=SdfInertia)


@dataclass
class SdfMeshGeometry:
    uri: str = ""
    resolved_path: str = ""
    scale: tuple = (1.0, 1.0, 1.0)


@dataclass
class SdfBoxGeometry:
    size: tuple = (1.0, 1.0, 1.0)


@dataclass
class SdfCylinderGeometry:
    radius: float = 0.5
    length: float = 1.0


@dataclass
class SdfSphereGeometry:
    radius: float = 0.5


@dataclass
class SdfCapsuleGeometry:
    radius: float = 0.5
    length: float = 1.0


@dataclass
class SdfGeometry:
    """Geometry container — exactly one of the sub-fields will be set."""
    mesh: Optional[SdfMeshGeometry] = None
    box: Optional[SdfBoxGeometry] = None
    cylinder: Optional[SdfCylinderGeometry] = None
    sphere: Optional[SdfSphereGeometry] = None
    capsule: Optional[SdfCapsuleGeometry] = None

    @property
    def type_name(self) -> str:
        if self.mesh:
            return "mesh"
        if self.box:
            return "box"
        if self.cylinder:
            return "cylinder"
        if self.sphere:
            return "sphere"
        if self.capsule:
            return "capsule"
        return "unknown"


@dataclass
class SdfPbrMaterial:
    metalness: float = 0.0
    roughness: float = 0.5
    albedo_map: str = ""
    albedo_map_resolved: str = ""


@dataclass
class SdfMaterial:
    diffuse: tuple = (0.8, 0.8, 0.8)
    specular: tuple = (0.1, 0.1, 0.1)
    pbr: Optional[SdfPbrMaterial] = None


@dataclass
class SdfVisual:
    name: str = ""
    pose: Pose = field(default_factory=Pose.identity)
    geometry: Optional[SdfGeometry] = None
    material: Optional[SdfMaterial] = None


@dataclass
class SdfCollision:
    name: str = ""
    pose: Pose = field(default_factory=Pose.identity)
    geometry: Optional[SdfGeometry] = None


@dataclass
class SdfJointAxis:
    xyz: tuple = (0.0, 0.0, 1.0)
    lower_limit: float = -1e16
    upper_limit: float = 1e16
    damping: float = 0.0
    stiffness: float = 0.0
    friction: float = 0.0


@dataclass
class SdfJoint:
    name: str = ""
    type: str = "revolute"  # revolute, prismatic, fixed, ball
    parent: str = ""
    child: str = ""
    pose: Pose = field(default_factory=Pose.identity)
    axis: SdfJointAxis = field(default_factory=SdfJointAxis)


@dataclass
class SdfLink:
    name: str = ""
    pose: Pose = field(default_factory=Pose.identity)
    inertial: Optional[SdfInertial] = None
    visuals: list = field(default_factory=list)
    collisions: list = field(default_factory=list)


@dataclass
class SdfModel:
    name: str = ""
    pose: Pose = field(default_factory=Pose.identity)
    links: list = field(default_factory=list)
    joints: list = field(default_factory=list)

    def get_link(self, name: str) -> Optional[SdfLink]:
        for link in self.links:
            if link.name == name:
                return link
        return None


# ─── Parser ────────────────────────────────────────────────────────

class SdfParser:
    """Parse a Gazebo SDF file into SdfModel dataclasses."""

    def __init__(self, sdf_path: str):
        self.sdf_path = os.path.abspath(sdf_path)
        self.model_dir = os.path.dirname(self.sdf_path)

    def parse(self) -> SdfModel:
        """Parse the SDF file and return an SdfModel."""
        logger.info(f"Parsing SDF: {self.sdf_path}")
        tree = ET.parse(self.sdf_path)
        root = tree.getroot()

        # Find <model> element (may be nested under <sdf>)
        model_elem = root.find("model")
        if model_elem is None and root.tag == "model":
            model_elem = root

        if model_elem is None:
            raise ValueError(f"No <model> element found in {self.sdf_path}")

        return self._parse_model(model_elem)

    def _parse_model(self, elem: ET.Element) -> SdfModel:
        model = SdfModel(name=elem.get("name", "model"))

        pose_elem = elem.find("pose")
        if pose_elem is not None:
            model.pose = self._parse_pose(pose_elem)

        for link_elem in elem.findall("link"):
            model.links.append(self._parse_link(link_elem))

        for joint_elem in elem.findall("joint"):
            model.joints.append(self._parse_joint(joint_elem))

        logger.info(f"Parsed model '{model.name}': "
                     f"{len(model.links)} links, {len(model.joints)} joints")
        return model

    def _parse_link(self, elem: ET.Element) -> SdfLink:
        link = SdfLink(name=elem.get("name", "link"))

        pose_elem = elem.find("pose")
        if pose_elem is not None:
            link.pose = self._parse_pose(pose_elem)

        inertial_elem = elem.find("inertial")
        if inertial_elem is not None:
            link.inertial = self._parse_inertial(inertial_elem)

        for vis_elem in elem.findall("visual"):
            visual = self._parse_visual(vis_elem)
            if visual.geometry:
                link.visuals.append(visual)

        for col_elem in elem.findall("collision"):
            collision = self._parse_collision(col_elem)
            if collision.geometry:
                link.collisions.append(collision)

        logger.debug(f"  Link '{link.name}': "
                      f"{len(link.visuals)} visuals, {len(link.collisions)} collisions")
        return link

    def _parse_inertial(self, elem: ET.Element) -> SdfInertial:
        inertial = SdfInertial()

        mass_elem = elem.find("mass")
        if mass_elem is not None:
            inertial.mass = float(mass_elem.text)

        pose_elem = elem.find("pose")
        if pose_elem is not None:
            inertial.pose = self._parse_pose(pose_elem)

        inertia_elem = elem.find("inertia")
        if inertia_elem is not None:
            inertial.inertia = self._parse_inertia(inertia_elem)

        return inertial

    def _parse_inertia(self, elem: ET.Element) -> SdfInertia:
        inertia = SdfInertia()
        for tag in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]:
            child = elem.find(tag)
            if child is not None:
                setattr(inertia, tag, float(child.text))
        return inertia

    def _parse_visual(self, elem: ET.Element) -> SdfVisual:
        visual = SdfVisual(name=elem.get("name", "visual"))

        pose_elem = elem.find("pose")
        if pose_elem is not None:
            visual.pose = self._parse_pose(pose_elem)

        geom_elem = elem.find("geometry")
        if geom_elem is not None:
            visual.geometry = self._parse_geometry(geom_elem)

        mat_elem = elem.find("material")
        if mat_elem is not None:
            visual.material = self._parse_material(mat_elem)

        return visual

    def _parse_collision(self, elem: ET.Element) -> SdfCollision:
        collision = SdfCollision(name=elem.get("name", "collision"))

        pose_elem = elem.find("pose")
        if pose_elem is not None:
            collision.pose = self._parse_pose(pose_elem)

        geom_elem = elem.find("geometry")
        if geom_elem is not None:
            collision.geometry = self._parse_geometry(geom_elem)

        return collision

    def _parse_geometry(self, elem: ET.Element) -> SdfGeometry:
        geom = SdfGeometry()

        mesh_elem = elem.find("mesh")
        if mesh_elem is not None:
            mesh = SdfMeshGeometry()
            uri_elem = mesh_elem.find("uri")
            if uri_elem is not None:
                mesh.uri = uri_elem.text.strip()
                mesh.resolved_path = resolve_model_uri(mesh.uri, self.model_dir)
            scale_elem = mesh_elem.find("scale")
            if scale_elem is not None:
                parts = [float(v) for v in scale_elem.text.strip().split()]
                mesh.scale = tuple(parts[:3]) if len(parts) >= 3 else (1.0, 1.0, 1.0)
            geom.mesh = mesh
            return geom

        box_elem = elem.find("box")
        if box_elem is not None:
            box = SdfBoxGeometry()
            size_elem = box_elem.find("size")
            if size_elem is not None:
                parts = [float(v) for v in size_elem.text.strip().split()]
                box.size = tuple(parts[:3]) if len(parts) >= 3 else (1.0, 1.0, 1.0)
            geom.box = box
            return geom

        cyl_elem = elem.find("cylinder")
        if cyl_elem is not None:
            cyl = SdfCylinderGeometry()
            r = cyl_elem.find("radius")
            l = cyl_elem.find("length")
            if r is not None:
                cyl.radius = float(r.text)
            if l is not None:
                cyl.length = float(l.text)
            geom.cylinder = cyl
            return geom

        sphere_elem = elem.find("sphere")
        if sphere_elem is not None:
            sph = SdfSphereGeometry()
            r = sphere_elem.find("radius")
            if r is not None:
                sph.radius = float(r.text)
            geom.sphere = sph
            return geom

        capsule_elem = elem.find("capsule")
        if capsule_elem is not None:
            cap = SdfCapsuleGeometry()
            r = capsule_elem.find("radius")
            l = capsule_elem.find("length")
            if r is not None:
                cap.radius = float(r.text)
            if l is not None:
                cap.length = float(l.text)
            geom.capsule = cap
            return geom

        return geom

    def _parse_material(self, elem: ET.Element) -> SdfMaterial:
        mat = SdfMaterial()

        diffuse_elem = elem.find("diffuse")
        if diffuse_elem is not None:
            parts = [float(v) for v in diffuse_elem.text.strip().split()]
            mat.diffuse = tuple(parts[:3]) if len(parts) >= 3 else (0.8, 0.8, 0.8)

        specular_elem = elem.find("specular")
        if specular_elem is not None:
            parts = [float(v) for v in specular_elem.text.strip().split()]
            mat.specular = tuple(parts[:3]) if len(parts) >= 3 else (0.1, 0.1, 0.1)

        pbr_elem = elem.find("pbr")
        if pbr_elem is not None:
            metal_elem = pbr_elem.find("metal")
            if metal_elem is not None:
                pbr = SdfPbrMaterial()
                m = metal_elem.find("metalness")
                if m is not None:
                    pbr.metalness = float(m.text)
                r = metal_elem.find("roughness")
                if r is not None:
                    pbr.roughness = float(r.text)
                a = metal_elem.find("albedo_map")
                if a is not None:
                    pbr.albedo_map = a.text.strip()
                    pbr.albedo_map_resolved = resolve_model_uri(
                        pbr.albedo_map, self.model_dir)
                mat.pbr = pbr

        return mat

    def _parse_joint(self, elem: ET.Element) -> SdfJoint:
        joint = SdfJoint(
            name=elem.get("name", "joint"),
            type=elem.get("type", "revolute"),
        )

        pose_elem = elem.find("pose")
        if pose_elem is not None:
            joint.pose = self._parse_pose(pose_elem)

        parent_elem = elem.find("parent")
        if parent_elem is not None:
            joint.parent = parent_elem.text.strip()

        child_elem = elem.find("child")
        if child_elem is not None:
            joint.child = child_elem.text.strip()

        axis_elem = elem.find("axis")
        if axis_elem is not None:
            joint.axis = self._parse_joint_axis(axis_elem)

        return joint

    def _parse_joint_axis(self, elem: ET.Element) -> SdfJointAxis:
        axis = SdfJointAxis()

        xyz_elem = elem.find("xyz")
        if xyz_elem is not None:
            parts = [float(v) for v in xyz_elem.text.strip().split()]
            axis.xyz = tuple(parts[:3]) if len(parts) >= 3 else (0.0, 0.0, 1.0)

        limit_elem = elem.find("limit")
        if limit_elem is not None:
            lower = limit_elem.find("lower")
            upper = limit_elem.find("upper")
            if lower is not None:
                axis.lower_limit = float(lower.text)
            if upper is not None:
                axis.upper_limit = float(upper.text)

        dynamics_elem = elem.find("dynamics")
        if dynamics_elem is not None:
            d = dynamics_elem.find("damping")
            s = dynamics_elem.find("stiffness")
            f = dynamics_elem.find("friction")
            if d is not None:
                axis.damping = float(d.text)
            if s is not None:
                axis.stiffness = float(s.text)
            if f is not None:
                axis.friction = float(f.text)

        return axis

    def _parse_pose(self, elem: ET.Element) -> Pose:
        """Parse a <pose> element, handling degrees='true' attribute."""
        if elem.text is None:
            return Pose.identity()
        degrees = elem.get("degrees", "false").lower() == "true"
        return parse_pose_string(elem.text, degrees=degrees)
