"""Physics: joints, collision, inertia -> USD Physics API."""

import math
import logging

from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from .parser import SdfJoint, SdfInertial, SdfCollision, SdfGeometry, SdfLink
from .utils import Pose, sanitize_usd_name

logger = logging.getLogger("sdf2usd")


class PhysicsBuilder:
    """Apply physics properties (joints, inertia, collision) to a USD stage."""

    def __init__(self, stage):
        self.stage = stage

    def setup_articulation_root(self, root_path: str):
        """Apply ArticulationRootAPI to the model root prim."""
        prim = self.stage.GetPrimAtPath(root_path)
        if prim:
            UsdPhysics.ArticulationRootAPI.Apply(prim)
            logger.debug(f"  ArticulationRoot: {root_path}")

    def apply_rigid_body(self, link_path: str, link: SdfLink):
        """Apply RigidBodyAPI and MassAPI to a link prim."""
        prim = self.stage.GetPrimAtPath(link_path)
        if not prim:
            return

        UsdPhysics.RigidBodyAPI.Apply(prim)

        if link.inertial and link.inertial.mass > 0:
            mass_api = UsdPhysics.MassAPI.Apply(prim)
            mass_api.CreateMassAttr().Set(link.inertial.mass)

            # Center of mass from inertial pose
            com = link.inertial.pose.position
            mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(
                float(com[0]), float(com[1]), float(com[2])
            ))

            # Diagonal inertia
            inertia = link.inertial.inertia
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(
                float(inertia.ixx), float(inertia.iyy), float(inertia.izz)
            ))

            logger.debug(f"  RigidBody + Mass({link.inertial.mass:.3f}kg): {link_path}")
        else:
            logger.debug(f"  RigidBody (no mass): {link_path}")

    def apply_collision(self, collision_prim_path: str):
        """Apply CollisionAPI to a collision geometry prim."""
        prim = self.stage.GetPrimAtPath(collision_prim_path)
        if prim:
            UsdPhysics.CollisionAPI.Apply(prim)

    def create_joint(self, model_path: str, joint: SdfJoint):
        """Create a USD physics joint from SDF joint data."""
        joint_name = sanitize_usd_name(joint.name)
        joint_path = f"{model_path}/{joint_name}"

        parent_path = f"{model_path}/{sanitize_usd_name(joint.parent)}"
        child_path = f"{model_path}/{sanitize_usd_name(joint.child)}"

        if joint.type == "revolute":
            self._create_revolute_joint(joint_path, parent_path, child_path, joint)
        elif joint.type == "prismatic":
            self._create_prismatic_joint(joint_path, parent_path, child_path, joint)
        elif joint.type == "fixed":
            self._create_fixed_joint(joint_path, parent_path, child_path, joint)
        elif joint.type == "ball":
            self._create_fixed_joint(joint_path, parent_path, child_path, joint)
            logger.warning(f"  Ball joint '{joint.name}' converted to fixed (not fully supported)")
        else:
            logger.warning(f"  Unsupported joint type '{joint.type}' for '{joint.name}'")
            return

    def _create_revolute_joint(self, joint_path: str, parent_path: str,
                                child_path: str, joint: SdfJoint):
        """Create a revolute (hinge) joint."""
        usd_joint = UsdPhysics.RevoluteJoint.Define(self.stage, joint_path)

        # Set bodies
        usd_joint.CreateBody0Rel().SetTargets([parent_path])
        usd_joint.CreateBody1Rel().SetTargets([child_path])

        # Set axis
        axis = self._determine_axis(joint.axis.xyz)
        usd_joint.CreateAxisAttr().Set(axis)

        # Joint frame offset (from joint pose)
        pos = joint.pose.position
        quat = joint.pose.to_quaternion_wxyz()
        usd_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(pos[0]), float(pos[1]), float(pos[2])))
        usd_joint.CreateLocalRot0Attr().Set(Gf.Quatf(
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))

        # Limits (convert from radians to degrees for USD)
        lower_deg = math.degrees(joint.axis.lower_limit)
        upper_deg = math.degrees(joint.axis.upper_limit)

        # Only set finite limits
        if abs(lower_deg) < 1e10:
            usd_joint.CreateLowerLimitAttr().Set(float(lower_deg))
        if abs(upper_deg) < 1e10:
            usd_joint.CreateUpperLimitAttr().Set(float(upper_deg))

        # Damping via DriveAPI
        if joint.axis.damping > 0:
            prim = self.stage.GetPrimAtPath(joint_path)
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateDampingAttr().Set(float(joint.axis.damping))
            if joint.axis.stiffness > 0:
                drive.CreateStiffnessAttr().Set(float(joint.axis.stiffness))

        logger.debug(f"  RevoluteJoint: {joint.name} ({joint.parent} -> {joint.child})")

    def _create_prismatic_joint(self, joint_path: str, parent_path: str,
                                 child_path: str, joint: SdfJoint):
        """Create a prismatic (sliding) joint."""
        usd_joint = UsdPhysics.PrismaticJoint.Define(self.stage, joint_path)

        usd_joint.CreateBody0Rel().SetTargets([parent_path])
        usd_joint.CreateBody1Rel().SetTargets([child_path])

        axis = self._determine_axis(joint.axis.xyz)
        usd_joint.CreateAxisAttr().Set(axis)

        pos = joint.pose.position
        quat = joint.pose.to_quaternion_wxyz()
        usd_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(pos[0]), float(pos[1]), float(pos[2])))
        usd_joint.CreateLocalRot0Attr().Set(Gf.Quatf(
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))

        # Limits in meters for prismatic joints
        if abs(joint.axis.lower_limit) < 1e10:
            usd_joint.CreateLowerLimitAttr().Set(float(joint.axis.lower_limit))
        if abs(joint.axis.upper_limit) < 1e10:
            usd_joint.CreateUpperLimitAttr().Set(float(joint.axis.upper_limit))

        if joint.axis.damping > 0:
            prim = self.stage.GetPrimAtPath(joint_path)
            drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
            drive.CreateDampingAttr().Set(float(joint.axis.damping))

        logger.debug(f"  PrismaticJoint: {joint.name} ({joint.parent} -> {joint.child})")

    def _create_fixed_joint(self, joint_path: str, parent_path: str,
                             child_path: str, joint: SdfJoint):
        """Create a fixed joint."""
        usd_joint = UsdPhysics.FixedJoint.Define(self.stage, joint_path)
        usd_joint.CreateBody0Rel().SetTargets([parent_path])
        usd_joint.CreateBody1Rel().SetTargets([child_path])

        pos = joint.pose.position
        quat = joint.pose.to_quaternion_wxyz()
        usd_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(pos[0]), float(pos[1]), float(pos[2])))
        usd_joint.CreateLocalRot0Attr().Set(Gf.Quatf(
            float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))

        logger.debug(f"  FixedJoint: {joint.name} ({joint.parent} -> {joint.child})")

    def _determine_axis(self, xyz: tuple) -> str:
        """Determine the primary USD axis from an SDF axis vector."""
        ax, ay, az = abs(xyz[0]), abs(xyz[1]), abs(xyz[2])
        if ax >= ay and ax >= az:
            return "X"
        elif ay >= ax and ay >= az:
            return "Y"
        return "Z"
