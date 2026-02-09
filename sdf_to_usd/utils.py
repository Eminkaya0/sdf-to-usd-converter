"""Utility functions: pose math, URI resolution, logging."""

import os
import math
import logging
import numpy as np
from dataclasses import dataclass

logger = logging.getLogger("sdf2usd")


@dataclass
class Pose:
    """6-DOF pose: position (x, y, z) + orientation (roll, pitch, yaw) in radians."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 homogeneous transform matrix (ZYX Euler convention)."""
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)

        R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr],
        ])

        mat = np.eye(4)
        mat[:3, :3] = R
        mat[:3, 3] = [self.x, self.y, self.z]
        return mat

    def to_quaternion_wxyz(self) -> tuple:
        """Convert to quaternion in (w, x, y, z) order for USD."""
        mat = self.to_matrix()
        R = mat[:3, :3]
        trace = R[0, 0] + R[1, 1] + R[2, 2]

        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        norm = math.sqrt(w * w + x * x + y * y + z * z)
        return (w / norm, x / norm, y / norm, z / norm)

    @staticmethod
    def compose(parent: "Pose", child: "Pose") -> "Pose":
        """Compose two poses: result = parent * child."""
        mat = parent.to_matrix() @ child.to_matrix()
        return Pose.from_matrix(mat)

    @staticmethod
    def from_matrix(mat: np.ndarray) -> "Pose":
        """Extract pose from a 4x4 transform matrix."""
        x, y, z = mat[0, 3], mat[1, 3], mat[2, 3]
        R = mat[:3, :3]

        # Extract ZYX Euler angles
        pitch = math.asin(-np.clip(R[2, 0], -1.0, 1.0))
        if abs(math.cos(pitch)) > 1e-6:
            roll = math.atan2(R[2, 1], R[2, 2])
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            yaw = 0.0

        return Pose(x, y, z, roll, pitch, yaw)

    @staticmethod
    def identity() -> "Pose":
        return Pose()


def parse_pose_string(pose_str: str, degrees: bool = False) -> Pose:
    """Parse SDF pose string 'x y z roll pitch yaw' into a Pose object."""
    parts = [float(v) for v in pose_str.strip().split()]
    if len(parts) != 6:
        logger.warning(f"Invalid pose string (expected 6 values): '{pose_str}'")
        return Pose.identity()

    x, y, z, roll, pitch, yaw = parts
    if degrees:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)

    return Pose(x, y, z, roll, pitch, yaw)


def resolve_model_uri(uri: str, model_dir: str) -> str:
    """Resolve SDF model:// URIs to absolute file paths.

    Handles:
      - model://model_name/path  ->  model_dir/path
      - file://path              ->  path
      - relative/path            ->  model_dir/relative/path
      - /absolute/path           ->  /absolute/path
    """
    if uri.startswith("model://"):
        # model://model_name/meshes/file.dae -> meshes/file.dae
        parts = uri[len("model://"):].split("/", 1)
        if len(parts) > 1:
            return os.path.join(model_dir, parts[1])
        return model_dir

    if uri.startswith("file://"):
        return uri[len("file://"):]

    if os.path.isabs(uri):
        return uri

    return os.path.join(model_dir, uri)


def sanitize_usd_name(name: str) -> str:
    """Make a name safe for use as a USD prim name."""
    result = name.replace("-", "_").replace(".", "_").replace(" ", "_")
    # USD prim names must start with a letter or underscore
    if result and result[0].isdigit():
        result = "_" + result
    # Remove any remaining invalid characters
    result = "".join(c for c in result if c.isalnum() or c == "_")
    return result or "_unnamed"
