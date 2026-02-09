"""Main converter orchestrator: SDF -> USD pipeline."""

import os
import logging

from .parser import SdfParser
from .mesh_converter import MeshConverter
from .usd_builder import UsdBuilder

logger = logging.getLogger("sdf2usd")


class SdfToUsdConverter:
    """Convert a Gazebo SDF model to USD format for Isaac Sim.

    Usage:
        converter = SdfToUsdConverter(
            sdf_path="model.sdf",
            output_path="model.usda",
        )
        converter.convert()
    """

    def __init__(
        self,
        sdf_path: str,
        output_path: str,
        up_axis: str = "Z",
        scale: float = 1.0,
        include_physics: bool = True,
        include_collision: bool = True,
        merge_fixed_joints: bool = False,
    ):
        self.sdf_path = os.path.abspath(sdf_path)
        self.output_path = os.path.abspath(output_path)
        self.up_axis = up_axis
        self.scale = scale
        self.include_physics = include_physics
        self.include_collision = include_collision
        self.merge_fixed_joints = merge_fixed_joints

        if not os.path.exists(self.sdf_path):
            raise FileNotFoundError(f"SDF file not found: {self.sdf_path}")

    def convert(self):
        """Run the full SDF -> USD conversion pipeline."""
        logger.info("=" * 60)
        logger.info("  sdf-to-usd-converter v0.1.0")
        logger.info("=" * 60)
        logger.info(f"  Input:  {self.sdf_path}")
        logger.info(f"  Output: {self.output_path}")
        logger.info(f"  Physics: {'Yes' if self.include_physics else 'No'}")
        logger.info(f"  Collision: {'Yes' if self.include_collision else 'No'}")
        logger.info(f"  Up axis: {self.up_axis}")
        logger.info(f"  Scale: {self.scale}")
        logger.info("")

        # Step 1: Parse SDF
        logger.info("[1/4] Parsing SDF...")
        parser = SdfParser(self.sdf_path)
        model = parser.parse()

        # Count meshes to convert
        mesh_count = 0
        for link in model.links:
            for vis in link.visuals:
                if vis.geometry and vis.geometry.mesh:
                    mesh_count += 1
            if self.include_collision:
                for col in link.collisions:
                    if col.geometry and col.geometry.mesh:
                        mesh_count += 1
        logger.info(f"  Found {mesh_count} mesh files to convert")

        # Step 2: Convert meshes
        logger.info("[2/4] Converting meshes...")
        output_dir = os.path.dirname(self.output_path)
        os.makedirs(output_dir, exist_ok=True)
        mesh_converter = MeshConverter(output_dir)

        # Pre-convert all unique mesh files
        converted = 0
        mesh_paths = set()
        for link in model.links:
            for vis in link.visuals:
                if vis.geometry and vis.geometry.mesh:
                    mesh_paths.add(vis.geometry.mesh.resolved_path)
            if self.include_collision:
                for col in link.collisions:
                    if col.geometry and col.geometry.mesh:
                        mesh_paths.add(col.geometry.mesh.resolved_path)

        for mesh_path in sorted(mesh_paths):
            if os.path.exists(mesh_path):
                try:
                    mesh_converter.convert(mesh_path)
                    converted += 1
                except Exception as e:
                    logger.error(f"  Failed to convert {mesh_path}: {e}")

        logger.info(f"  Converted {converted}/{len(mesh_paths)} unique meshes")

        # Step 3: Build USD
        logger.info("[3/4] Building USD stage...")
        builder = UsdBuilder(
            output_path=self.output_path,
            mesh_converter=mesh_converter,
            up_axis=self.up_axis,
            scale=self.scale,
            include_physics=self.include_physics,
            include_collision=self.include_collision,
        )
        builder.build(model)

        # Step 4: Summary
        logger.info("[4/4] Done!")
        logger.info("")
        logger.info("=" * 60)
        logger.info(f"  Output: {self.output_path}")
        file_size = os.path.getsize(self.output_path)
        logger.info(f"  Size: {file_size / 1024:.1f} KB")
        logger.info(f"  Links: {len(model.links)}")
        logger.info(f"  Joints: {len(model.joints)}")
        logger.info(f"  Meshes converted: {converted}")
        logger.info("=" * 60)

        return self.output_path
