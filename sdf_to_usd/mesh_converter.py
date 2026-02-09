"""Mesh converter: DAE/STL/OBJ -> USD using Isaac Sim's asset converter."""

import os
import shutil
import asyncio
import logging

logger = logging.getLogger("sdf2usd")

# Supported input formats for Isaac Sim's asset converter
SUPPORTED_MESH_FORMATS = {".dae", ".stl", ".obj", ".fbx", ".gltf", ".glb"}


class MeshConverter:
    """Convert mesh files to USD format using omni.kit.asset_converter.

    Caches results to avoid redundant conversions.
    """

    def __init__(self, output_dir: str):
        """
        Args:
            output_dir: Directory to store converted USD meshes.
        """
        self.output_dir = os.path.join(output_dir, "meshes")
        os.makedirs(self.output_dir, exist_ok=True)
        self._cache: dict[str, str] = {}  # source_path -> usd_path
        self._converter = None

    def convert(self, source_path: str) -> str:
        """Convert a mesh file to USD.

        Args:
            source_path: Absolute path to the source mesh file.

        Returns:
            Absolute path to the converted USD file.

        Raises:
            FileNotFoundError: If source file doesn't exist.
            RuntimeError: If conversion fails.
        """
        source_path = os.path.abspath(source_path)

        if not os.path.exists(source_path):
            raise FileNotFoundError(f"Mesh file not found: {source_path}")

        # Check cache
        if source_path in self._cache:
            logger.debug(f"  Cache hit: {os.path.basename(source_path)}")
            return self._cache[source_path]

        ext = os.path.splitext(source_path)[1].lower()
        if ext not in SUPPORTED_MESH_FORMATS:
            raise RuntimeError(
                f"Unsupported mesh format '{ext}'. "
                f"Supported: {', '.join(sorted(SUPPORTED_MESH_FORMATS))}"
            )

        # Determine output path
        base_name = os.path.splitext(os.path.basename(source_path))[0]
        usd_path = os.path.join(self.output_dir, f"{base_name}.usd")

        # Handle name collisions
        counter = 1
        while usd_path in self._cache.values():
            usd_path = os.path.join(self.output_dir, f"{base_name}_{counter}.usd")
            counter += 1

        # Copy texture files that might be alongside the source
        self._copy_adjacent_textures(source_path)

        # Convert using Isaac Sim's asset converter
        logger.info(f"  Converting: {os.path.basename(source_path)} -> {os.path.basename(usd_path)}")
        self._run_conversion(source_path, usd_path)

        self._cache[source_path] = usd_path
        return usd_path

    def copy_texture(self, source_path: str) -> str:
        """Copy a texture file to the output meshes directory.

        Returns:
            Path to the copied texture file.
        """
        if not os.path.exists(source_path):
            logger.warning(f"Texture not found: {source_path}")
            return source_path

        dest = os.path.join(self.output_dir, os.path.basename(source_path))
        if not os.path.exists(dest):
            shutil.copy2(source_path, dest)
            logger.debug(f"  Copied texture: {os.path.basename(source_path)}")
        return dest

    def _copy_adjacent_textures(self, mesh_path: str):
        """Copy common texture files from the same directory as the mesh."""
        mesh_dir = os.path.dirname(mesh_path)
        texture_exts = {".png", ".jpg", ".jpeg", ".tga", ".bmp", ".tiff", ".hdr"}
        for fname in os.listdir(mesh_dir):
            if os.path.splitext(fname)[1].lower() in texture_exts:
                src = os.path.join(mesh_dir, fname)
                dst = os.path.join(self.output_dir, fname)
                if not os.path.exists(dst):
                    shutil.copy2(src, dst)

    def _run_conversion(self, source_path: str, output_path: str):
        """Run the actual mesh conversion via omni.kit.asset_converter."""
        import omni.kit.asset_converter

        async def _convert():
            ctx = omni.kit.asset_converter.AssetConverterContext()
            ctx.ignore_materials = False
            ctx.ignore_cameras = True
            ctx.ignore_animations = True
            ctx.ignore_light = True
            ctx.export_preview_surface = False
            ctx.use_meter_as_world_unit = True
            ctx.embed_textures = True

            instance = omni.kit.asset_converter.get_instance()
            task = instance.create_converter_task(
                source_path, output_path,
                progress_callback=lambda p: None,
                asset_converter_context=ctx,
            )
            success = await task.wait_until_finished()
            if not success:
                error_msg = task.get_error_message()
                raise RuntimeError(
                    f"Mesh conversion failed for {source_path}: {error_msg}"
                )

        asyncio.get_event_loop().run_until_complete(_convert())
