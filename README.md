# sdf-to-usd-converter

Convert [Gazebo SDF](http://sdformat.org/) robot models to [USD](https://openusd.org/) format for [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim).

There are thousands of robot models in SDF format from the Gazebo ecosystem, but no easy way to use them in Isaac Sim. This tool bridges that gap with a simple command-line interface.

## Features

- **Full SDF parsing** — models, links, joints, visuals, collisions, inertials
- **Mesh conversion** — DAE (COLLADA), STL, OBJ, FBX, glTF/GLB via Isaac Sim's asset converter
- **Physics support** — revolute/prismatic/fixed joints, rigid bodies, mass, inertia tensors
- **Collision geometry** — mesh-based and primitive shapes (box, cylinder, sphere, capsule)
- **PBR materials** — diffuse color, metalness, roughness, albedo texture maps
- **Primitive geometry** — box, cylinder, sphere, capsule (no mesh file needed)
- **Pose handling** — full SDF pose composition with degrees/radians support
- **URI resolution** — `model://`, `file://`, relative, and absolute paths
- **Isaac Sim compatible** — Z-up axis, meters, ArticulationRootAPI, PhysicsDriveAPI

## Requirements

- **NVIDIA Isaac Sim** (2023.1.1 or later) — provides the `pxr` USD library and mesh converter
- **Python 3.10+** (included with Isaac Sim)

## Installation

```bash
# Clone the repository
git clone https://github.com/Eminkaya0/sdf-to-usd-converter.git
cd sdf-to-usd-converter

# Install with Isaac Sim's Python
/path/to/isaac-sim/python.sh -m pip install -e .
```

## Usage

### Command Line

```bash
# Basic conversion
/path/to/isaac-sim/python.sh -m sdf_to_usd model.sdf output.usda

# Visual only (no physics)
/path/to/isaac-sim/python.sh -m sdf_to_usd model.sdf output.usda --no-physics

# Skip collision geometry
/path/to/isaac-sim/python.sh -m sdf_to_usd model.sdf output.usda --no-collision

# Custom scale and verbose output
/path/to/isaac-sim/python.sh -m sdf_to_usd model.sdf output.usda --scale 1.0 --verbose
```

### Python API

```python
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

from sdf_to_usd.converter import SdfToUsdConverter

converter = SdfToUsdConverter(
    sdf_path="path/to/model.sdf",
    output_path="path/to/output.usda",
    include_physics=True,
    include_collision=True,
)
converter.convert()

app.close()
```

### CLI Options

| Option | Description | Default |
|--------|-------------|---------|
| `input` | Path to SDF file | (required) |
| `output` | Path for output USD | (required) |
| `--no-physics` | Skip joints, inertia, rigid bodies | `False` |
| `--no-collision` | Skip collision geometry | `False` |
| `--merge-fixed-joints` | Merge fixed-joint links | `False` |
| `--scale FLOAT` | Uniform scale factor | `1.0` |
| `--up-axis {Y,Z}` | Stage up axis | `Z` |
| `-v, --verbose` | Debug logging | `False` |
| `-q, --quiet` | Errors only | `False` |

## Supported SDF Elements

| Element | Status | Notes |
|---------|--------|-------|
| `<model>` | Supported | Single model per file |
| `<link>` | Supported | With pose, inertial |
| `<visual>` | Supported | Mesh + primitive geometry |
| `<collision>` | Supported | Mesh + primitive geometry |
| `<joint>` revolute | Supported | Axis, limits, damping |
| `<joint>` prismatic | Supported | Axis, limits, damping |
| `<joint>` fixed | Supported | — |
| `<joint>` ball | Partial | Converted to fixed |
| `<inertial>` | Supported | Mass, inertia tensor, CoM |
| `<material>` | Supported | Diffuse, specular, PBR |
| `<geometry>` mesh | Supported | DAE, STL, OBJ, FBX, glTF |
| `<geometry>` box | Supported | — |
| `<geometry>` cylinder | Supported | — |
| `<geometry>` sphere | Supported | — |
| `<geometry>` capsule | Supported | — |
| `<include>` | Not yet | Planned for v0.2 |
| `<world>` | Not yet | Planned for v0.2 |
| `<sensor>` | Not yet | Planned for v0.2 |
| `<plugin>` | Not supported | Simulation-specific |

## USD Output Structure

The converter creates a structured USD hierarchy following Isaac Sim conventions:

```
/<model_name>/                      (Xform + ArticulationRootAPI)
  Looks/                            (Material scope)
    material_0/                     (UsdPreviewSurface)
  <link_name>/                      (Xform + RigidBodyAPI + MassAPI)
    visuals/
      <visual_name>/                (Xform → references converted mesh USD)
    collisions/
      <collision_name>/             (Xform + CollisionAPI)
  <joint_name>/                     (PhysicsRevoluteJoint / PhysicsPrismaticJoint)
```

## Example: Converting a Gazebo Model

```bash
# Download a model from Gazebo Fuel or the model database
# For example, the Mini Talon V-Tail fixed-wing aircraft

/path/to/isaac-sim/python.sh -m sdf_to_usd \
    ~/models/mini_talon_vtail/model.sdf \
    ~/usd_models/mini_talon_vtail.usda \
    --verbose
```

Output:
```
============================================================
  sdf-to-usd-converter v0.1.0
============================================================
  Input:  /home/user/models/mini_talon_vtail/model.sdf
  Output: /home/user/usd_models/mini_talon_vtail.usda
  Physics: Yes
  Collision: Yes

[1/4] Parsing SDF...
  Parsed model 'mini_talon_vtail': 7 links, 6 joints
  Found 23 mesh files to convert
[2/4] Converting meshes...
  Converted 23/23 unique meshes
[3/4] Building USD stage...
  USD saved: mini_talon_vtail.usda (245.3 KB)
[4/4] Done!
============================================================
```

## How It Works

1. **Parse SDF** — Uses `xml.etree.ElementTree` to extract the model hierarchy, geometry, materials, physics, and pose data into Python dataclasses.
2. **Convert meshes** — Uses Isaac Sim's `omni.kit.asset_converter` to convert DAE/STL/OBJ mesh files to USD format. Meshes are cached to avoid redundant conversion.
3. **Build USD stage** — Uses the `pxr` (OpenUSD) Python API to construct the USD prim hierarchy with proper transforms, references, materials, and physics schemas.
4. **Save** — Writes the final `.usda` (ASCII) or `.usd` (binary) file.

## Comparison with Other Tools

| Tool | Language | Dependencies | Physics | Materials | Ease of Use |
|------|----------|-------------|---------|-----------|-------------|
| **sdf-to-usd-converter** | Python | Isaac Sim | Full | PBR | `pip install` + CLI |
| [gz-usd](https://github.com/gazebosim/gz-usd) | C++ | sdformat, gz-sim, USD libs | Partial | Basic | Build from source |
| [sdf2usd_docker](https://github.com/kuralme/sdf2usd_docker) | Docker | Docker | Partial | Basic | Docker pull |
| Manual (Blender) | — | Blender | None | Partial | Manual per model |

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

Areas where help is needed:
- Testing with more SDF models from Gazebo Fuel
- `<include>` and `<world>` support
- Sensor conversion (camera, lidar, IMU)
- Improved material/texture handling

## License

MIT License. See [LICENSE](LICENSE) for details.
