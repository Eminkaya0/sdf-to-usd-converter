#!/bin/bash
# Example: Convert the Mini Talon V-Tail Gazebo model to USD for Isaac Sim.
#
# Prerequisites:
#   - NVIDIA Isaac Sim installed
#   - Mini Talon model downloaded (adjust MODEL_SDF path below)
#
# Usage:
#   chmod +x convert_mini_talon.sh
#   ./convert_mini_talon.sh

ISAAC_SIM_PYTHON="/home/emin/isaac-sim/python.sh"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Input SDF
MODEL_SDF="/home/emin/İndirilenler/mini_talon_vtail/model.sdf"

# Output directory
OUTPUT_DIR="$SCRIPT_DIR/output"
mkdir -p "$OUTPUT_DIR"

echo "Converting Mini Talon V-Tail SDF -> USD..."
echo ""

$ISAAC_SIM_PYTHON -m sdf_to_usd \
    "$MODEL_SDF" \
    "$OUTPUT_DIR/mini_talon_vtail.usda" \
    --verbose

echo ""
echo "Output: $OUTPUT_DIR/mini_talon_vtail.usda"
