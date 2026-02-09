"""CLI entry point for sdf-to-usd-converter."""

import sys
import argparse
import logging


def main():
    parser = argparse.ArgumentParser(
        prog="sdf2usd",
        description="Convert Gazebo SDF robot models to USD format for NVIDIA Isaac Sim.",
        epilog=(
            "Examples:\n"
            "  sdf2usd model.sdf output.usda\n"
            "  sdf2usd model.sdf output.usda --no-physics\n"
            "  sdf2usd model.sdf output.usda --scale 0.01 --up-axis Y\n"
            "\n"
            "With Isaac Sim:\n"
            "  /path/to/isaac-sim/python.sh -m sdf_to_usd model.sdf output.usda\n"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "input",
        help="Path to the input SDF file (model.sdf)",
    )
    parser.add_argument(
        "output",
        help="Path for the output USD file (e.g., model.usda or model.usd)",
    )
    parser.add_argument(
        "--no-physics",
        action="store_true",
        help="Skip physics properties (joints, inertia, rigid bodies)",
    )
    parser.add_argument(
        "--no-collision",
        action="store_true",
        help="Skip collision geometry",
    )
    parser.add_argument(
        "--merge-fixed-joints",
        action="store_true",
        help="Merge links connected by fixed joints into a single link",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Uniform scale factor (default: 1.0)",
    )
    parser.add_argument(
        "--up-axis",
        choices=["Y", "Z"],
        default="Z",
        help="Stage up axis (default: Z, standard for Isaac Sim)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable verbose (debug) logging",
    )
    parser.add_argument(
        "-q", "--quiet",
        action="store_true",
        help="Suppress all output except errors",
    )

    args = parser.parse_args()

    # Configure logging
    log_level = logging.DEBUG if args.verbose else (logging.ERROR if args.quiet else logging.INFO)
    logging.basicConfig(
        level=log_level,
        format="%(message)s",
        stream=sys.stdout,
    )

    # Initialize Isaac Sim (headless)
    logger = logging.getLogger("sdf2usd")

    if not args.quiet:
        logger.info("Starting Isaac Sim (headless)...")

    try:
        from isaacsim import SimulationApp
        simulation_app = SimulationApp({"headless": True})
    except ImportError:
        logger.error(
            "ERROR: Isaac Sim not found.\n"
            "\n"
            "This tool requires NVIDIA Isaac Sim. Run it using Isaac Sim's Python:\n"
            "  /path/to/isaac-sim/python.sh -m sdf_to_usd <input.sdf> <output.usda>\n"
            "\n"
            "Or install Isaac Sim and ensure it's on your PYTHONPATH.\n"
        )
        sys.exit(1)

    try:
        from .converter import SdfToUsdConverter

        converter = SdfToUsdConverter(
            sdf_path=args.input,
            output_path=args.output,
            up_axis=args.up_axis,
            scale=args.scale,
            include_physics=not args.no_physics,
            include_collision=not args.no_collision,
            merge_fixed_joints=args.merge_fixed_joints,
        )

        output_path = converter.convert()

        if not args.quiet:
            logger.info(f"\nDone! USD file: {output_path}")

    except FileNotFoundError as e:
        logger.error(f"ERROR: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"ERROR: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)
    finally:
        simulation_app.close()


# Allow running as: python -m sdf_to_usd
if __name__ == "__main__":
    main()
