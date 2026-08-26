#!/usr/bin/env python3
"""Compatibility dispatcher for the two controller-specific variant tools.

Prefer invoking ``generate_sumo_safety_variants.py`` or
``generate_carla_safety_variants.py`` directly.  The former selected-critical-
actor braking workflow has been retired because it mixed CARLA interventions
with a SUMO background and did not define a controller-pure baseline.
"""

import argparse
import os
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "pipeline", choices=("sumo-hybrid", "carla-only"),
        help="controller-pure variant pipeline")
    args, remainder = parser.parse_known_args(argv)
    if "--critical-actor" in remainder:
        parser.error(
            "--critical-actor belongs to the retired mixed-control workflow; "
            "the SUMO pipeline now varies every moving SUMO vehicle")
    if args.pipeline == "sumo-hybrid":
        from carla_reconstruction.tools import generate_sumo_safety_variants
        generate_sumo_safety_variants.main(remainder)
    else:
        from carla_reconstruction.tools import generate_carla_safety_variants
        generate_carla_safety_variants.main(remainder)


if __name__ == "__main__":
    main()
