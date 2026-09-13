#!/usr/bin/env python3
"""Build aligned REAL/SIM cameras, LiDAR and schematic top-down panels."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from carla_reconstruction.visualization.panels import main


if __name__ == "__main__":
    main(full=True)
