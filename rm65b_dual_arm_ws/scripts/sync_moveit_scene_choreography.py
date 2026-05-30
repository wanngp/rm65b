#!/usr/bin/env python3
"""Deprecated entry point.

The acceptance recording path no longer supports scripted prop choreography.
Use physical_interaction_controller.py with RuntimeLinkAttacher evidence instead.
"""

from __future__ import annotations

import sys


def main() -> int:
    print(
        "sync_moveit_scene_choreography.py is disabled; use "
        "physical_interaction_controller.py for contact/attach-only recordings.",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
