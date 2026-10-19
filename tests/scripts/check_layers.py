#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.
"""Check the layers of RobotIK: a layer only includes the layers below it.

    Common  <  Robot  <  Task  <  Package

Common holds the shared helpers; Robot one robot (joints, sensors, physics);
Task robots doing a job (skills, beliefs, plans); Package the files on disk
(YAML scenarios, asset packs, plugins). Each layer works without the ones
above it, so an application can stop at the layer it needs.

A second rule keeps the ground truth away from the robot's decisions: skills
and the planner read the beliefs (WorldModel), never the simulator truth.

Usage: check_layers.py [repository root]. Exit code 1 when a rule is broken.
"""
import pathlib
import re
import sys

LAYERS = ["Common", "Robot", "Task", "Package"]
INCLUDE = re.compile(r'#include\s+"Robotik/([A-Za-z]+)/')
# Truth API of the simulation: forbidden in what drives the robot.
TRUTH = re.compile(r"\b(Simulation|Cell)\b[^;\n]*\.\s*(pose|query|initialPose|entity)\s*\(")
NO_TRUTH = ["Task/Skills", "Task/Planning"]


def layer_of(path, root):
    rel = path.relative_to(root).parts
    # include/Robotik/<Layer>/... or src/Robotik/<Layer>/...
    if len(rel) >= 3 and rel[0] in ("include", "src") and rel[1] == "Robotik":
        return rel[2] if rel[2] in LAYERS else None
    return None


def main():
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    errors = []
    for base in ("include/Robotik", "src/Robotik"):
        for path in sorted((root / base).rglob("*")):
            if path.suffix not in (".hpp", ".cpp", ".h"):
                continue
            own = layer_of(path, root)
            if own is None:
                continue
            text = path.read_text()
            for number, line in enumerate(text.splitlines(), 1):
                found = INCLUDE.search(line)
                if found and found.group(1) in LAYERS:
                    other = found.group(1)
                    if LAYERS.index(other) > LAYERS.index(own):
                        errors.append(f"{path.relative_to(root)}:{number}: {own} includes {other}")
            rel = str(path.relative_to(root))
            if any(f"/{part}/" in rel for part in NO_TRUTH):
                for number, line in enumerate(text.splitlines(), 1):
                    if TRUTH.search(line):
                        errors.append(f"{rel}:{number}: reads the simulator truth")
    for error in errors:
        print(error)
    print(f"{len(errors)} layer rule(s) broken" if errors else "layers ok")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
