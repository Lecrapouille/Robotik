#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.
"""Check the layers of RobotIK with the compiler: a layer only sees the
layers below it.

    Common  <  Robot  <  Task  <  Package

Common holds the shared helpers; Robot one robot (joints, sensors, physics);
Task robots doing a job (skills, beliefs, plans); Package the files on disk
(YAML scenarios, asset packs, plugins). Each layer works without the ones
above it, so an application can stop at the layer it needs.

How: every header and source of a layer is compiled alone (-fsyntax-only)
with an include path where only the allowed layers exist. An include of a
layer above is then "file not found", reported by the compiler, whatever
the way the line is written. A header that does not compile alone (it
forgets an include) is found the same way.

The second rule keeps the ground truth away from the robot's decisions:
Task/Skills and Task/Planning are compiled without Task/RoboticCell.hpp, so
they cannot call the truth (RoboticCell::pose, RoboticCell::query): without
the definition of the class there is nothing to call.

Usage: check_layers.py [repository root]. Exit code 1 when a rule is broken.
"""
import os
import pathlib
import shlex
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

LAYERS = ["Common", "Robot", "Task", "Package"]
# Parts of a layer that must not see some files of their own layer.
HIDDEN = {
    "Task/Skills": ["Task/RoboticCell.hpp"],
    "Task/Planning": ["Task/RoboticCell.hpp"],
}


def compile_flags(root):
    """Flags of the core build (-std, -D, include paths), from a dry run of
    make, so that the check follows the versions of the third parties."""
    dry = subprocess.run(["make", "-n", "-B", "robotik-core"], cwd=root,
                         capture_output=True, text=True)
    lines = dry.stdout.splitlines()
    for i, line in enumerate(lines):
        if "Compiling C++: robotik-core" in line and i + 1 < len(lines):
            words = shlex.split(lines[i + 1])
            break
    else:
        sys.exit("check_layers: no compile line in 'make -n -B robotik-core'")
    compiler = [w for w in words if not w.endswith("ccache")][0]
    flags, keep = [], False
    for word in words:
        if keep:
            flags.append(word)
            keep = False
        elif word in ("-isystem", "-include"):
            flags.append(word)
            keep = True
        elif word.startswith(("-std=", "-D")) or (
                word.startswith("-I") and word not in ("-Isrc", "-Iinclude")):
            flags.append(word)
    return compiler, flags


def part_of(rel):
    """'Task/Skills' for 'Task/Skills/Skill.hpp', the layer otherwise."""
    for part in HIDDEN:
        if rel.startswith(part + "/"):
            return part
    return rel.split("/")[0]


def stage(root, build, layer, part):
    """Builds the include folders of one layer: links to the allowed
    layers, without the hidden files. Returns the folders for -I."""
    allowed = LAYERS[: LAYERS.index(layer) + 1]
    hidden = set(HIDDEN.get(part, []))
    folders = []
    for side in ("include", "src"):
        top = build / "layers" / part.replace("/", "_") / side / "Robotik"
        if top.exists():
            subprocess.run(["rm", "-rf", str(top)], check=True)
        top.mkdir(parents=True)
        for name in allowed:
            source = root / side / "Robotik" / name
            if not source.exists():
                continue
            if not any(h.startswith(name + "/") for h in hidden):
                (top / name).symlink_to(source)
                continue
            # Same layer as a hidden file: link file by file.
            for path in source.rglob("*"):
                rel = name + "/" + str(path.relative_to(source))
                if path.is_dir() or rel in hidden:
                    continue
                target = top / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                target.symlink_to(path)
        folders.append(str(top.parent))
    return folders


def main():
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    build = root / "build" / "linux-gcc-debug"
    compiler, flags = compile_flags(root)

    jobs = []
    staged = {}
    for side in ("include", "src"):
        base = root / side / "Robotik"
        for path in sorted(base.rglob("*")):
            if path.suffix not in (".hpp", ".h", ".cpp"):
                continue
            rel = str(path.relative_to(base))
            layer = rel.split("/")[0]
            if layer not in LAYERS:
                continue
            part = part_of(rel)
            if part not in staged:
                staged[part] = stage(root, build, layer, part)
            jobs.append((path, part))

    def check(job):
        path, part = job
        command = [compiler, "-fsyntax-only", "-w", "-x", "c++"] + flags
        for folder in staged[part]:
            command.append("-I" + folder)
        command.append(str(path))
        done = subprocess.run(command, cwd=root, capture_output=True, text=True)
        if done.returncode == 0:
            return None
        first = next((l for l in done.stderr.splitlines() if "error" in l),
                     done.stderr.strip())
        return f"{path.relative_to(root)}: {part} sees only " \
               f"{', '.join(LAYERS[: LAYERS.index(part.split('/')[0]) + 1])}" \
               f"{' (without ' + ', '.join(HIDDEN[part]) + ')' if part in HIDDEN else ''}" \
               f":\n    {first}"

    with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
        errors = [e for e in pool.map(check, jobs) if e]
    for error in errors:
        print(error)
    print(f"{len(errors)} layer rule(s) broken in {len(jobs)} files"
          if errors else f"layers ok ({len(jobs)} files compiled alone)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
