#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.
"""Check the relative links of the Markdown files of RobotIK.

A link [text](path#anchor) must give a file that exists, and when it names
an anchor of a Markdown file, a heading of that file must make it (GitHub
rules: lower case, spaces become '-', signs and emoji are dropped).

Not checked: links to the web (http, https, mailto), code blocks and inline
code (an example of a link is not a link), and the folders external/ and
build/ (third parties and outputs).

Usage: check_links.py [repository root]. Exit code 1 when a link is broken.
"""
import pathlib
import re
import sys
import unicodedata

FENCE = re.compile(r"^\s*(```|~~~)")
HEADING = re.compile(r"^(#{1,6}) (.*?)\s*$")
LINK = re.compile(r"\]\(([^)\s]+?)\)")
INLINE_CODE = re.compile(r"`[^`]*`")
SKIPPED = {"external", "build", ".git", "node_modules"}


def anchor(p_heading):
    """GitHub anchor of a heading."""
    text = re.sub(r"\]\([^)]*\)", "", p_heading)
    text = re.sub(r"[`*\[\]]", "", text)
    out = []
    for ch in text.strip().lower():
        if ch == " ":
            out.append("-")
        elif ch in "-_" or unicodedata.category(ch)[0] in "LN":
            out.append(ch)
    return "".join(out)


def lines_out_of_code(p_file):
    """(line number, text) of the lines out of code blocks."""
    fenced = False
    for number, line in enumerate(p_file.read_text().splitlines(), 1):
        if FENCE.match(line):
            fenced = not fenced
            continue
        if not fenced:
            yield number, line


ANCHORS = {}


def anchors_of(p_file):
    """Anchors made by the headings of a file (a second same heading gets
    '-1', as on GitHub)."""
    if p_file not in ANCHORS:
        found, seen = set(), {}
        for _, line in lines_out_of_code(p_file):
            match = HEADING.match(line)
            if match:
                name = anchor(match.group(2))
                count = seen.get(name, 0)
                seen[name] = count + 1
                found.add(name if count == 0 else f"{name}-{count}")
        ANCHORS[p_file] = found
    return ANCHORS[p_file]


def main():
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    broken = 0
    for page in sorted(root.rglob("*.md")):
        if SKIPPED & set(page.relative_to(root).parts):
            continue
        for number, line in lines_out_of_code(page):
            for href in LINK.findall(INLINE_CODE.sub("", line)):
                if href.startswith(("http:", "https:", "mailto:")):
                    continue
                path, _, fragment = href.partition("#")
                target = page if path == "" else page.parent / path
                where = f"{page.relative_to(root)}:{number}"
                if not target.exists():
                    print(f"{where}: no file {href}")
                    broken += 1
                elif (fragment and target.suffix == ".md"
                      and fragment not in anchors_of(target.resolve())):
                    print(f"{where}: no anchor {href}")
                    broken += 1
    if broken:
        print(f"{broken} broken link(s)")
        return 1
    print("links ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
