# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Reads the dependencies: section of a plugin.yaml, one entry per line:
#
#   dependencies:
#     system:
#       - { name: opencv4, license: Apache-2.0 }
#     thirdparties:
#       - { repo: AprilRobotics/apriltag, license: BSD-2-Clause, description: AprilTag }
#       - { name: onnxruntime, url: https://..., license: MIT }
#
# -v section=thirdparties  prints a MyMakefile manifest: the entries with a
#                          repo (user/repo[:recurse][@ref]), each with its
#                          description and license as comments;
# -v section=system        prints the system names (pkg-config packages).
#
# A value holding a comma must be quoted ("..."), as YAML wants it anyway.

# Value of @key in a flow map line, or "".
function field(line, key,    rest, value)
{
    if (!match(line, "[{,][ \t]*" key ":[ \t]*"))
    {
        return ""
    }
    rest = substr(line, RSTART + RLENGTH)
    if (substr(rest, 1, 1) == "\"")
    {
        rest = substr(rest, 2)
        return substr(rest, 1, index(rest, "\"") - 1)
    }
    match(rest, /^[^,}]*/)
    value = substr(rest, 1, RLENGTH)
    sub(/[ \t]+$/, "", value)
    return value
}

/^dependencies:/ { inside = 1; next }
inside && /^[^ \t#]/ { inside = 0 }
inside && /^  [A-Za-z_]+:/ {
    current = $1
    sub(/:.*/, "", current)
    next
}
inside && current == section && /^ +- / {
    if (section == "system")
    {
        name = field($0, "name")
        if (name == "" && $0 !~ /\{/)
        {
            name = $0
            sub(/^ +- +/, "", name)
            sub(/[ \t]+$/, "", name)
        }
        if (name != "")
        {
            print name
        }
        next
    }
    repo = field($0, "repo")
    if (repo == "")
    {
        next
    }
    description = field($0, "description")
    if (description != "")
    {
        print "### " description
    }
    print "### License: " field($0, "license")
    print repo
    print ""
}
