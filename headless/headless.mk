# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Headless robot runtime (Pinocchio + MuJoCo + Compages). Paths are
# relative to this directory.

TARGETS += Robotik-Headless
Robotik-Headless.DESCRIPTION := Headless robot runtime (Pinocchio + MuJoCo + Compages)
Robotik-Headless.SOURCES := main.cpp
Robotik-Headless.INCLUDES := .
Robotik-Headless.DEPENDS := robotik-core
Robotik-Headless.LIBS := -ldl
