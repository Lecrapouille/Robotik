# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Headless robot runtime (Pinocchio + MuJoCo + Compages). Paths are
# relative to this directory.

TARGETS += RobotIK-Headless
RobotIK-Headless.DESCRIPTION := Headless robot runtime (Pinocchio + MuJoCo + Compages)
RobotIK-Headless.SOURCES := main.cpp
RobotIK-Headless.INCLUDES := .
RobotIK-Headless.DEPENDS := robotik-core
RobotIK-Headless.LIBS := -ldl
