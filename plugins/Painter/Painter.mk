# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin Painter. Paths are relative to this directory.

TARGETS += robotik_painter
robotik_painter.DESCRIPTION := Robotic painter demonstration
robotik_painter.SOURCES := src/*.cpp
$(eval $(call ROBOTIK_PLUGIN,robotik_painter,Painter))
