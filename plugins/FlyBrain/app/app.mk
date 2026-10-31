# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Demo: a fly brain closed loop, headless or in Compages.

TARGETS += Robotik-Fly
Robotik-Fly.DESCRIPTION := Demo: a fly brain closed loop, headless or in Compages
Robotik-Fly.SOURCES := main.cpp $(filter-out ../src/Plugin.cpp,$(patsubst $(HERE)%,%,$(wildcard $(HERE)../src/*.cpp)))
Robotik-Fly.INCLUDES := . ../src
Robotik-Fly.DEPENDS := robotik-core
Robotik-Fly.PKG_CONFIG := glfw3 glew gl
