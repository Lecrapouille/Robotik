# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Demo: AprilTag localization and line following, without a window. The
# simulator shows the same run in 3D; this program writes the map in a PNG
# (stb_image_write, as the simulator screenshots).

ifneq ($(filter robotik_line_follower,$(TARGETS)),)
    TARGETS += RobotIK-LineFollower
    RobotIK-LineFollower.DESCRIPTION := Demo: AprilTag localization and line following, headless
    RobotIK-LineFollower.SOURCES := main.cpp $(filter-out ../src/Plugin.cpp,$(patsubst $(HERE)%,%,$(wildcard $(HERE)../src/*.cpp)))
    RobotIK-LineFollower.INCLUDES := . ../src
    RobotIK-LineFollower.DEPENDS := robotik-core apriltag stb
    RobotIK-LineFollower.LIBS := $(LINE_FOLLOWER_OPENCV_LIBS)
    RobotIK-LineFollower.CXXFLAGS := $(robotik_line_follower.CXXFLAGS)
endif
