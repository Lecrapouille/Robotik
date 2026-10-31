# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Demo: AprilTag localization and line following (OpenCV).

ifneq ($(filter robotik_line_follower,$(TARGETS)),)
    TARGETS += Robotik-LineFollower
    Robotik-LineFollower.DESCRIPTION := Demo: AprilTag localization and line following (OpenCV)
    Robotik-LineFollower.SOURCES := main.cpp $(filter-out ../src/Plugin.cpp,$(patsubst $(HERE)%,%,$(wildcard $(HERE)../src/*.cpp)))
    Robotik-LineFollower.INCLUDES := . ../src
    Robotik-LineFollower.DEPENDS := robotik-core apriltag
    Robotik-LineFollower.PKG_CONFIG := opencv4
    Robotik-LineFollower.CXXFLAGS := $(robotik_line_follower.CXXFLAGS)
endif
