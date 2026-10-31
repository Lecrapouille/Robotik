# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin LineFollower: AprilTag localization and line following, with
# OpenCV (opencv4, not built without it). Paths are relative to this
# directory.

ifeq ($(call robotik-missing,opencv4),)
    TARGETS += apriltag robotik_line_follower

    ### AprilTag fiducial detection. License: BSD-2-Clause
    apriltag.GIT := https://github.com/AprilRobotics/apriltag.git
    apriltag.TYPE := cmake
    # Release, as its debug build names its library libapriltagd.a.
    apriltag.CMAKE_ARGS := -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_WRAPPER=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
    apriltag.PUBLIC_INCLUDES := include/apriltag
    apriltag.LIBRARIES := apriltag

    robotik_line_follower.DESCRIPTION := Line follower demonstration
    robotik_line_follower.SOURCES := src/*.cpp
    robotik_line_follower.DEPENDS := apriltag
    robotik_line_follower.PKG_CONFIG := opencv4
    robotik_line_follower.CXXFLAGS := -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion \
        -Wno-double-promotion -Wno-useless-cast
    $(eval $(call ROBOTIK_PLUGIN,robotik_line_follower,LineFollower))
else
    $(call print-warning,LineFollower skipped: system library $(call robotik-missing,opencv4) not found by pkg-config)
endif
