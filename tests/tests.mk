# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Unit tests, with Google Test, also of the sources of three plugins. Paths
# are relative to this directory.

TARGETS += RobotIK-tests
RobotIK-tests.TYPE := test
RobotIK-tests.DESCRIPTION := Unit tests for RobotIK
RobotIK-tests.SOURCES := **/*.cpp \
    ../demos/PickAndPlace/src/ColorDetector.cpp ../demos/PickAndPlace/src/PickPlaceLogic.cpp \
    ../demos/PickAndPlace/src/PickPlaceSkills.cpp ../demos/PickAndPlace/src/PickPlaceControl.cpp \
    ../demos/PickAndPlace/src/PickPlacePlanning.cpp \
    ../demos/Painter/src/PainterLogic.cpp ../demos/Painter/src/PainterSkills.cpp \
    ../demos/FlyBrain/src/FlyScenario.cpp ../demos/FlyBrain/src/FlyVision.cpp \
    ../demos/FlyBrain/src/FlyController.cpp ../demos/FlyBrain/src/ShiuNetwork.cpp \
    ../demos/FlyBrain/src/FlyBrain.cpp ../demos/FlyBrain/src/FlyEnvironment.cpp \
    ../demos/FlyBrain/src/FlyTrace.cpp ../demos/FlyBrain/src/FlyLogic.cpp
RobotIK-tests.INCLUDES := . ../demos/PickAndPlace/src ../demos/FlyBrain/src ../demos/Painter/src
RobotIK-tests.DEPENDS := robotik-core Compages
RobotIK-tests.PKG_CONFIG := gtest gmock glfw3 glew gl
RobotIK-tests.CXXFLAGS := -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches \
    -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal -Wno-switch-enum -Wno-null-dereference
RobotIK-tests.LIBS := -ldl
