# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Unit tests, with Google Test, also of the sources of three plugins. Paths
# are relative to this directory.

TARGETS += Robotik-tests
Robotik-tests.TYPE := test
Robotik-tests.DESCRIPTION := Unit tests for Robotik
Robotik-tests.SOURCES := **/*.cpp \
    ../plugins/PickAndPlace/src/ColorDetector.cpp ../plugins/PickAndPlace/src/PickPlaceMission.cpp \
    ../plugins/PickAndPlace/src/PickPlaceSkills.cpp ../plugins/PickAndPlace/src/PickPlaceControl.cpp \
    ../plugins/PickAndPlace/src/PickPlacePlanning.cpp \
    ../plugins/Painter/src/PainterMission.cpp ../plugins/Painter/src/PainterSkills.cpp \
    ../plugins/FlyBrain/src/FlyScenario.cpp ../plugins/FlyBrain/src/FlyVision.cpp \
    ../plugins/FlyBrain/src/FlyController.cpp ../plugins/FlyBrain/src/ShiuNetwork.cpp \
    ../plugins/FlyBrain/src/FlyBrain.cpp ../plugins/FlyBrain/src/FlyEnvironment.cpp \
    ../plugins/FlyBrain/src/FlyTrace.cpp ../plugins/FlyBrain/src/FlyMission.cpp
Robotik-tests.INCLUDES := . ../plugins/PickAndPlace/src ../plugins/FlyBrain/src ../plugins/Painter/src
Robotik-tests.DEPENDS := robotik-core
Robotik-tests.PKG_CONFIG := gtest gmock glfw3 glew gl
Robotik-tests.CXXFLAGS := -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches \
    -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal -Wno-switch-enum -Wno-null-dereference
Robotik-tests.LIBS := -ldl
