# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Demo: pick and place as N parallel reinforcement learning environments.

TARGETS += RobotIK-PickAndPlaceRL
RobotIK-PickAndPlaceRL.DESCRIPTION := Demo: pick and place as N parallel reinforcement learning environments
RobotIK-PickAndPlaceRL.SOURCES := *.cpp ../src/ColorDetector.cpp ../src/PickPlaceLogic.cpp \
    ../src/PickPlaceSkills.cpp ../src/PickPlaceControl.cpp ../src/PickPlacePlanning.cpp
RobotIK-PickAndPlaceRL.INCLUDES := . ../src
RobotIK-PickAndPlaceRL.DEPENDS := robotik-core
RobotIK-PickAndPlaceRL.LIBS := -ldl
