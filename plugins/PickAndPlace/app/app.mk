# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Demo: pick and place as N parallel reinforcement learning environments.

TARGETS += Robotik-PickAndPlaceRL
Robotik-PickAndPlaceRL.DESCRIPTION := Demo: pick and place as N parallel reinforcement learning environments
Robotik-PickAndPlaceRL.SOURCES := *.cpp ../src/ColorDetector.cpp ../src/PickPlaceMission.cpp \
    ../src/PickPlaceSkills.cpp ../src/PickPlaceControl.cpp ../src/PickPlacePlanning.cpp
Robotik-PickAndPlaceRL.INCLUDES := . ../src
Robotik-PickAndPlaceRL.DEPENDS := robotik-core
Robotik-PickAndPlaceRL.LIBS := -ldl
