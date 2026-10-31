# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin PickAndPlace. Paths are relative to this directory.

TARGETS += robotik_pick_and_place
robotik_pick_and_place.DESCRIPTION := Pick and place demonstrations
robotik_pick_and_place.SOURCES := src/*.cpp
$(eval $(call ROBOTIK_PLUGIN,robotik_pick_and_place,PickAndPlace))
