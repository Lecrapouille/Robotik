# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin DualArm. Paths are relative to this directory.

TARGETS += robotik_dual_arm
robotik_dual_arm.DESCRIPTION := Two arms planned in parallel
robotik_dual_arm.SOURCES := src/*.cpp
$(eval $(call ROBOTIK_PLUGIN,robotik_dual_arm,DualArm))
