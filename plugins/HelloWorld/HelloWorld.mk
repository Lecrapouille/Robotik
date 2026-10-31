# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin HelloWorld. Paths are relative to this directory.

TARGETS += robotik_hello_world
robotik_hello_world.DESCRIPTION := Tutorial plugin: one small lesson per scenario
robotik_hello_world.SOURCES := src/*.cpp
$(eval $(call ROBOTIK_PLUGIN,robotik_hello_world,HelloWorld))
