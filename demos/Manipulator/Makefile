# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_manipulator
TARGET_DESCRIPTION := Manipulator demonstrations sharing one plugin
PLUGIN_NAME := Manipulator
PLUGIN_DIR := $(P)/demos/Manipulator
include $(M)/project/Makefile

LIB_FILES := $(PLUGIN_DIR)/Plugin.cpp $(PLUGIN_DIR)/ManipulatorMission.cpp

include $(PLUGIN_DIR)/../plugin.mk
