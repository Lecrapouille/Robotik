# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_pick_and_place
TARGET_DESCRIPTION := Pick and place demonstrations
PLUGIN_NAME := PickAndPlaceBT
PLUGIN_DIR := $(P)/demos/PickAndPlaceBT
include $(M)/project/Makefile

LIB_FILES := $(PLUGIN_DIR)/Plugin.cpp $(PLUGIN_DIR)/ColorDetector.cpp
LIB_FILES += $(PLUGIN_DIR)/PickPlaceMission.cpp

include $(PLUGIN_DIR)/../plugin.mk
