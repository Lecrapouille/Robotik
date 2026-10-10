# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_pick_and_place_rl
TARGET_DESCRIPTION := Pick and place reinforcement learning demonstration
PLUGIN_NAME := PickAndPlaceRL
PLUGIN_DIR := $(P)/demos/PickAndPlaceRL
PICK_DIR := $(P)/demos/PickAndPlaceBT
include $(M)/project/Makefile

INCLUDES += $(PICK_DIR)
LIB_FILES := $(wildcard $(PLUGIN_DIR)/*.cpp)
LIB_FILES += $(PICK_DIR)/PickPlaceControl.cpp

include $(PLUGIN_DIR)/../plugin.mk
