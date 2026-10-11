# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_painter
TARGET_DESCRIPTION := Robotic painter demonstration
PLUGIN_NAME := Painter
PLUGIN_DIR := $(P)/demos/Painter
include $(M)/project/Makefile

LIB_FILES := $(PLUGIN_DIR)/Plugin.cpp $(PLUGIN_DIR)/PainterMission.cpp
LIB_FILES += $(PLUGIN_DIR)/PainterSkills.cpp

include $(PLUGIN_DIR)/../plugin.mk
