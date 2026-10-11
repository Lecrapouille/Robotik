# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_fly
TARGET_DESCRIPTION := Fly demonstration package
PLUGIN_NAME := FlyBrain
PLUGIN_DIR := $(P)/plugins/FlyBrain
PLUGIN_SRC := $(PLUGIN_DIR)/src
include $(M)/project/Makefile

# FlyView and FlyDraw belong to the standalone Robotik-Fly window only.
LIB_FILES := $(PLUGIN_SRC)/Plugin.cpp $(PLUGIN_SRC)/FlyMission.cpp
LIB_FILES += $(PLUGIN_SRC)/FlyScenario.cpp $(PLUGIN_SRC)/FlyVision.cpp
LIB_FILES += $(PLUGIN_SRC)/FlyController.cpp $(PLUGIN_SRC)/FlyEnvironment.cpp
LIB_FILES += $(PLUGIN_SRC)/FlyBrain.cpp $(PLUGIN_SRC)/ShiuNetwork.cpp

include $(P)/plugins/plugin.mk
