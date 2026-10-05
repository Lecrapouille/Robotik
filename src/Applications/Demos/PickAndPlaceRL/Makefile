# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../../../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-PickAndPlaceRL
TARGET_DESCRIPTION := Demo: pick and place as N parallel reinforcement learning environments
include $(M)/project/Makefile

CURRENT_DIR := $(P)/src/Applications/Demos/PickAndPlaceRL
INCLUDES += $(CURRENT_DIR) $(P)/include $(P)/src
VPATH += $(CURRENT_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(CURRENT_DIR)/main.cpp $(CURRENT_DIR)/PickPlaceEnvironment.cpp

# Headers only: BlackThorn is compiled into robotik-core.
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

include $(M)/rules/Makefile
