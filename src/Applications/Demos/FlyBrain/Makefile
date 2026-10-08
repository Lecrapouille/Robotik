# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../../../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-Fly
TARGET_DESCRIPTION := Demo: a fly brain closed loop, headless or in Compages
include $(M)/project/Makefile

CURRENT_DIR := $(P)/src/Applications/Demos/FlyBrain
INCLUDES += $(CURRENT_DIR) $(P)/include $(P)/src
VPATH += $(CURRENT_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(CURRENT_DIR)/main.cpp $(CURRENT_DIR)/FlyScenario.cpp
SRC_FILES += $(CURRENT_DIR)/FlyVision.cpp $(CURRENT_DIR)/FlyController.cpp
SRC_FILES += $(CURRENT_DIR)/ShiuNetwork.cpp $(CURRENT_DIR)/FlyBrain.cpp
SRC_FILES += $(CURRENT_DIR)/FlyEnvironment.cpp $(CURRENT_DIR)/FlyTrace.cpp
SRC_FILES += $(CURRENT_DIR)/FlyDraw.cpp $(CURRENT_DIR)/FlyView.cpp

PKG_LIBS += glfw3 glew gl

# Headers only: BlackThorn is compiled into robotik-core. The scenario parser
# uses its YAML reader.
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

include $(M)/rules/Makefile
