# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-Fly
TARGET_DESCRIPTION := Demo: a fly brain closed loop, headless or in Compages
include $(M)/project/Makefile

APP_DIR := $(P)/demos/FlyBrain/app
DEMO_DIR := $(P)/demos/FlyBrain
INCLUDES += $(APP_DIR) $(DEMO_DIR) $(P)/include $(P)/src
VPATH += $(APP_DIR) $(DEMO_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(APP_DIR)/main.cpp
SRC_FILES += $(filter-out $(DEMO_DIR)/Plugin.cpp,$(wildcard $(DEMO_DIR)/*.cpp))

PKG_LIBS += glfw3 glew gl

BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

include $(M)/rules/Makefile
