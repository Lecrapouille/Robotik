# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-Headless
TARGET_DESCRIPTION := Headless robot runtime (Pinocchio + MuJoCo + Compages)
include $(M)/project/Makefile

CURRENT_DIR := $(P)/headless
INCLUDES += $(CURRENT_DIR) $(P)/include $(P)/src
VPATH += $(CURRENT_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(CURRENT_DIR)/main.cpp

BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

USER_LDFLAGS += -ldl

include $(M)/rules/Makefile
