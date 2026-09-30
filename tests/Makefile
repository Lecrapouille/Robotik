# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Location of the project directory and Makefiles
#
P := ..
M := $(P)/.makefile

###################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := $(PROJECT_NAME)-tests
TARGET_DESCRIPTION := Unit tests for $(PROJECT_NAME)
COMPILATION_MODE := test
CXX_STANDARD := --std=c++20
include $(M)/project/Makefile

###############################################################################
# Eigen3 is set by Makefile.common. These relaxations remain needed by the
# templates it expands inside our own code.
#
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set units library
#
INCLUDES += $(THIRD_PARTIES_DIR) $(P)/include $(P)/src

###############################################################################
# Set OpenGL libraries pulled in by Compages.
#
PKG_LIBS += glfw3 glew gl

###############################################################################
# Set SWI-Prolog library, needed by the Core sources compiled below
#
PKG_LIBS += swipl

###############################################################################
# Set pugixml library
#
SRC_FILES += $(THIRD_PARTIES_DIR)/pugixml/src/pugixml.cpp
INCLUDES += $(THIRD_PARTIES_DIR)/pugixml/src
USER_CXXFLAGS += -Wno-switch-enum -Wno-null-dereference

###################################################
# Set BlackThorn. Skill nodes are thin wrappers over it. The tests compile the
# engine sources directly, the same way robotik-core does. Oakular and Dear
# ImGui left with the old viewer.
#
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk

INCLUDES += $(BLACKTHORN_INCLUDES)
VPATH += $(BLACKTHORN_VPATH)
SRC_FILES += $(BLACKTHORN_SOURCES)
DEFINES += $(BLACKTHORN_DEFINES)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
THIRD_PARTIES_LIBS += $(BLACKTHORN_STATIC_DEPS)

###############################################################################
# GTest and GMock
#
PKG_LIBS += gtest gmock

###################################################
# Compile the unit tests
#
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Core,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Backends,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Behavior,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Model,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Runtime,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Skills,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Systems,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/tests,*.cpp)

INCLUDES += $(P)/src $(P)/tests
VPATH += $(P)/src

###################################################
# Generic Makefile rules
#
include $(M)/rules/Makefile

###################################################
# Build rapidyaml on demand, as done by src/Robotik/Core/Makefile.
#
$(BLACKTHORN_STATIC_DEPS):
	@$(MAKE) --no-print-directory -C $(P) compile-external-libs

$(TARGET_BINARY_NAME): $(BLACKTHORN_STATIC_DEPS)
