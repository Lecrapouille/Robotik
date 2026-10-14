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
USER_LDFLAGS += -ldl

###################################################
# Compile the unit tests
#
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Core,*.cpp)
ROBOTIK_MODULES := Assets Backends Environment Perception Plugin Robot Scene
ROBOTIK_MODULES += Runtime Scenario Sensors Skills Systems
SRC_FILES += $(foreach m,$(ROBOTIK_MODULES),$(call rwildcard,$(P)/src/Robotik/$(m),*.cpp))
SRC_FILES += $(call rwildcard,$(P)/tests,*.cpp)
PICK_DIR := $(P)/plugins/PickAndPlaceBT/src
FLY_DIR := $(P)/plugins/FlyBrain/src
SRC_FILES += $(PICK_DIR)/ColorDetector.cpp $(PICK_DIR)/PickPlaceMission.cpp
SRC_FILES += $(PICK_DIR)/PickPlaceSkills.cpp $(PICK_DIR)/PickPlaceControl.cpp
PAINT_DIR := $(P)/plugins/Painter/src
SRC_FILES += $(PAINT_DIR)/PainterMission.cpp $(PAINT_DIR)/PainterSkills.cpp
SRC_FILES += $(FLY_DIR)/FlyScenario.cpp $(FLY_DIR)/FlyVision.cpp
SRC_FILES += $(FLY_DIR)/FlyController.cpp $(FLY_DIR)/ShiuNetwork.cpp
SRC_FILES += $(FLY_DIR)/FlyBrain.cpp $(FLY_DIR)/FlyEnvironment.cpp
SRC_FILES += $(FLY_DIR)/FlyTrace.cpp $(FLY_DIR)/FlyMission.cpp

INCLUDES += $(P)/src $(P)/tests $(PICK_DIR) $(FLY_DIR) $(PAINT_DIR)
VPATH += $(P)/src

###################################################
# Generic Makefile rules
#
include $(M)/rules/Makefile
include $(P)/Makefile.relink

###################################################
# Build rapidyaml on demand (rule in Makefile.common).
#
$(TARGET_BINARY_NAME): $(BLACKTHORN_STATIC_DEPS)
