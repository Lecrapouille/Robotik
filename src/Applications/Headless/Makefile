# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-Headless
TARGET_DESCRIPTION := Headless robot runtime (Pinocchio + MuJoCo + Compages)
include $(M)/project/Makefile

CURRENT_DIR := $(P)/src/Applications/Headless
PICK_DIR := $(P)/src/Applications/Demos/PickAndPlace
LINE_DIR := $(P)/src/Applications/Demos/LineFollower
INCLUDES += $(CURRENT_DIR) $(PICK_DIR) $(LINE_DIR) $(P)/include $(P)/src
VPATH += $(CURRENT_DIR) $(PICK_DIR) $(LINE_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(CURRENT_DIR)/main.cpp
SRC_FILES += $(PICK_DIR)/ColorDetector.cpp $(PICK_DIR)/PickPlaceMission.cpp
SRC_FILES += $(LINE_DIR)/Track.cpp $(LINE_DIR)/DiffDrive.cpp
SRC_FILES += $(LINE_DIR)/Floor.cpp $(LINE_DIR)/Vision.cpp
SRC_FILES += $(LINE_DIR)/Skills.cpp $(LINE_DIR)/LineFollowerMission.cpp

# Headers only: BlackThorn is compiled into robotik-core.
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

USER_CXXFLAGS += $(patsubst -I%,-isystem %,$(shell pkg-config --cflags-only-I opencv4))
USER_LDFLAGS += -Wl,-rpath-link,$(shell pkg-config --variable=libdir opencv4)
USER_LDFLAGS += $(shell pkg-config --libs-only-L opencv4) -lopencv_imgproc -lopencv_core
APRILTAG_DIR := $(THIRD_PARTIES_DIR)/apriltag
INCLUDES += $(APRILTAG_DIR)
THIRD_PARTIES_LIBS += $(abspath $(APRILTAG_DIR)/build/libapriltag.a)
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion
USER_CXXFLAGS += -Wno-double-promotion -Wno-useless-cast

include $(M)/rules/Makefile
