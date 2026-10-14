# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-LineFollower
TARGET_DESCRIPTION := Demo: AprilTag localization and line following (OpenCV)
include $(M)/project/Makefile

APP_DIR := $(P)/plugins/LineFollower/app
DEMO_DIR := $(P)/plugins/LineFollower/src
INCLUDES += $(APP_DIR) $(DEMO_DIR) $(P)/include $(P)/src
VPATH += $(APP_DIR) $(DEMO_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(APP_DIR)/main.cpp
SRC_FILES += $(filter-out $(DEMO_DIR)/Plugin.cpp,$(wildcard $(DEMO_DIR)/*.cpp))

BLACKTHORN_DIR := $(ROBOTIK_EXTERNAL)/BlackThorn
RYML_DIR := $(ROBOTIK_EXTERNAL)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

USER_CXXFLAGS += $(patsubst -I%,-isystem %,$(shell pkg-config --cflags-only-I opencv4))
USER_LDFLAGS += -Wl,-rpath-link,$(shell pkg-config --variable=libdir opencv4)
USER_LDFLAGS += $(shell pkg-config --libs-only-L opencv4) -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_core
# Cloned and built by the plugin (dependencies of plugins/LineFollower/plugin.yaml).
APRILTAG_DIR := $(P)/plugins/LineFollower/external/apriltag
APRILTAG_LIB := $(abspath $(APRILTAG_DIR)/build/libapriltag.a)
INCLUDES += $(APRILTAG_DIR)
THIRD_PARTIES_LIBS += $(APRILTAG_LIB)
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion
USER_CXXFLAGS += -Wno-double-promotion -Wno-useless-cast

include $(M)/rules/Makefile
include $(P)/Makefile.relink

$(TARGET_BINARY_NAME): $(APRILTAG_LIB)

$(APRILTAG_LIB):
	$(MAKE) --no-print-directory -C $(P)/plugins/LineFollower $@
