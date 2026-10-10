# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_line_follower
TARGET_DESCRIPTION := Line follower demonstration
PLUGIN_NAME := LineFollower
PLUGIN_DIR := $(P)/demos/LineFollower
include $(M)/project/Makefile

LIB_FILES := $(wildcard $(PLUGIN_DIR)/*.cpp)

USER_CXXFLAGS += $(patsubst -I%,-isystem %,$(shell pkg-config --cflags-only-I opencv4))
USER_LDFLAGS += -Wl,-rpath-link,$(shell pkg-config --variable=libdir opencv4)
USER_LDFLAGS += $(shell pkg-config --libs-only-L opencv4) -lopencv_imgproc -lopencv_core
APRILTAG_DIR := $(THIRD_PARTIES_DIR)/apriltag
INCLUDES += $(APRILTAG_DIR)
THIRD_PARTIES_LIBS += $(abspath $(APRILTAG_DIR)/build/libapriltag.a)
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion
USER_CXXFLAGS += -Wno-double-promotion -Wno-useless-cast

include $(PLUGIN_DIR)/../plugin.mk
