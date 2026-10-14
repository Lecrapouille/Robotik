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
PLUGIN_DIR := $(P)/plugins/LineFollower
PLUGIN_SRC := $(PLUGIN_DIR)/src
include $(M)/project/Makefile

LIB_FILES := $(wildcard $(PLUGIN_SRC)/*.cpp)

USER_CXXFLAGS += $(patsubst -I%,-isystem %,$(shell pkg-config --cflags-only-I opencv4))
USER_LDFLAGS += -Wl,-rpath-link,$(shell pkg-config --variable=libdir opencv4)
USER_LDFLAGS += $(shell pkg-config --libs-only-L opencv4) -lopencv_imgproc -lopencv_core
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion
USER_CXXFLAGS += -Wno-double-promotion -Wno-useless-cast

# AprilTag (dependencies of plugin.yaml): a static PIC archive, private
# to this tree, built by the rule below.
APRILTAG_DIR := $(THIRD_PARTIES_DIR)/apriltag
APRILTAG_LIB := $(APRILTAG_DIR)/build/libapriltag.a
INCLUDES += $(APRILTAG_DIR)
THIRD_PARTIES_LIBS += $(APRILTAG_LIB)
PLUGIN_REQUIRES := $(APRILTAG_DIR)/apriltag.h
PLUGIN_DEPENDS := $(APRILTAG_LIB)

include $(P)/plugins/plugin.mk

$(APRILTAG_LIB):
	cmake -S $(APRILTAG_DIR) -B $(APRILTAG_DIR)/build -DCMAKE_BUILD_TYPE=Release \
	    -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF \
	    -DBUILD_PYTHON_WRAPPER=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF
	cmake --build $(APRILTAG_DIR)/build --target apriltag -j$(shell nproc)
