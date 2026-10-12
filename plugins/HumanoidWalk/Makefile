# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

# ONNX Runtime and the asset pack come from make compile-external-libs
# (external/onnxruntime, data/hu_d04). Without them the plugin is skipped.
ONNXRUNTIME_DIR := $(abspath $(P)/external/onnxruntime)
ifeq ($(and $(wildcard $(ONNXRUNTIME_DIR)/include/onnxruntime_cxx_api.h),$(wildcard $(P)/plugins/HumanoidWalk/data/hu_d04/walk.onnx)),)
.PHONY: all clean
all:
	@echo "HumanoidWalk skipped: run make download-external-libs && make compile-external-libs"
clean:
else

include $(P)/Makefile.common
TARGET_NAME := robotik_humanoid_walk
TARGET_DESCRIPTION := Humanoid walking by a learned policy
PLUGIN_NAME := HumanoidWalk
PLUGIN_DIR := $(P)/plugins/HumanoidWalk
PLUGIN_SRC := $(PLUGIN_DIR)/src
include $(M)/project/Makefile

LIB_FILES := $(wildcard $(PLUGIN_SRC)/*.cpp)

USER_CXXFLAGS += -isystem $(ONNXRUNTIME_DIR)/include
USER_LDFLAGS += -L$(ONNXRUNTIME_DIR)/lib -Wl,-rpath,$(ONNXRUNTIME_DIR)/lib -lonnxruntime

include $(P)/plugins/plugin.mk

endif
