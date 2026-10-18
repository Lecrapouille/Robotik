# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_microduck
TARGET_DESCRIPTION := Microduck walking by a learned policy
PLUGIN_NAME := MicroDuck
PLUGIN_DIR := $(P)/plugins/MicroDuck
PLUGIN_SRC := $(PLUGIN_DIR)/src
include $(M)/project/Makefile

LIB_FILES := $(wildcard $(PLUGIN_SRC)/*.cpp)

# microduck_rl (dependencies of plugin.yaml) and the ONNX Runtime release
# (downloaded below) are private to this plugin. Without them the plugin is
# skipped. The asset pack data/microduck is made from the clone.
ORT_VERSION := 1.20.1
ONNXRUNTIME_DIR := $(THIRD_PARTIES_DIR)/onnxruntime
MICRODUCK_RL := $(THIRD_PARTIES_DIR)/microduck_rl
DUCK := $(abspath $(PLUGIN_DIR)/data/microduck)
PLUGIN_REQUIRES := $(ONNXRUNTIME_DIR)/include/onnxruntime_cxx_api.h
PLUGIN_REQUIRES += $(MICRODUCK_RL)
PLUGIN_DEPENDS := $(DUCK)/walk.onnx

USER_CXXFLAGS += -isystem $(ONNXRUNTIME_DIR)/include
USER_LDFLAGS += -L$(ONNXRUNTIME_DIR)/lib -Wl,-rpath,$(ONNXRUNTIME_DIR)/lib -lonnxruntime

include $(P)/plugins/plugin.mk

$(DUCK)/walk.onnx: $(PLUGIN_DIR)/prepare_assets.py $(MICRODUCK_RL)/src/mjlab_microduck/robot/microduck/robot_groundcontact.xml
	python3 -I $(PLUGIN_DIR)/prepare_assets.py $(MICRODUCK_RL) $(PLUGIN_DIR)

# Prebuilt release of Microsoft: nothing to compile.
download-external-libs::
	@if [ ! -f $(ONNXRUNTIME_DIR)/include/onnxruntime_cxx_api.h ]; then \
	    case "$$(uname -m)" in \
	        x86_64) arch=x64 ;; aarch64) arch=aarch64 ;; \
	        *) echo "No ONNX Runtime release for $$(uname -m)"; exit 1 ;; \
	    esac; \
	    name=onnxruntime-linux-$$arch-$(ORT_VERSION); \
	    cd $(THIRD_PARTIES_DIR) && \
	    curl -Ls https://github.com/microsoft/onnxruntime/releases/download/v$(ORT_VERSION)/$$name.tgz | tar -xz && \
	    rm -rf onnxruntime && mv $$name onnxruntime; \
	fi
