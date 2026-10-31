# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# What the plugins share. A plugin is a shared library loaded by the
# simulator and the headless runtime, found with its plugin.yaml and its
# content in BUILD/lib/robotik/plugins/<Name>/, where the executables of
# BUILD/bin look for them (as once installed).
#
#   TARGETS += robotik_hello_world
#   robotik_hello_world.SOURCES := src/*.cpp
#   $(eval $(call ROBOTIK_PLUGIN,robotik_hello_world,HelloWorld))
#
# A plugin needing system packages not found by pkg-config is not built:
#   ifneq ($(call robotik-missing,opencv4),)

ROBOTIK_PLUGINS_DIR = $(BUILD)/lib/robotik/plugins

# The pkg-config packages not installed.
robotik-missing = $(strip $(foreach p,$1,$(if $(shell $(PKG_CONFIG) --exists $p && echo y),,$p)))

# $(1): target, $(2): name of the plugin. Read in the file of the plugin.
define ROBOTIK_PLUGIN
$(1).TYPE := shared
$(1).INCLUDES += src
$(1).DEPENDS += robotik-core
$(1).INSTALL := no
$(1).CLEAN := $$(ROBOTIK_PLUGINS_DIR)/$(2)
$(1).POST_BUILD := mkdir -p $$(ROBOTIK_PLUGINS_DIR)/$(2) \
    && cp $(HERE)plugin.yaml $$(ROBOTIK_PLUGINS_DIR)/$(2)/ \
    && for d in $(notdir $(wildcard $(addprefix $(HERE),worlds scenarios experiments data))); do \
        rm -rf $$(ROBOTIK_PLUGINS_DIR)/$(2)/$$$$$$$$d && cp -R $(HERE)$$$$$$$$d $$(ROBOTIK_PLUGINS_DIR)/$(2)/$$$$$$$$d; done \
    && rm -rf $$(ROBOTIK_PLUGINS_DIR)/$(2)/external \
    && cp -L $$(BUILD)/lib/lib$(1).so $$(ROBOTIK_PLUGINS_DIR)/$(2)/
endef

# ONNX Runtime, the inference of the walking policies (MicroDuck,
# HumanoidWalk): a release of binaries, once in the cache. License: MIT
ORT_VERSION := 1.20.1
ORT_ARCH := $(if $(filter aarch64 arm64,$(HOST_ARCH)),aarch64,x64)
define ROBOTIK_ONNXRUNTIME
TARGETS += onnxruntime
onnxruntime.URL := https://github.com/microsoft/onnxruntime/releases/download/v$(ORT_VERSION)/onnxruntime-linux-$(ORT_ARCH)-$(ORT_VERSION).tgz
onnxruntime.SUBDIR := onnxruntime-linux-$(ORT_ARCH)-$(ORT_VERSION)
onnxruntime.PUBLIC_INCLUDES := include
onnxruntime.LIBRARIES := lib/libonnxruntime.so
endef
