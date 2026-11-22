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
    && cp -L $$(BUILD)/lib/lib$(1).so $$(ROBOTIK_PLUGINS_DIR)/$(2)/
# plugin.yaml and the folders of the package are copied again when one of
# their files changes, even when the library is not linked again: a change
# of a world or a scenario needs no C++ build. The folders are in the list
# too, so a removed file changes their date.
$(1)._PACKAGE_DIRS := $(wildcard $(addprefix $(HERE),worlds scenarios experiments data))
$(1)._PACKAGE_STAMP := $$(ROBOTIK_PLUGINS_DIR)/$(2)/.package
$$($(1)._PACKAGE_STAMP): $(HERE)plugin.yaml $$($(1)._PACKAGE_DIRS) \
        $$(foreach d,$$($(1)._PACKAGE_DIRS),$$(call rwildcard,$$d,*))
	@mkdir -p $$(ROBOTIK_PLUGINS_DIR)/$(2)
	@cp $(HERE)plugin.yaml $$(ROBOTIK_PLUGINS_DIR)/$(2)/
	@for d in $$(notdir $$($(1)._PACKAGE_DIRS)); do \
	    rm -rf $$(ROBOTIK_PLUGINS_DIR)/$(2)/$$$$d && cp -R $(HERE)$$$$d $$(ROBOTIK_PLUGINS_DIR)/$(2)/$$$$d; done
	@rm -rf $$(ROBOTIK_PLUGINS_DIR)/$(2)/external
	@touch $$@
$(1): $$($(1)._PACKAGE_STAMP)
endef

# ONNX Runtime, the inference of the walking policies (MicroDuck,
# HumanoidWalk). License: MIT. GitHub no longer serves its releases, so it
# comes from its NuGet package: a zip with the headers and the library of
# each platform, checked by its SHA256 and unpacked once in the cache of
# MyMakefile. MyMakefile unpacks only names ending in .tar* or .zip, so this
# file does it and gives the folder to MyMakefile as a local copy (PATH).
ORT_VERSION := 1.20.1
ORT_SHA256 := 9c32108e9a1c78b090a95c2a219f244dd5f067d4caf68fb0f967103f3f20839c
ORT_URL := https://api.nuget.org/v3-flatcontainer/microsoft.ml.onnxruntime/$(ORT_VERSION)/microsoft.ml.onnxruntime.$(ORT_VERSION).nupkg
ORT_ARCH := $(if $(filter aarch64 arm64,$(HOST_ARCH)),arm64,x64)
ORT_CACHE := $(or $(EXTERNAL_CACHE),$(MYMAKEFILE_CACHE),$(or $(XDG_CACHE_HOME),$(HOME)/.cache)/mymakefile)
ORT_ZIP := $(ORT_CACHE)/downloads/$(notdir $(ORT_URL))
ORT_HOME := $(ORT_CACHE)/sources/onnxruntime-nuget/$(ORT_VERSION)-linux-$(ORT_ARCH)
ORT_LIBDIR := runtimes/linux-$(ORT_ARCH)/native

# Downloads (unless done), checks and unpacks the package. The library has
# the soname libonnxruntime.so.1: a link gives that name to the loader.
ORT_FETCH := set -e; mkdir -p $(dir $(ORT_ZIP)) $(ORT_HOME); \
    test -f $(ORT_ZIP) || curl -fsSL -o $(ORT_ZIP) $(ORT_URL); \
    echo "$(ORT_SHA256)  $(ORT_ZIP)" | sha256sum -c --quiet -; \
    unzip -q -o $(ORT_ZIP) 'build/native/include/*' '$(ORT_LIBDIR)/*' -d $(ORT_HOME); \
    ln -sf libonnxruntime.so $(ORT_HOME)/$(ORT_LIBDIR)/libonnxruntime.so.1; \
    echo $(ORT_URL) > $(ORT_HOME)/.robotik-ok

define ROBOTIK_ONNXRUNTIME
ifeq ($$(wildcard $$(ORT_HOME)/.robotik-ok),)
    $$(info *** Downloading: onnxruntime $$(ORT_VERSION) => $$(ORT_HOME))
    ORT_LOG := $$(shell ( $$(ORT_FETCH) ) 2>&1)
    ifeq ($$(wildcard $$(ORT_HOME)/.robotik-ok),)
        $$(error cannot fetch onnxruntime $$(ORT_URL): $$(ORT_LOG))
    endif
endif
TARGETS += onnxruntime
onnxruntime.PATH := $$(ORT_HOME)
onnxruntime.PUBLIC_INCLUDES := build/native/include
onnxruntime.LIBRARIES := $$(ORT_LIBDIR)/libonnxruntime.so
endef
