# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

PROJECT_NAME := RobotIK
PROJECT_VERSION := 0.4.0
COMPILATION_MODE ?= debug
CXX_STANDARD := c++20

#? The plugins built. PLUGINS="A B" builds only A and B. MicroDuck and
#? HumanoidWalk fetch large models: add them to the list with
#? EXTRA_PLUGINS="MicroDuck HumanoidWalk" (PLUGINS+= on the command line
#? replaces the list, it does not add). FlyBrain uses the whole connectome
#? of the fly (about 100 MB) with FLYBRAIN_CONNECTOME=1.
PLUGINS ?= HelloWorld Painter PickAndPlace DualArm LineFollower FlyBrain
EXTRA_PLUGINS ?=
#? The web page (TOOLCHAIN=emscripten) holds the light plugins only, linked
#? into it: no OpenCV (LineFollower), no ONNX Runtime (MicroDuck,
#? HumanoidWalk), and FlyBrain with its 13-neuron circuit only.
WEB_PLUGINS ?= HelloWorld Painter PickAndPlace DualArm FlyBrain
# (MyMakefile chooses TOOLCHAIN later, with this same test.)
ifneq ($(EMSCRIPTEN)$(EXAEQUOS)$(filter em++%,$(notdir $(CXX)))$(filter emscripten,$(TOOLCHAIN)),)
    override PLUGINS := $(WEB_PLUGINS)
    override EXTRA_PLUGINS :=
    override FLYBRAIN_CONNECTOME := 0
endif
override PLUGINS := $(sort $(PLUGINS) $(EXTRA_PLUGINS))
#? The folders where the sources of the plugins are: a plugin <Name> is the
#? first <dir>/<Name>/<Name>.mk found. Add your own folder with
#? PLUGIN_DIRS="demos doc/tutorials ~/my_plugins".
PLUGIN_DIRS ?= demos doc/tutorials
# The source folder of the plugin $(1), with a trailing slash.
plugin-dir = $(dir $(firstword $(foreach d,$(PLUGIN_DIRS),$(wildcard $d/$(1)/$(1).mk))))
$(foreach p,$(PLUGINS),$(if $(call plugin-dir,$p),,$(error plugin $p is not in PLUGIN_DIRS ($(PLUGIN_DIRS)))))

###############################################################################
# The core library, its applications, the plugins and the tests, each
# declared in the directory of its sources. Compages and BlackThorn are
# compiled from their sources (external/external.mk).
#
SUBMAKEFILES := external/external.mk src/RobotIK/robotik.mk src/Simulator/simulator.mk \
    src/Headless/headless.mk demos/plugin.mk \
    $(foreach p,$(PLUGINS),$(call plugin-dir,$p)$p.mk $(wildcard $(call plugin-dir,$p)app/app.mk)) \
    tests/tests.mk

M ?= .makefile
include $(M)/MyMakefile.mk

#? Install Pinocchio and MuJoCo from conda-forge in external/forge, with
#? micromamba (no sudo, nothing outside external/). Native builds only.
FORGE_DIR := external/forge
MAMBA_ROOT := external/.micromamba
.PHONY: forge
forge:
	$(Q)mkdir -p $(MAMBA_ROOT)
	$(Q)[ -x $(MAMBA_ROOT)/bin/micromamba ] || \
	    curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C $(MAMBA_ROOT) bin/micromamba
	$(Q)$(MAMBA_ROOT)/bin/micromamba create -y -p $(FORGE_DIR) -c conda-forge pinocchio mujoco

#? Check the dependencies between the layers of the sources.
.PHONY: check-layers
check-layers:
	@python3 tests/scripts/check_layers.py .

#? Run the tests with AddressSanitizer (and UndefinedBehaviorSanitizer when
#? libubsan is installed) in build-asan/, then once more in a random order:
#? a test that passes only after another one hides a shared state.
ASAN_SANITIZER := $(if $(wildcard /usr/lib64/libubsan.so* /usr/lib/x86_64-linux-gnu/libubsan.so*),address$(COMMA)undefined,address)
.PHONY: check-asan
check-asan:
	$(Q)$(MAKE) --no-print-directory BUILD_DIR=build-asan SANITIZER=$(ASAN_SANITIZER) all RobotIK-tests
	$(Q)ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
	    build-asan/linux-gcc-debug-$(subst $(COMMA),-,$(ASAN_SANITIZER))/bin/RobotIK-tests
	$(Q)ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
	    build-asan/linux-gcc-debug-$(subst $(COMMA),-,$(ASAN_SANITIZER))/bin/RobotIK-tests --gtest_shuffle

#? Run the tests of the debug build under valgrind memcheck: it sees the
#? values read before they are written (ASan does not).
.PHONY: check-valgrind
check-valgrind: RobotIK-tests
	$(Q)valgrind --error-exitcode=1 --track-origins=yes --leak-check=no \
	    $(BUILD)/bin/RobotIK-tests

#? Check that the relative links of the Markdown files resolve.
.PHONY: check-links
check-links:
	@python3 tests/scripts/check_links.py .
