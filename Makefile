# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Location of the project directory and Makefiles
#
P := .
M := $(P)/.makefile

###############################################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := $(PROJECT_NAME)
RUNNING_TARGET_NAME := Robotik-Simulator
TARGET_DESCRIPTION := A robot library
ORCHESTRATOR_MODE := 1

include $(M)/project/Makefile

###################################################
# Internal libs to compile in the correct order
#
LIB_ROBOTIK_CORE := $(call internal-lib,robotik-core)
INTERNAL_LIBS := $(LIB_ROBOTIK_CORE)
DIRS_WITH_MAKEFILE := $(P)/src/Robotik

###################################################
# Generic Makefile rules
#
include $(M)/rules/Makefile

###################################################
# Post-download setup. external/manifest is cloned by MyMakefile; then:
# - units gets the header name the code includes;
# - Compages downloads its own libraries (units, entt...): its headers need
#   them before anything is compiled;
# - Pinocchio and MuJoCo come from conda-forge in a user-local prefix
#   (external/forge): they need Boost and urdfdom, which are not assumed to
#   be installed. No sudo, nothing outside external/;
# - every plugin with dependencies.thirdparties in its plugin.yaml downloads
#   its own libraries (into plugins/<Name>/external).
# Then plain make builds everything: each library has a Make rule.
#
FORGE_DIR := $(THIRD_PARTIES_DIR)/forge
MAMBA_ROOT := $(THIRD_PARTIES_DIR)/.micromamba
PLUGINS_WITH_LIBS := $(dir $(shell grep -l '^  thirdparties:' $(P)/plugins/*/plugin.yaml))

TPNE_EXTERNAL := $(THIRD_PARTIES_DIR)/TimedPetriNetEditor/external

download-external-libs::
	@cp $(THIRD_PARTIES_DIR)/units/include/units.h $(THIRD_PARTIES_DIR)/units/units.hpp
	@# TimedPetriNetEditor: only the two libraries of its core, not its editor.
	@[ -d $(TPNE_EXTERNAL)/json ] || git clone --depth 1 https://github.com/nlohmann/json.git $(TPNE_EXTERNAL)/json
	@[ -d $(TPNE_EXTERNAL)/tinyxml2 ] || git clone --depth 1 https://github.com/leethomason/tinyxml2.git $(TPNE_EXTERNAL)/tinyxml2
	@$(MAKE) --no-print-directory -C $(THIRD_PARTIES_DIR)/Compages download-external-libs
	@if [ -z "$$(find $(FORGE_DIR) -path '*/pkgconfig/pinocchio.pc' -print -quit 2>/dev/null)" ] || \
	    [ -z "$$(find $(FORGE_DIR) -name 'libmujoco.so*' -print -quit 2>/dev/null)" ]; then \
	    if [ -n "$$EMSCRIPTEN" ] || [ -n "$$EXAEQUOS" ]; then \
	        echo "Skipping Pinocchio and MuJoCo: they are native libraries."; \
	    else \
	    mkdir -p $(MAMBA_ROOT) && \
	    ( [ -x $(MAMBA_ROOT)/bin/micromamba ] || \
	      curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xj -C $(MAMBA_ROOT) bin/micromamba ) && \
	    $(MAMBA_ROOT)/bin/micromamba create -y -p $(FORGE_DIR) -c conda-forge pinocchio mujoco; \
	    fi; \
	fi
	@for d in $(PLUGINS_WITH_LIBS); do \
	    $(MAKE) --no-print-directory -C $$d download-external-libs || exit 1; \
	done

###################################################
# Extra rules: compile applications after everything
#
APPLICATIONS = $(sort $(dir $(wildcard $(P)/simulator/Makefile $(P)/headless/Makefile $(P)/plugins/*/Makefile $(P)/plugins/*/app/Makefile)))

.PHONY: applications
applications: $(DIRS_WITH_MAKEFILE)
	@$(call print-from,"Compiling applications",$(PROJECT_NAME),$(APPLICATIONS))
	@for i in $(APPLICATIONS);     \
	do                             \
		$(MAKE) -C $$i all || exit 1; \
	done;

post-build:: applications

###################################################
# A layer only includes the layers below it (Common < Robot < Task < Package),
# and skills and the planner never read the simulator truth.
#
.PHONY: check-layers
check-layers:
	@python3 $(P)/tests/scripts/check_layers.py $(P)
