# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

PROJECT_NAME := Robotik
PROJECT_VERSION := 0.4.0
COMPILATION_MODE ?= debug
CXX_STANDARD := c++20

#? The plugins built. MicroDuck and HumanoidWalk fetch large models: add them
#? with PLUGINS+="MicroDuck HumanoidWalk". FlyBrain uses the whole connectome
#? of the fly (about 100 MB) with FLYBRAIN_CONNECTOME=1.
PLUGINS ?= HelloWorld Painter PickAndPlace DualArm LineFollower FlyBrain

###############################################################################
# The core library, its applications, the plugins and the tests, each
# declared in the directory of its sources. Compages, BlackThorn and
# TimedPetriNetEditor are compiled from their sources (external/external.mk).
#
SUBMAKEFILES := external/external.mk src/Robotik/robotik.mk simulator/simulator.mk \
    headless/headless.mk plugins/plugin.mk \
    $(foreach p,$(PLUGINS),plugins/$p/$p.mk $(wildcard plugins/$p/app/app.mk)) \
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
