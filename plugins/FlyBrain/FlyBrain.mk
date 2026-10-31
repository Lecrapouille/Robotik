# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin FlyBrain: a fly brain closed loop, with a circuit of 13
# neurons, or with the whole connectome of Drosophila_brain_model (about
# 100 MB) when FLYBRAIN_CONNECTOME=1. Paths are relative to this directory.

#? 1: fetch the connectome of the fly and extract its edges.
FLYBRAIN_CONNECTOME ?= 0

TARGETS += robotik_fly

robotik_fly.DESCRIPTION := Fly demonstration package
robotik_fly.SOURCES := src/Plugin.cpp src/FlyMission.cpp src/FlyScenario.cpp src/FlyVision.cpp \
    src/FlyController.cpp src/FlyEnvironment.cpp src/FlyBrain.cpp src/ShiuNetwork.cpp
$(eval $(call ROBOTIK_PLUGIN,robotik_fly,FlyBrain))

ifeq ($(FLYBRAIN_CONNECTOME),1)
### Whole adult fly brain model (Shiu et al. 2024). License: MIT
TARGETS += Drosophila_brain_model
Drosophila_brain_model.GIT := https://github.com/philshiu/Drosophila_brain_model.git

# The edges of the connectome, made from its parquet file (with pyarrow).
FLY_DIR := $(HERE)
FLY_DATA := $(abspath $(HERE)data/flywire)
FLY_PY := $(abspath $(BUILD_DIR)/pydeps)
robotik_fly: $(FLY_DATA)/edges.csv
$(FLY_DATA)/edges.csv: | Drosophila_brain_model
	@PYTHONPATH=$(FLY_PY) python3 -c "import pyarrow" 2> /dev/null || \
	    python3 -m pip install --target $(FLY_PY) pyarrow
	PYTHONPATH=$(FLY_PY) python3 $(FLY_DIR)prepare_edges.py $(Drosophila_brain_model.DIR)Connectivity_783.parquet \
	    $(Drosophila_brain_model.DIR)Completeness_783.csv $@
	cp -f $(Drosophila_brain_model.DIR)Completeness_783.csv $(FLY_DATA)/completeness.csv
endif
