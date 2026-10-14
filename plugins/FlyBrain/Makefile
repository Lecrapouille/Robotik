# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_fly
TARGET_DESCRIPTION := Fly demonstration package
PLUGIN_NAME := FlyBrain
PLUGIN_DIR := $(P)/plugins/FlyBrain
PLUGIN_SRC := $(PLUGIN_DIR)/src
include $(M)/project/Makefile

# FlyView and FlyDraw belong to the standalone Robotik-Fly window only.
LIB_FILES := $(PLUGIN_SRC)/Plugin.cpp $(PLUGIN_SRC)/FlyMission.cpp
LIB_FILES += $(PLUGIN_SRC)/FlyScenario.cpp $(PLUGIN_SRC)/FlyVision.cpp
LIB_FILES += $(PLUGIN_SRC)/FlyController.cpp $(PLUGIN_SRC)/FlyEnvironment.cpp
LIB_FILES += $(PLUGIN_SRC)/FlyBrain.cpp $(PLUGIN_SRC)/ShiuNetwork.cpp

# FlyWire v783 tables (Shiu et al. 2024, dependencies of plugin.yaml), written
# into the pack as data/flywire/. Optional: without the clone, the plugin is
# built anyway and only the 13-neuron circuit is available. The parquet is
# read with pyarrow, installed in external/.pydeps when the system lacks it.
FLY_SRC := $(THIRD_PARTIES_DIR)/Drosophila_brain_model
FLY_DATA := $(abspath $(PLUGIN_DIR)/data/flywire)
FLY_PY := $(THIRD_PARTIES_DIR)/.pydeps
ifneq ($(wildcard $(FLY_SRC)/Connectivity_783.parquet),)
PLUGIN_DEPENDS := $(FLY_DATA)/edges.csv
endif

include $(P)/plugins/plugin.mk

$(FLY_DATA)/edges.csv: $(FLY_SRC)/Connectivity_783.parquet $(FLY_SRC)/Completeness_783.csv
	@PYTHONPATH=$(FLY_PY) python3 -c "import pyarrow" 2> /dev/null || \
	    python3 -m pip install --target $(FLY_PY) pyarrow
	PYTHONPATH=$(FLY_PY) python3 $(PLUGIN_DIR)/prepare_edges.py $^ $@
	cp -f $(FLY_SRC)/Completeness_783.csv $(FLY_DATA)/completeness.csv
