# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_probe
TARGET_DESCRIPTION := Lifecycle probe used by the plugin tests
PLUGIN_NAME := Probe
PLUGIN_DIR := $(P)/demos/Probe
include $(M)/project/Makefile

LIB_FILES := $(PLUGIN_DIR)/Plugin.cpp

include $(PLUGIN_DIR)/../plugin.mk
