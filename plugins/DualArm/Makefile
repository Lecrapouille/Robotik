# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := robotik_dual_arm
TARGET_DESCRIPTION := Two arms planned in parallel
PLUGIN_NAME := DualArm
PLUGIN_DIR := $(P)/plugins/DualArm
PLUGIN_SRC := $(PLUGIN_DIR)/src
include $(M)/project/Makefile

LIB_FILES := $(wildcard $(PLUGIN_SRC)/*.cpp)

include $(P)/plugins/plugin.mk
