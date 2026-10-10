# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Shared recipe for a demo plugin.
# The including Makefile sets TARGET_NAME, PLUGIN_NAME, PLUGIN_DIR and
# LIB_FILES, then includes this file. The staged package is
# build/plugins/$(PLUGIN_NAME)/ with plugin.yaml, the .so and scenarios/.

INCLUDES += $(PLUGIN_DIR) $(P)/include $(P)/src
VPATH += $(PLUGIN_DIR)
DIRS_WITH_MAKEFILE := $(P)/src/Robotik/Core
DO_NOT_COMPILE_STATIC_LIB := 1
USER_LDFLAGS += -L$(BUILD_PATH) -Wl,-rpath,$(BUILD_PATH) -Wl,-rpath,'$$ORIGIN/../..'
USER_LDFLAGS += -lrobotik-core -ldl

BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)

include $(M)/rules/Makefile

PLUGIN_STAGE := $(BUILD_PATH)/plugins/$(PLUGIN_NAME)

post-build:: $(TARGET_SHARED_LIB_NAME)
	@$(MKDIR) $(PLUGIN_STAGE)/scenarios
	$(Q)cp $(PLUGIN_DIR)/plugin.yaml $(PLUGIN_STAGE)/plugin.yaml
	$(Q)cp $(PLUGIN_DIR)/scenarios/*.yml $(PLUGIN_STAGE)/scenarios/
	$(Q)cp $(TARGET_SHARED_LIB_NAME) $(PLUGIN_STAGE)/
