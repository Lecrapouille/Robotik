# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Shared recipe for a plugin package.
# The including Makefile sets TARGET_NAME, PLUGIN_NAME, PLUGIN_DIR,
# PLUGIN_SRC and LIB_FILES, then includes this file. The staged package is
# build/plugins/$(PLUGIN_NAME)/: plugin.yaml, the .so, and the content
# folders the package has (worlds/, scenarios/, data/ with its asset pack).

INCLUDES += $(PLUGIN_SRC) $(P)/include $(P)/src
VPATH += $(PLUGIN_SRC)
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
include $(P)/Makefile.relink

PLUGIN_STAGE := $(BUILD_PATH)/plugins/$(PLUGIN_NAME)
PLUGIN_CONTENT := $(notdir $(wildcard $(addprefix $(PLUGIN_DIR)/,worlds scenarios data)))

# Content folders are copied whole: a file removed from the sources must not
# survive in the stage.
post-build:: $(TARGET_SHARED_LIB_NAME)
	@$(MKDIR) $(PLUGIN_STAGE)
	$(Q)cp $(PLUGIN_DIR)/plugin.yaml $(PLUGIN_STAGE)/plugin.yaml
	$(Q)for d in $(PLUGIN_CONTENT); do rm -rf $(PLUGIN_STAGE)/$$d && cp -R $(PLUGIN_DIR)/$$d $(PLUGIN_STAGE)/$$d; done
	$(Q)cp $(TARGET_SHARED_LIB_NAME) $(PLUGIN_STAGE)/
