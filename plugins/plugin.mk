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
# folders the package has (worlds/, scenarios/, experiments/, data/ with
# its asset pack).
#
# Libraries of the plugin are listed in dependencies: of its plugin.yaml,
# with their license (see dependencies.awk):
#   system:        pkg-config packages; when one is missing the plugin is
#                  skipped;
#   thirdparties:  { repo: user/repo, license: ... } entries are written as
#                  a MyMakefile manifest in build/manifests/<Name> (generated)
#                  and cloned into $(PLUGIN_DIR)/external by make
#                  download-external-libs. THIRD_PARTIES_DIR is that folder
#                  (see Makefile.common).
# The including Makefile may also set:
#   PLUGIN_REQUIRES  files that must exist, or the plugin is skipped (not
#                    failed) with a hint to download its libraries;
#   PLUGIN_DEPENDS   files its own rules make (an archive, generated data):
#                    they are made before the library is linked and staged.
# Its rules come after the include of this file.

PLUGIN_AWK := $(abspath $(P)/plugins/dependencies.awk)
PLUGIN_YAML := $(abspath $(PLUGIN_DIR)/plugin.yaml)
# Generated: kept in build/, not beside the sources.
PROJECT_MANIFEST_THIRD_PARTIES := $(BUILD_PATH)/manifests/$(PLUGIN_NAME)
$(shell manifest="$$(awk -v section=thirdparties -f $(PLUGIN_AWK) $(PLUGIN_YAML))"; \
    if [ -n "$$manifest" ]; then mkdir -p $(THIRD_PARTIES_DIR) $(BUILD_PATH)/manifests && \
    printf '%s\n' "$$manifest" > $(PROJECT_MANIFEST_THIRD_PARTIES); fi)
PLUGIN_SYSTEM := $(shell awk -v section=system -f $(PLUGIN_AWK) $(PLUGIN_YAML))
PLUGIN_SYSTEM_MISSING := $(strip $(foreach p,$(PLUGIN_SYSTEM),$(if $(shell pkg-config --exists $(p) && echo y),,$(p))))

PLUGIN_MISSING := $(strip $(foreach f,$(PLUGIN_REQUIRES),$(if $(wildcard $(f)),,$(f))))
ifneq ($(PLUGIN_MISSING)$(PLUGIN_SYSTEM_MISSING),)

.DEFAULT_GOAL := all
.PHONY: all clean download-external-libs
all:
ifneq ($(PLUGIN_SYSTEM_MISSING),)
	@echo "$(PLUGIN_NAME) skipped: system library $(PLUGIN_SYSTEM_MISSING) not found by pkg-config."
endif
ifneq ($(PLUGIN_MISSING),)
	@echo "$(PLUGIN_NAME) skipped: missing $(notdir $(PLUGIN_MISSING))."
	@echo "  Run: make -C plugins/$(PLUGIN_NAME) download-external-libs, then make"
endif
clean:
download-external-libs::
ifneq ($(wildcard $(PROJECT_MANIFEST_THIRD_PARTIES)),)
	$(Q)cd $(THIRD_PARTIES_DIR) && bash $(SCRIPT_DOWNLOAD_THIRD_PARTIES) $(PROJECT_NAME) $(PROJECT_MANIFEST_THIRD_PARTIES)
endif

else

INCLUDES += $(PLUGIN_SRC) $(P)/include $(P)/src
VPATH += $(PLUGIN_SRC)
DIRS_WITH_MAKEFILE := $(P)/src/Robotik
DO_NOT_COMPILE_STATIC_LIB := 1
USER_LDFLAGS += -L$(BUILD_PATH) -Wl,-rpath,$(BUILD_PATH) -Wl,-rpath,'$$ORIGIN/../..'
USER_LDFLAGS += -lrobotik-core -ldl

BLACKTHORN_DIR := $(ROBOTIK_EXTERNAL)/BlackThorn
RYML_DIR := $(ROBOTIK_EXTERNAL)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)

include $(M)/rules/Makefile
include $(P)/Makefile.relink

PLUGIN_STAGE := $(BUILD_PATH)/plugins/$(PLUGIN_NAME)
PLUGIN_CONTENT := $(notdir $(wildcard $(addprefix $(PLUGIN_DIR)/,worlds scenarios experiments data)))

$(TARGET_SHARED_LIB_NAME): $(PLUGIN_DEPENDS)

# Content folders are mirrored: a file removed from the sources must not
# survive in the stage. rsync copies only what changed (data/ may hold
# hundreds of MB); without it the folder is copied whole. external/ is not
# staged: the libraries and their licenses are listed in plugin.yaml.
post-build:: $(TARGET_SHARED_LIB_NAME)
	@$(MKDIR) $(PLUGIN_STAGE)
	$(Q)cp $(PLUGIN_DIR)/plugin.yaml $(PLUGIN_STAGE)/plugin.yaml
	$(Q)for d in $(PLUGIN_CONTENT); do \
	    if command -v rsync > /dev/null; then rsync -a --delete $(PLUGIN_DIR)/$$d/ $(PLUGIN_STAGE)/$$d/; \
	    else rm -rf $(PLUGIN_STAGE)/$$d && cp -R $(PLUGIN_DIR)/$$d $(PLUGIN_STAGE)/$$d; fi; \
	done
	$(Q)cp $(TARGET_SHARED_LIB_NAME) $(PLUGIN_STAGE)/
	$(Q)rm -rf $(PLUGIN_STAGE)/external

endif
