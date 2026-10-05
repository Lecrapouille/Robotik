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
DIRS_WITH_MAKEFILE := $(P)/src/Robotik/Core

###################################################
# Generic Makefile rules
#
include $(M)/rules/Makefile

###################################################
# Post-download setup
#
download-external-libs::
	@cp $(THIRD_PARTIES_DIR)/units/include/units.h $(THIRD_PARTIES_DIR)/units/units.hpp

###################################################
# Extra rules: compile applications after everything
#
APPLICATIONS = $(sort $(dir $(wildcard $(P)/src/Applications/*/Makefile $(P)/src/Applications/Demos/*/Makefile)))

.PHONY: applications
applications: $(DIRS_WITH_MAKEFILE)
	@$(call print-from,"Compiling applications",$(PROJECT_NAME),$(APPLICATIONS))
	@for i in $(APPLICATIONS);     \
	do                             \
		$(MAKE) -C $$i all;        \
	done;

post-build:: applications
