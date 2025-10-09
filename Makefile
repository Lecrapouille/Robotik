###############################################################################
## Robotik: A basic logger.
## Copyright 2025 Quentin Quadrat <lecrapouille@gmail.com>
##
## This file is part of Robotik.
##
## Robotik is free software: you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Robotik is distributed in the hope that it will be useful, but
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
## General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Robotik.  If not, see <http://www.gnu.org/licenses/>.
###############################################################################

###############################################################################
# Location of the project directory and Makefiles
#
P := .
M := $(P)/.makefile

###############################################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := $(PROJECT_NAME)
TARGET_DESCRIPTION := A robot library
include $(M)/project/Makefile
ORCHESTRATOR_MODE := 1

###################################################
# Internal libs to compile
#
LIB_ROBOTIK_CORE := $(call internal-lib,robotik-core)
LIB_ROBOTIK_VIEWER := $(call internal-lib,robotik-viewer)
INTERNAL_LIBS := $(LIB_ROBOTIK_CORE) $(LIB_ROBOTIK_VIEWER)
DIRS_WITH_MAKEFILE := $(P)/src/Robotik $(P)/src/Viewer

###################################################
# Demos to compile after libs
#
DEMO_DIRS := $(sort $(dir $(wildcard $(P)/doc/demos/*/.)))

###################################################
# Compile internal libraries in the correct order:
# robotik-viewer depends on robotik-core
#
$(LIB_ROBOTIK_VIEWER): $(P)/src/Viewer

$(LIB_ROBOTIK_CORE): $(P)/src/Robotik

$(LIB_ROBOTIK_VIEWER):| $(LIB_ROBOTIK_CORE)

###################################################
# Generic Makefile rules
#
include $(M)/rules/Makefile

###################################################
# Extra rules: compile demos after everything
#
DEMOS = $(sort $(dir $(wildcard ./applications/*/.)))

.PHONY: applications
applications: | $(INTERNAL_LIBS)
	@$(call print-from,"Compiling applications",$(PROJECT_NAME),$(DEMOS))
	@for i in $(DEMOS);        \
	do                             \
		$(MAKE) -C $$i all;        \
	done;

###################################################
# Clean targets
#
all:: applications