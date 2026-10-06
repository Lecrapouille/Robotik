# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Location of the project directory and Makefiles
#
P := ../../..
M := $(P)/.makefile

###############################################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := robotik-core
TARGET_DESCRIPTION := Core library for Robotik
include $(M)/project/Makefile

###############################################################################
# Inform Makefile where to find header files
#
INCLUDES += $(P)/src $(P)/include $(THIRD_PARTIES_DIR)

###############################################################################
# Inform Makefile where to find *.cpp files
#
VPATH += $(P)/src

###############################################################################
# Project defines
#
DEFINES +=

###############################################################################
# Make the list of files to compile
#
ROBOTIK_CORE_PATH := $(P)/src/Robotik/Core
LIB_FILES += $(call rwildcard,$(ROBOTIK_CORE_PATH),*.cpp)
ROBOTIK_MODULES := Backends Environment Perception Robot
ROBOTIK_MODULES += Runtime Scenario Sensors Skills Systems
LIB_FILES += $(foreach m,$(ROBOTIK_MODULES),$(call rwildcard,$(P)/src/Robotik/$(m),*.cpp))
HEADER_FILES := $(call rwildcard,$(ROBOTIK_CORE_PATH),*.hpp)
HEADER_FILES += $(call rwildcard,$(P)/include/Robotik,*.hpp)

###############################################################################
# Set pugixml library (from thirdparties)
#
LIB_FILES += $(THIRD_PARTIES_DIR)/pugixml/src/pugixml.cpp
INCLUDES += $(THIRD_PARTIES_DIR)/pugixml/src
USER_CXXFLAGS += -Wno-switch-enum

###############################################################################
# Eigen3 is set by Makefile.common. These relaxations remain needed by the
# templates it expands inside our own code.
#
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set BlackThorn behavior tree engine (from thirdparties). Its sources are
# compiled straight into this library, as done for pugixml. rapidyaml, its YAML
# backend, is cloned and built at the Robotik level: point BlackThorn at it.
#
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk

INCLUDES += $(BLACKTHORN_INCLUDES)
VPATH += $(BLACKTHORN_VPATH)
LIB_FILES += $(BLACKTHORN_SOURCES)
DEFINES += $(BLACKTHORN_DEFINES)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
THIRD_PARTIES_LIBS += $(BLACKTHORN_STATIC_DEPS)

###############################################################################
# Set SWI-Prolog library
#
PKG_LIBS += swipl

###############################################################################
# Sharable information between all Makefiles
#
include $(M)/rules/Makefile

###############################################################################
# Build rapidyaml on demand. MyMakefile ignores the archives produced by
# external/compilation, so a tree where they are missing fails late with an
# opaque "ar: libryml.a: No such file" instead of building them.
#
$(BLACKTHORN_STATIC_DEPS):
	@$(MAKE) --no-print-directory -C $(P) compile-external-libs

$(TARGET_STATIC_LIB_NAME) $(TARGET_SHARED_LIB_NAME): $(BLACKTHORN_STATIC_DEPS)
