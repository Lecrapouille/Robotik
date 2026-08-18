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
INCLUDES += $(P)/src $(THIRD_PARTIES_DIR)

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
HEADER_FILES := $(call rwildcard,$(ROBOTIK_CORE_PATH),*.hpp)

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
# Set units library
#
INCLUDES += $(THIRD_PARTIES_DIR)/units/include

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