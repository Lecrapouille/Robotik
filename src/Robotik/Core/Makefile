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
# Set Eigen3 library
#
PKG_LIBS += eigen3
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set pugixml library
#
LIB_FILES += $(THIRD_PARTIES_DIR)/pugixml/src/pugixml.cpp
INCLUDES += $(THIRD_PARTIES_DIR)/pugixml/src
USER_CXXFLAGS += -Wno-switch-enum

###############################################################################
# Set units library
#
INCLUDES += $(THIRD_PARTIES_DIR)/units/include

###############################################################################
# Set Tracy frame profiler library
#
DEFINES += -DTRACER_ENABLED
INCLUDES += $(THIRD_PARTIES_DIR)/tracy/public

###############################################################################
# Sharable information between all Makefiles
#
include $(M)/rules/Makefile