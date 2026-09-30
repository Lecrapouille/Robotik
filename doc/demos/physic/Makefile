# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Location of the project directory and Makefiles
#
P := ../..
M := $(P)/.makefile

###############################################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := Robotik-Physics
TARGET_DESCRIPTION := An URDF robot physics application demo
include $(M)/project/Makefile

###############################################################################
# Inform Makefile where to find header files
#
INCLUDES += $(P)/include $(P)/doc/demos/viewer

###############################################################################
# Inform Makefile where to find *.cpp files
#
VPATH += $(P)/doc/demos/viewer

###################################################
# Set third-party libraries. Order matters: dependencies last
#
THIRD_PARTIES_LIBS += $(call internal-lib,robotik-viewer)
THIRD_PARTIES_LIBS += $(call internal-lib,robotik-core)

###############################################################################
# Make the list of files to compile
#
SRC_FILES := main.cpp
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set operating system libraries.
#
PKG_LIBS += glfw3 glew gl eigen3

###############################################################################
# Sharable information between all Makefiles
#
include $(M)/rules/Makefile
