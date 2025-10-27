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
P := ../..
M := $(P)/.makefile

###############################################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := Robotik-Viewer
TARGET_DESCRIPTION := An URDF robot viewer application demo
include $(M)/project/Makefile

###############################################################################
# Inform Makefile where to find header files
#
IMGUI_DIR := $(THIRD_PARTIES_DIR)/imgui
INCLUDES += $(P)/include $(P)/applications/viewer
INCLUDES += $(IMGUI_DIR) $(IMGUI_DIR)/misc/cpp $(IMGUI_DIR)/backends

###############################################################################
# Inform Makefile where to find *.cpp files
#
VPATH += $(P)/doc/applications/viewer

###################################################
# Set third-party libraries. Order matters: dependencies last
#
THIRD_PARTIES_LIBS += $(call internal-lib,robotik-renderer)
THIRD_PARTIES_LIBS += $(call internal-lib,robotik-core)

###############################################################################
# Make the list of files to compile
#
SRC_FILES := $(wildcard $(P)/applications/viewer/*.cpp)
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