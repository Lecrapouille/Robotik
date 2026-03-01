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
TARGET_NAME := Robotik-Simulator
TARGET_DESCRIPTION := Robot simulation and control application
include $(M)/project/Makefile

CURRENT_DIR := $(P)/src/Applications/Simulator

###############################################################################
# Inform Makefile where to find header files
#
IMGUI_DIR := $(THIRD_PARTIES_DIR)/imgui
IMGUIFILEDIALOG_DIR := $(THIRD_PARTIES_DIR)/ImGuiFileDialog
INCLUDES += $(CURRENT_DIR) $(THIRD_PARTIES_DIR) $(P)/src $(IMGUI_DIR) $(IMGUI_DIR)/misc/cpp $(IMGUI_DIR)/backends
INCLUDES += $(IMGUIFILEDIALOG_DIR) $(IMGUIFILEDIALOG_DIR)/stb

###############################################################################
# Inform Makefile where to find *.cpp files
#
VPATH += $(P)/src $(CURRENT_DIR)

###################################################
# Set third-party libraries. Order matters: dependencies last
#
INTERNAL_LIBS += $(call internal-lib,robotik-renderer)
INTERNAL_LIBS += $(call internal-lib,robotik-core)

###############################################################################
# Make the list of files to compile
#
SRC_FILES := $(call rwildcard,$(CURRENT_DIR),*.cpp)
SRC_FILES += $(IMGUIFILEDIALOG_DIR)/ImGuiFileDialog.cpp
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set operating system libraries.
#
PKG_LIBS += glfw3 glew gl eigen3 yaml-cpp sfml-network

###############################################################################
# Sharable information between all Makefiles
#
include $(M)/rules/Makefile