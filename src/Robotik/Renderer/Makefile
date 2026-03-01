###############################################################################
## Robotik: A robot library.
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
TARGET_NAME := robotik-renderer
TARGET_DESCRIPTION := Renderer library for Robotik
include $(M)/project/Makefile

###############################################################################
# Inform Makefile where to find header files
#
INCLUDES += $(P)/src

###############################################################################
# Inform Makefile where to find *.cpp files
#
VPATH += $(P)/src

###############################################################################
# Make the list of files to compile
#
RENDERER_PATH := $(P)/src/Robotik/Renderer
LIB_FILES := $(call rwildcard,$(RENDERER_PATH),*.cpp)
HEADER_FILES := $(call rwildcard,$(RENDERER_PATH),*.hpp)

###############################################################################
# Set Eigen3 library
#
PKG_LIBS += eigen3
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set OpenGL library
#
PKG_LIBS += glfw3 glew gl

###################################################
# Set third party Dear ImGui
#
IMGUI_DIR := $(THIRD_PARTIES_DIR)/imgui
INCLUDES += $(IMGUI_DIR) $(IMGUI_DIR)/misc/cpp $(IMGUI_DIR)/backends
VPATH += $(IMGUI_DIR) $(IMGUI_DIR)/misc/cpp $(IMGUI_DIR)/backends
LIB_FILES += $(IMGUI_DIR)/imgui_widgets.cpp $(IMGUI_DIR)/imgui_draw.cpp
LIB_FILES += $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui.cpp
LIB_FILES += $(IMGUI_DIR)/misc/cpp/imgui_stdlib.cpp
LIB_FILES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp
LIB_FILES += $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
USER_CXXFLAGS += -Wno-conversion -Wno-arith-conversion -Wno-double-promotion -Wno-cast-qual

###################################################
# Set third party ImGuiFileDialog (for BehaviorTree IDE)
#
IMGUIFILEDIALOG_DIR := $(THIRD_PARTIES_DIR)/ImGuiFileDialog
INCLUDES += $(IMGUIFILEDIALOG_DIR)
VPATH += $(IMGUIFILEDIALOG_DIR)
LIB_FILES += $(IMGUIFILEDIALOG_DIR)/ImGuiFileDialog.cpp

###############################################################################
# Sharable information between all Makefiles
#
include $(M)/rules/Makefile