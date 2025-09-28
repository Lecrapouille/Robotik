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
P := ../..
M := $(P)/.makefile

###############################################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := Robotik-viewer
TARGET_DESCRIPTION := Demo of Viewer usage
include $(M)/project/Makefile

###############################################################################
# Inform Makefile where to find header files
#
INCLUDES += $(P)/src $(P)/include

###############################################################################
# Make the list of files to compile
#
SRC_FILES += Application.cpp OpenGLWindow.cpp RobotViewerApplication.cpp
SRC_FILES += Camera.cpp GeometryRenderer.cpp MeshManager.cpp
SRC_FILES += ShaderManager.cpp STLLoader.cpp RobotManager.cpp
SRC_FILES += main.cpp
INTERNAL_LIBS := $(call internal-lib,$(PROJECT_NAME))

###############################################################################
# Set libraries (Eigen3 and OpenGL for visualization)
#
PKG_LIBS += eigen3 glfw3 glew gl
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Sharable information between all Makefiles
#
include $(M)/rules/Makefile