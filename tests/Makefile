###################################################
# Location of the project directory and Makefiles
#
P := ..
M := $(P)/.makefile

###################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := $(PROJECT_NAME)-tests
TARGET_DESCRIPTION := Unit tests for $(PROJECT_NAME)
COMPILATION_MODE := test
CXX_STANDARD := --std=c++17
include $(M)/project/Makefile

###############################################################################
# Set Eigen3 library
#
PKG_LIBS += eigen3
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set TinyXML2 library
#
SRC_FILES += $(THIRD_PARTIES_DIR)/tinyxml2/tinyxml2.cpp

###############################################################################
# Set units library
#
INCLUDES += $(THIRD_PARTIES_DIR) $(THIRD_PARTIES_DIR)/units/include

###############################################################################
# Set OpenGL libraries (for Viewer)
#
PKG_LIBS += glfw3 glew

###############################################################################
# GTest and GMock
#
PKG_LIBS += gtest gmock

###################################################
# Compile the unit tests
#
SRC_FILES += $(wildcard $(P)/src/Common/*.cpp)
SRC_FILES += $(wildcard $(P)/src/Loaders/*.cpp)
SRC_FILES += $(wildcard $(P)/src/Robot/Blueprint/*.cpp)
SRC_FILES += $(wildcard $(P)/src/Robot/*.cpp)
SRC_FILES += $(wildcard $(P)/src/Simulation/*.cpp)
SRC_FILES += $(wildcard $(P)/src/Solvers/*.cpp)
SRC_FILES += $(wildcard $(P)/src/Renderer/*/*.cpp)
SRC_FILES += $(wildcard $(P)/src/Renderer/*.cpp)
SRC_FILES += $(wildcard $(P)/tests/*.cpp)
INCLUDES += $(P)/include
INCLUDES += $(P)/src
VPATH += $(P)/src

###################################################
# Generic Makefile rules
#
include $(M)/rules/Makefile