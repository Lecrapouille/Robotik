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
# Set units library
#
INCLUDES += $(THIRD_PARTIES_DIR) $(THIRD_PARTIES_DIR)/units/include

###############################################################################
# Set OpenGL libraries (for Viewer)
#
PKG_LIBS += glfw3 glew

###############################################################################
# Set pugixml library
#
SRC_FILES += $(THIRD_PARTIES_DIR)/pugixml/src/pugixml.cpp
INCLUDES += $(THIRD_PARTIES_DIR)/pugixml/src
USER_CXXFLAGS += -Wno-switch-enum -Wno-null-dereference

###################################################
# Set third party Dear ImGui
#
IMGUI_DIR := $(THIRD_PARTIES_DIR)/imgui
INCLUDES += $(IMGUI_DIR) $(IMGUI_DIR)/misc/cpp $(IMGUI_DIR)/backends
VPATH += $(IMGUI_DIR) $(IMGUI_DIR)/misc/cpp $(IMGUI_DIR)/backends
SRC_FILES += $(IMGUI_DIR)/imgui_widgets.cpp $(IMGUI_DIR)/imgui_draw.cpp
SRC_FILES += $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui.cpp
SRC_FILES += $(IMGUI_DIR)/misc/cpp/imgui_stdlib.cpp
SRC_FILES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp
SRC_FILES += $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
USER_CXXFLAGS += -Wno-conversion -Wno-arith-conversion -Wno-double-promotion -Wno-cast-qual

###############################################################################
# GTest and GMock
#
PKG_LIBS += gtest gmock

###################################################
# Compile the unit tests
#
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Core,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/src/Robotik/Renderer,*.cpp)
SRC_FILES += $(call rwildcard,$(P)/tests,*.cpp)

INCLUDES += $(P)/src $(P)/tests
VPATH += $(P)/src

###################################################
# Generic Makefile rules
#
include $(M)/rules/Makefile