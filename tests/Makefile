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
# Eigen3 is set by Makefile.common. These relaxations remain needed by the
# templates it expands inside our own code.
#
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
# Set SWI-Prolog library, needed by the Core sources compiled below
#
PKG_LIBS += swipl

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

###################################################
# Set BlackThorn and its editor Oakular, mirroring what robotik-core and
# robotik-renderer embed, since the tests compile their sources directly.
#
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk

INCLUDES += $(BLACKTHORN_INCLUDES) $(OAKULAR_INCLUDES)
VPATH += $(BLACKTHORN_VPATH) $(OAKULAR_VPATH)
SRC_FILES += $(BLACKTHORN_SOURCES) $(OAKULAR_SOURCES)
DEFINES += $(BLACKTHORN_DEFINES) $(OAKULAR_DEFINES)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS) $(OAKULAR_PKG_LIBS)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS) $(OAKULAR_CXXFLAGS)
THIRD_PARTIES_LIBS += $(BLACKTHORN_STATIC_DEPS)
DEFINES += -DIMGUI_DEFINE_MATH_OPERATORS

###################################################
# Set third party ImGuiFileDialog
#
IMGUIFILEDIALOG_DIR := $(THIRD_PARTIES_DIR)/ImGuiFileDialog
INCLUDES += $(IMGUIFILEDIALOG_DIR)
VPATH += $(IMGUIFILEDIALOG_DIR)
SRC_FILES += $(IMGUIFILEDIALOG_DIR)/ImGuiFileDialog.cpp

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

###################################################
# Build rapidyaml on demand, as done by src/Robotik/Core/Makefile.
#
$(BLACKTHORN_STATIC_DEPS):
	@$(MAKE) --no-print-directory -C $(P) compile-external-libs

$(TARGET_BINARY_NAME): $(BLACKTHORN_STATIC_DEPS)