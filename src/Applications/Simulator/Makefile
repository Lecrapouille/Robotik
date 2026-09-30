###############################################################################
P := ../../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-Simulator
TARGET_DESCRIPTION := Operator interface: world, robot camera, behavior tree and skills
include $(M)/project/Makefile

CURRENT_DIR := $(P)/src/Applications/Simulator
INCLUDES += $(CURRENT_DIR) $(P)/include $(P)/src
VPATH += $(CURRENT_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(CURRENT_DIR)/main.cpp $(CURRENT_DIR)/Panels.cpp
PKG_LIBS += glfw3

# Headers only: BlackThorn is compiled into robotik-core.
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

# Dear ImGui (docking branch) with the GLFW and OpenGL 3 backends.
IMGUI_DIR := $(THIRD_PARTIES_DIR)/imgui
IMGUI_FILES := $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_draw.cpp
IMGUI_FILES += $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui_widgets.cpp
IMGUI_FILES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp
IMGUI_FILES += $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
SRC_FILES += $(IMGUI_FILES)
INCLUDES += $(IMGUI_DIR) $(IMGUI_DIR)/backends
VPATH += $(IMGUI_DIR) $(IMGUI_DIR)/backends

USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion
USER_CXXFLAGS += -Wno-float-equal -Wno-undef -Wno-shadow -Wno-double-promotion
USER_CXXFLAGS += -Wno-cast-qual -Wno-switch-default -Wno-useless-cast
USER_CXXFLAGS += -Wno-zero-as-null-pointer-constant -Wno-format-nonliteral
USER_CXXFLAGS += -Wno-suggest-attribute=noreturn

include $(M)/rules/Makefile
