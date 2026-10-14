# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-Simulator
TARGET_DESCRIPTION := Operator interface: world, robot camera, behavior tree and skills
include $(M)/project/Makefile

CURRENT_DIR := $(P)/simulator
INCLUDES += $(CURRENT_DIR) $(P)/include $(P)/src
VPATH += $(CURRENT_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(CURRENT_DIR)/main.cpp $(CURRENT_DIR)/App.cpp
SRC_FILES += $(CURRENT_DIR)/Window.cpp $(CURRENT_DIR)/SimFrame.cpp
SRC_FILES += $(CURRENT_DIR)/MainLoop.cpp $(CURRENT_DIR)/SimulatorView.cpp
SRC_FILES += $(CURRENT_DIR)/Workspace.cpp $(CURRENT_DIR)/Widgets.cpp
SRC_FILES += $(CURRENT_DIR)/ScenePanel.cpp $(CURRENT_DIR)/CameraPanel.cpp
SRC_FILES += $(CURRENT_DIR)/OverviewPanels.cpp $(CURRENT_DIR)/PackagePanels.cpp
SRC_FILES += $(CURRENT_DIR)/RobotPanel.cpp
SRC_FILES += $(CURRENT_DIR)/TeachPanel.cpp $(CURRENT_DIR)/MissionPanels.cpp
SRC_FILES += $(CURRENT_DIR)/SkillsPanel.cpp $(CURRENT_DIR)/AutonomyPanels.cpp
SRC_FILES += $(CURRENT_DIR)/PluginUi.cpp
PKG_LIBS += glfw3

BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
include $(BLACKTHORN_DIR)/BlackThorn.mk
INCLUDES += $(BLACKTHORN_INCLUDES)
DEFINES += $(BLACKTHORN_DEFINES)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)

IMGUI_DIR := $(THIRD_PARTIES_DIR)/imgui
IMGUI_FILES := $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_draw.cpp
IMGUI_FILES += $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui_widgets.cpp
IMGUI_FILES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp
IMGUI_FILES += $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
IMGUI_FILES += $(IMGUI_DIR)/misc/cpp/imgui_stdlib.cpp
SRC_FILES += $(IMGUI_FILES)
INCLUDES += $(IMGUI_DIR) $(IMGUI_DIR)/backends $(IMGUI_DIR)/misc/cpp
VPATH += $(IMGUI_DIR) $(IMGUI_DIR)/backends $(IMGUI_DIR)/misc/cpp

# Joint plots of the Robot workspace.
IMPLOT_DIR := $(THIRD_PARTIES_DIR)/implot
SRC_FILES += $(IMPLOT_DIR)/implot.cpp $(IMPLOT_DIR)/implot_items.cpp
INCLUDES += $(IMPLOT_DIR)
VPATH += $(IMPLOT_DIR)

# User interface fonts, found next to the executable.
DEFINES += -DROBOTIK_FONTS_DIR=\"$(abspath $(IMGUI_DIR)/misc/fonts)\"

USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion
USER_CXXFLAGS += -Wno-float-equal -Wno-undef -Wno-shadow -Wno-double-promotion
USER_CXXFLAGS += -Wno-cast-qual -Wno-switch-default -Wno-useless-cast
USER_CXXFLAGS += -Wno-zero-as-null-pointer-constant -Wno-format-nonliteral
USER_CXXFLAGS += -Wno-suggest-attribute=noreturn
USER_LDFLAGS += -ldl

include $(M)/rules/Makefile
include $(P)/Makefile.relink
