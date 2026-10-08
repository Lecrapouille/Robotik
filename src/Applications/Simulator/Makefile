# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

P := ../../..
M := $(P)/.makefile

include $(P)/Makefile.common
TARGET_NAME := Robotik-Simulator
TARGET_DESCRIPTION := Operator interface: world, robot camera, behavior tree and skills
include $(M)/project/Makefile

CURRENT_DIR := $(P)/src/Applications/Simulator
PICK_DIR := $(P)/src/Applications/Demos/PickAndPlace
LINE_DIR := $(P)/src/Applications/Demos/LineFollower
FLY_DIR := $(P)/src/Applications/Demos/FlyBrain
INCLUDES += $(CURRENT_DIR) $(PICK_DIR) $(LINE_DIR) $(FLY_DIR) $(P)/include $(P)/src
VPATH += $(CURRENT_DIR) $(PICK_DIR) $(LINE_DIR) $(FLY_DIR)
INTERNAL_LIBS += $(call internal-lib,robotik-core)
SRC_FILES := $(CURRENT_DIR)/main.cpp $(CURRENT_DIR)/App.cpp
SRC_FILES += $(CURRENT_DIR)/Window.cpp $(CURRENT_DIR)/SimFrame.cpp
SRC_FILES += $(CURRENT_DIR)/MainLoop.cpp $(CURRENT_DIR)/Panels.cpp
SRC_FILES += $(CURRENT_DIR)/SimulatorView.cpp $(CURRENT_DIR)/TeachPanel.cpp
SRC_FILES += $(CURRENT_DIR)/FlyHost.cpp
SRC_FILES += $(PICK_DIR)/ColorDetector.cpp $(PICK_DIR)/PickPlaceMission.cpp
SRC_FILES += $(PICK_DIR)/PickPlaceControl.cpp
SRC_FILES += $(LINE_DIR)/Track.cpp $(LINE_DIR)/DiffDrive.cpp
SRC_FILES += $(LINE_DIR)/Floor.cpp $(LINE_DIR)/Vision.cpp
SRC_FILES += $(LINE_DIR)/Skills.cpp $(LINE_DIR)/LineFollowerMission.cpp
SRC_FILES += $(FLY_DIR)/FlyScenario.cpp $(FLY_DIR)/FlyVision.cpp
SRC_FILES += $(FLY_DIR)/FlyController.cpp $(FLY_DIR)/ShiuNetwork.cpp
SRC_FILES += $(FLY_DIR)/FlyBrain.cpp $(FLY_DIR)/FlyEnvironment.cpp
SRC_FILES += $(FLY_DIR)/FlyTrace.cpp $(FLY_DIR)/FlyDraw.cpp
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
IMGUI_FILES += $(IMGUI_DIR)/misc/cpp/imgui_stdlib.cpp
SRC_FILES += $(IMGUI_FILES)
INCLUDES += $(IMGUI_DIR) $(IMGUI_DIR)/backends $(IMGUI_DIR)/misc/cpp
VPATH += $(IMGUI_DIR) $(IMGUI_DIR)/backends $(IMGUI_DIR)/misc/cpp

USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion
USER_CXXFLAGS += -Wno-float-equal -Wno-undef -Wno-shadow -Wno-double-promotion
USER_CXXFLAGS += -Wno-cast-qual -Wno-switch-default -Wno-useless-cast
USER_CXXFLAGS += -Wno-zero-as-null-pointer-constant -Wno-format-nonliteral
USER_CXXFLAGS += -Wno-suggest-attribute=noreturn

USER_CXXFLAGS += $(patsubst -I%,-isystem %,$(shell pkg-config --cflags-only-I opencv4))
USER_LDFLAGS += -Wl,-rpath-link,$(shell pkg-config --variable=libdir opencv4)
USER_LDFLAGS += $(shell pkg-config --libs-only-L opencv4) -lopencv_imgproc -lopencv_core
APRILTAG_DIR := $(THIRD_PARTIES_DIR)/apriltag
INCLUDES += $(APRILTAG_DIR)
THIRD_PARTIES_LIBS += $(abspath $(APRILTAG_DIR)/build/libapriltag.a)

include $(M)/rules/Makefile
