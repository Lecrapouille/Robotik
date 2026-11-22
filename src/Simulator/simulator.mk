# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Operator interface: world, robot camera, behavior tree and skills. Paths
# are relative to this directory.

TARGETS += RobotIK-Simulator
RobotIK-Simulator.DESCRIPTION := Operator interface: world, robot camera, behavior tree and skills
RobotIK-Simulator.SOURCES := *.cpp
RobotIK-Simulator.INCLUDES := .
RobotIK-Simulator.DEPENDS := robotik-core Compages imgui implot stb
RobotIK-Simulator.DEFINES = ROBOTIK_FONTS_DIR="$(imgui.DIR)misc/fonts"
RobotIK-Simulator.CXXFLAGS := -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion \
    -Wno-float-equal -Wno-undef -Wno-shadow -Wno-double-promotion -Wno-cast-qual \
    -Wno-switch-default -Wno-useless-cast -Wno-zero-as-null-pointer-constant \
    -Wno-format-nonliteral -Wno-suggest-attribute=noreturn
ifeq ($(TARGET_OS),Emscripten)
    RobotIK-Simulator.CXXFLAGS += -Wno-unknown-warning-option -Wno-unused-parameter
    ifdef EXAEQUOS
        PKG_CONFIG_STATIC := 1
        RobotIK-Simulator.PKG_CONFIG := exa-wayland glfw
        RobotIK-Simulator.LDFLAGS := -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sFULL_ES3
    else
        RobotIK-Simulator.LDFLAGS := -sUSE_GLFW=3 -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sFULL_ES3 \
            --shell-file $(abspath $(HERE)shell.html) \
            --preload-file $(abspath $(HERE)../../plugins)@/plugins \
            --exclude-file '*/external/*' --exclude-file '*.csv' --exclude-file '*.stl' \
            --exclude-file '*.onnx' --exclude-file '*.so' --exclude-file '*.a' --exclude-file '*.o' \
            -sFORCE_FILESYSTEM=1 -sASYNCIFY -sALLOW_MEMORY_GROWTH=1 -sGL_ENABLE_GET_PROC_ADDRESS
    endif
else
    RobotIK-Simulator.PKG_CONFIG := glfw3
    RobotIK-Simulator.LIBS := -ldl
endif

# Editor lifecycle regression checks use ImGui without a window or GPU.
ifneq ($(TARGET_OS),Emscripten)
TARGETS += RobotIK-Simulator-tests
RobotIK-Simulator-tests.TYPE := test
RobotIK-Simulator-tests.DESCRIPTION := Simulator editor identity and layout restoration
RobotIK-Simulator-tests.SOURCES := tests/*.cpp $(filter-out main.cpp,$(notdir $(wildcard $(HERE)*.cpp)))
RobotIK-Simulator-tests.INCLUDES := .
RobotIK-Simulator-tests.DEPENDS := $(RobotIK-Simulator.DEPENDS)
RobotIK-Simulator-tests.DEFINES = $(RobotIK-Simulator.DEFINES)
RobotIK-Simulator-tests.CXXFLAGS := $(RobotIK-Simulator.CXXFLAGS)
RobotIK-Simulator-tests.PKG_CONFIG := glfw3 gtest_main
RobotIK-Simulator-tests.LIBS := -ldl
endif
