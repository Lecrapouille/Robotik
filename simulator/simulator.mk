# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Operator interface: world, robot camera, behavior tree and skills. Paths
# are relative to this directory.

TARGETS += Robotik-Simulator
Robotik-Simulator.DESCRIPTION := Operator interface: world, robot camera, behavior tree and skills
Robotik-Simulator.SOURCES := *.cpp
Robotik-Simulator.INCLUDES := .
Robotik-Simulator.DEPENDS := robotik-core Compages imgui implot stb
Robotik-Simulator.DEFINES = ROBOTIK_FONTS_DIR="$(imgui.DIR)misc/fonts"
Robotik-Simulator.CXXFLAGS := -Wno-old-style-cast -Wno-sign-conversion -Wno-conversion \
    -Wno-float-equal -Wno-undef -Wno-shadow -Wno-double-promotion -Wno-cast-qual \
    -Wno-switch-default -Wno-useless-cast -Wno-zero-as-null-pointer-constant \
    -Wno-format-nonliteral -Wno-suggest-attribute=noreturn
ifeq ($(TARGET_OS),Emscripten)
    Robotik-Simulator.CXXFLAGS += -Wno-unknown-warning-option -Wno-unused-parameter
    ifdef EXAEQUOS
        PKG_CONFIG_STATIC := 1
        Robotik-Simulator.PKG_CONFIG := exa-wayland glfw
        Robotik-Simulator.LDFLAGS := -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sFULL_ES3
    else
        Robotik-Simulator.LDFLAGS := -sUSE_GLFW=3 -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sFULL_ES3 \
            --shell-file $(abspath $(HERE)shell.html) \
            --preload-file $(abspath $(HERE)../plugins)@/plugins \
            --exclude-file '*/external/*' --exclude-file '*.csv' --exclude-file '*.stl' \
            --exclude-file '*.onnx' --exclude-file '*.so' --exclude-file '*.a' --exclude-file '*.o' \
            -sFORCE_FILESYSTEM=1 -sASYNCIFY -sALLOW_MEMORY_GROWTH=1 -sGL_ENABLE_GET_PROC_ADDRESS
    endif
else
    Robotik-Simulator.PKG_CONFIG := glfw3
    Robotik-Simulator.LIBS := -ldl
endif

# Editor lifecycle regression checks use ImGui without a window or GPU.
ifneq ($(TARGET_OS),Emscripten)
TARGETS += Robotik-Simulator-tests
Robotik-Simulator-tests.TYPE := test
Robotik-Simulator-tests.DESCRIPTION := Simulator editor identity and layout restoration
Robotik-Simulator-tests.SOURCES := tests/*.cpp $(filter-out main.cpp,$(notdir $(wildcard simulator/*.cpp)))
Robotik-Simulator-tests.INCLUDES := .
Robotik-Simulator-tests.DEPENDS := $(Robotik-Simulator.DEPENDS)
Robotik-Simulator-tests.DEFINES = $(Robotik-Simulator.DEFINES)
Robotik-Simulator-tests.CXXFLAGS := $(Robotik-Simulator.CXXFLAGS)
Robotik-Simulator-tests.PKG_CONFIG := glfw3 gtest_main
Robotik-Simulator-tests.LIBS := -ldl
endif
