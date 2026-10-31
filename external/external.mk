# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Third parties, fetched on the first build; their commits are locked in
# external/versions.lock (commit it). Compages, BlackThorn and
# TimedPetriNetEditor are built by MyMakefile 3: their own .mk files declare
# their libraries and third parties. For a local copy of one of them:
#   make CompagesProject.PATH=$$HOME/MyGithub/Compages

TARGETS += CompagesProject BlackThorn TimedPetriNetEditor imgui implot

### Compages: the GPU, world and renderer layers. License: GPL-3.0
CompagesProject.GIT := https://github.com/Lecrapouille/Compages.git
CompagesProject.MAKEFILES := Compages.mk external/external.mk

### BlackThorn: behavior trees (with rapidyaml). License: GPL-3.0
BlackThorn.GIT := https://github.com/Lecrapouille/BlackThorn.git
BlackThorn.MAKEFILES := BlackThorn.mk external/external.mk

### TimedPetriNetEditor: Petri nets, GRAFCET and their viewer. License: GPL-3.0
TimedPetriNetEditor.GIT := https://github.com/Lecrapouille/TimedPetriNetEditor.git
TimedPetriNetEditor.MAKEFILES := TimedPetriNet.mk external/external.mk

### Dear ImGui, with the GLFW and OpenGL 3 backends: the one of the
### simulator, also used by BlackThorn and TimedPetriNetEditor. License: MIT
imgui.GIT := https://github.com/ocornut/imgui.git
imgui.VERSION := docking
imgui.TYPE := static
imgui.SOURCES := imgui.cpp imgui_draw.cpp imgui_tables.cpp imgui_widgets.cpp \
    misc/cpp/imgui_stdlib.cpp backends/imgui_impl_glfw.cpp backends/imgui_impl_opengl3.cpp
imgui.PUBLIC_INCLUDES := . backends misc/cpp
ifeq ($(TARGET_OS),Emscripten)
    ifdef EXAEQUOS
        imgui.PKG_CONFIG := glfw
    endif
else
    imgui.PKG_CONFIG := glfw3
endif

### Immediate Mode Plotting. License: MIT
implot.GIT := https://github.com/epezent/implot.git
implot.TYPE := static
implot.SOURCES := implot.cpp implot_items.cpp
implot.PUBLIC_INCLUDES := .
implot.DEPENDS := imgui

ifeq ($(TARGET_OS),Emscripten)
    ### Eigen, header only: pkg-config of Emscripten does not see the one of
    ### the system. License: MPL-2.0
    TARGETS += eigen
    eigen.GIT := https://gitlab.com/libeigen/eigen.git
    eigen.VERSION := 3.4.0
    eigen.PUBLIC_INCLUDES := .
else
    ### Kairos: task planning. License: Apache-2.0
    TARGETS += kairos
    kairos.GIT := https://github.com/PlanSys2/kairos.git
    kairos.VERSION := 1.0.0
    kairos.TYPE := cmake
    kairos.CMAKE_ARGS := -DKAIROS_BUILD_TESTS=OFF -DKAIROS_BUILD_CLI=ON -DKAIROS_SHARED=ON
    kairos.LIBRARIES := libkairos.so

    ### Pinocchio and MuJoCo, from conda-forge in external/forge ('make forge').
    ### Eigen from external/forge, else from the system.
    ifneq ($(wildcard $(HERE)forge/include/pinocchio/parsers/urdf.hpp),)
        TARGETS += forge
        forge.PATH := forge
        forge.PUBLIC_INCLUDES := include include/eigen3
        forge.LIBS := -L$(abspath $(HERE)forge/lib) -Wl,-rpath,$(abspath $(HERE)forge/lib) \
            -lpinocchio_parsers -lpinocchio_default -lmujoco
    endif
endif
