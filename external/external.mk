# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Third parties, fetched on the first build; their commits are locked in
# external/versions.lock (commit it). Compages and BlackThorn are built
# by MyMakefile 3: their own .mk files declare
# their libraries and third parties. For a local copy of one of them:
#   make CompagesProject.PATH=$$HOME/MyGithub/Compages

TARGETS += CompagesProject BlackThorn imgui implot

### Compages: the GPU, world and renderer layers. License: GPL-3.0
CompagesProject.GIT := https://github.com/Lecrapouille/Compages.git
CompagesProject.MAKEFILES := Compages.mk external/external.mk

### BlackThorn: behavior trees (with rapidyaml). License: GPL-3.0
BlackThorn.GIT := https://github.com/Lecrapouille/BlackThorn.git
BlackThorn.MAKEFILES := BlackThorn.mk external/external.mk

### Dear ImGui, with the GLFW and OpenGL 3 backends: the one of the
### simulator, also used by BlackThorn. License: MIT
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

### Kairos: task planning. License: Apache-2.0
TARGETS += kairos
kairos.GIT := https://github.com/PlanSys2/kairos.git
kairos.VERSION := 1.0.0
kairos.TYPE := cmake

ifeq ($(TARGET_OS),Emscripten)
    ### Eigen, header only: pkg-config of Emscripten does not see the one of
    ### the system. License: MPL-2.0
    TARGETS += eigen
    eigen.GIT := https://gitlab.com/libeigen/eigen.git
    eigen.VERSION := 3.4.0
    eigen.PUBLIC_INCLUDES := .

    # The page has no shared library: Kairos is its static libraries.
    kairos.CMAKE_ARGS := -DKAIROS_BUILD_TESTS=OFF -DKAIROS_BUILD_CLI=OFF -DKAIROS_SHARED=OFF
    kairos.LIBRARIES := kairos kairos_orchestrator kairos_validate kairos_temporal kairos_plan \
        kairos_search kairos_heuristics kairos_sas kairos_normalize kairos_ground kairos_lift \
        kairos_state kairos_semantic kairos_pddl kairos_core

    ### MuJoCo and Pinocchio of the web page, built by external/web/CMakeLists.txt
    ### (the desktop takes them from conda-forge). Their sources are fetched
    ### here; the targets below are only these sources.
    ### MuJoCo: physics. License: Apache-2.0
    TARGETS += mujoco-src
    mujoco-src.GIT := https://github.com/google-deepmind/mujoco.git
    mujoco-src.VERSION := 3.16.0
    ### Pinocchio: kinematics. License: BSD-2-Clause
    TARGETS += pinocchio-src
    pinocchio-src.GIT := https://github.com/stack-of-tasks/pinocchio.git
    pinocchio-src.VERSION := v4.1.0
    pinocchio-src.SUBMODULES := 1
    ### urdfdom: URDF reader of Pinocchio. License: BSD-3-Clause
    TARGETS += urdfdom-src urdfdom_headers-src console_bridge-src
    urdfdom-src.GIT := https://github.com/ros/urdfdom.git
    urdfdom-src.VERSION := 6.0.2
    urdfdom_headers-src.GIT := https://github.com/ros/urdfdom_headers.git
    urdfdom_headers-src.VERSION := 3.0.1
    console_bridge-src.GIT := https://github.com/ros/console_bridge.git
    console_bridge-src.VERSION := 1.0.2
    ### Boost: used by Pinocchio. License: BSL-1.0
    TARGETS += boost-src
    boost-src.URL := https://github.com/boostorg/boost/releases/download/boost-1.90.0/boost-1.90.0-cmake.tar.xz
    boost-src.SHA256 := aca59f889f0f32028ad88ba6764582b63c916ce5f77b31289ad19421a96c555f
    boost-src.SUBDIR := boost-1.90.0

    TARGETS += webphysics
    webphysics.TYPE := cmake
    webphysics.PATH := web
    # = and not :=, the folders are known once the sources are fetched.
    # The checksum of external/web/ builds again when its files change.
    WEB_PHYSICS_SUM := $(firstword $(shell cat $(sort $(wildcard $(HERE)web/*)) | cksum))
    webphysics.CMAKE_ARGS = -DROBOTIK_WEB_FILES=$(WEB_PHYSICS_SUM) -DMUJOCO_SOURCE=$(abspath $(mujoco-src.DIR)) \
        -DPINOCCHIO_SOURCE=$(abspath $(pinocchio-src.DIR)) -DEIGEN_SOURCE=$(abspath $(eigen.DIR)) \
        -DURDFDOM_SOURCE=$(abspath $(urdfdom-src.DIR)) -DURDFDOM_HEADERS_SOURCE=$(abspath $(urdfdom_headers-src.DIR)) \
        -DCONSOLE_BRIDGE_SOURCE=$(abspath $(console_bridge-src.DIR)) -DBOOST_SOURCE=$(abspath $(boost-src.DIR))
    webphysics.PUBLIC_INCLUDES := include include/urdfdom_headers
    webphysics.LIBRARIES := pinocchio_parsers pinocchio_default urdfdom_model console_bridge \
        boost_filesystem boost_serialization boost_atomic \
        mujoco ccd qhullstatic_r tinyobjloader miniz tinyxml2
    webphysics.PUBLIC_DEFINES := PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION PINOCCHIO_WITH_URDFDOM \
        PINOCCHIO_URDFDOM_HEADERS_MAJOR_VERSION=3 PINOCCHIO_URDFDOM_HEADERS_MINOR_VERSION=0 \
        PINOCCHIO_URDFDOM_HEADERS_PATCH_VERSION=1 BOOST_FUSION_INVOKE_MAX_ARITY=12
else
    kairos.CMAKE_ARGS := -DKAIROS_BUILD_TESTS=OFF -DKAIROS_BUILD_CLI=ON -DKAIROS_SHARED=ON
    kairos.LIBRARIES := libkairos.so

    ### Pinocchio and MuJoCo, from conda-forge in external/forge ('make forge').
    ### Eigen from external/forge, else from the system.
    ifneq ($(wildcard $(HERE)forge/include/pinocchio/parsers/urdf.hpp),)
        TARGETS += forge
        forge.PATH := forge
        forge.PUBLIC_INCLUDES := include include/eigen3
        # Under AddressSanitizer Eigen aligns its heap blocks by hand, but
        # the Pinocchio of conda was built without it: a matrix allocated in
        # libpinocchio and freed in our code would free a wrong address.
        ifneq ($(filter address,$(subst $(COMMA), ,$(SANITIZER))),)
            forge.PUBLIC_DEFINES := EIGEN_MALLOC_ALREADY_ALIGNED=1
        endif
        forge.LIBS := -L$(abspath $(HERE)forge/lib) -Wl,-rpath,$(abspath $(HERE)forge/lib) \
            -lpinocchio_parsers -lpinocchio_default -lmujoco
    endif
endif
