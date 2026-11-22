# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The physics and kinematics of the web page, built with Emscripten: MuJoCo,
# Pinocchio and what Pinocchio needs (Boost, console_bridge, urdfdom).
# MyMakefile fetches the sources (external/external.mk, target webphysics)
# and gives their folders here; this project builds them in order and
# installs them, as static libraries, in one prefix.
#
# Why a project of its own: each library finds the ones before it with
# find_package, so they must share a prefix, and MuJoCo has no install rules
# with Emscripten. On the desktop these libraries come from conda-forge
# (make forge), not from here.

cmake_minimum_required(VERSION 3.16)
project(RobotIKWebPhysics NONE)

include(ExternalProject)

foreach(source BOOST_SOURCE CONSOLE_BRIDGE_SOURCE URDFDOM_HEADERS_SOURCE
               URDFDOM_SOURCE PINOCCHIO_SOURCE MUJOCO_SOURCE EIGEN_SOURCE)
    if(NOT IS_DIRECTORY "${${source}}")
        message(FATAL_ERROR "${source} is not a folder: '${${source}}'")
    endif()
endforeach()

set(PREFIX "${CMAKE_INSTALL_PREFIX}")
# Emscripten has no threads in the page: Boost would then take its Windows
# mutex. Its pthread stubs are enough.
set(WEB_CXX_FLAGS "-DBOOST_HAS_PTHREADS")
set(COMMON_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_CROSSCOMPILING_EMULATOR=${CMAKE_CROSSCOMPILING_EMULATOR}
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX=${PREFIX}
    -DCMAKE_INSTALL_LIBDIR=lib
    -DCMAKE_PREFIX_PATH=${PREFIX}
    -DCMAKE_FIND_ROOT_PATH=${PREFIX}
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    -DBUILD_SHARED_LIBS=OFF
    -DCMAKE_PROJECT_INCLUDE=${CMAKE_CURRENT_SOURCE_DIR}/static-only.cmake
    -DBUILD_TESTING=OFF
    "-DCMAKE_CXX_FLAGS=${WEB_CXX_FLAGS}")

### MuJoCo: only the library and the static libraries it fetches. Its wasm
### bindings (TypeScript) are not built, and it has no install rules here.
ExternalProject_Add(mujoco
    SOURCE_DIR ${MUJOCO_SOURCE}
    CMAKE_ARGS ${COMMON_ARGS}
        -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF
        -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_BUILD_TESTS_WASM=OFF
        -DMUJOCO_TEST_PYTHON_UTIL=OFF -DMUJOCO_WASM_THREADS=OFF
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --target mujoco
    INSTALL_COMMAND ${CMAKE_COMMAND} -DSOURCE=<SOURCE_DIR> -DBINARY=<BINARY_DIR>
        -DPREFIX=${PREFIX} -P ${CMAKE_CURRENT_SOURCE_DIR}/install-mujoco.cmake)

### Eigen, headers and CMake package.
ExternalProject_Add(eigen
    SOURCE_DIR ${EIGEN_SOURCE}
    CMAKE_ARGS ${COMMON_ARGS} -DEIGEN_BUILD_DOC=OFF
    BUILD_COMMAND "")

### Boost: only filesystem and serialization are compiled (Pinocchio links
### them); the other libraries are headers, all copied in the prefix.
ExternalProject_Add(boost
    SOURCE_DIR ${BOOST_SOURCE}
    # | stands for ; in the list given to Boost (ExternalProject splits on ;).
    LIST_SEPARATOR |
    CMAKE_ARGS ${COMMON_ARGS} -DBOOST_INCLUDE_LIBRARIES=filesystem|serialization
    INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR>
    COMMAND ${CMAKE_COMMAND} -DSOURCE=<SOURCE_DIR> -DPREFIX=${PREFIX}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/install-boost-headers.cmake)

ExternalProject_Add(console_bridge
    SOURCE_DIR ${CONSOLE_BRIDGE_SOURCE}
    CMAKE_ARGS ${COMMON_ARGS})

ExternalProject_Add(urdfdom_headers
    SOURCE_DIR ${URDFDOM_HEADERS_SOURCE}
    CMAKE_ARGS ${COMMON_ARGS}
    BUILD_COMMAND "")

### urdfdom reads XML with the tinyxml2 of MuJoCo (one copy in the page).
ExternalProject_Add(urdfdom
    SOURCE_DIR ${URDFDOM_SOURCE}
    CMAKE_ARGS ${COMMON_ARGS}
        -DTINYXML2_INCLUDE_DIR=${PREFIX}/include
        -DTINYXML2_LIBRARY=${PREFIX}/lib/libtinyxml2.a
    DEPENDS mujoco console_bridge urdfdom_headers)

### Pinocchio: the URDF parser and the algorithms, no Python, no collision.
ExternalProject_Add(pinocchio
    SOURCE_DIR ${PINOCCHIO_SOURCE}
    CMAKE_ARGS ${COMMON_ARGS}
        -DCMAKE_PROJECT_INCLUDE=${CMAKE_CURRENT_SOURCE_DIR}/pinocchio-web.cmake
        -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_URDF_SUPPORT=ON
        -DBUILD_WITH_COLLISION_SUPPORT=OFF -DBUILD_EXAMPLES=OFF
        -DBUILD_BENCHMARK=OFF -DBUILD_UTILS=OFF -DPINOCCHIO_BUILD_VISUALIZERS=OFF
        -DINSTALL_DOCUMENTATION=OFF -DBUILD_DOCUMENTATION=OFF
    DEPENDS eigen boost urdfdom)
