# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Read by the CMake of Pinocchio before its own file (CMAKE_PROJECT_INCLUDE)
# in the web build.

# Pinocchio sets Boost_USE_STATIC_LIBS to OFF, but Emscripten has only
# static libraries: Boost would then refuse them. Turn it back on each time.
function(robotik_static_boost p_variable p_access p_value)
    if(p_access STREQUAL "MODIFIED_ACCESS" AND NOT p_value)
        set(Boost_USE_STATIC_LIBS ON PARENT_SCOPE)
    endif()
endfunction()
variable_watch(Boost_USE_STATIC_LIBS robotik_static_boost)

# The static package of urdfdom names console_bridge and tinyxml2 without
# finding them.
find_package(console_bridge CONFIG REQUIRED)
# tinyxml2 is the one of MuJoCo, installed without a CMake package.
if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 STATIC IMPORTED)
    set_target_properties(tinyxml2::tinyxml2 PROPERTIES
        IMPORTED_LOCATION ${CMAKE_INSTALL_PREFIX}/lib/libtinyxml2.a
        INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_INSTALL_PREFIX}/include)
endif()
