# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Installs MuJoCo built with Emscripten, which has no install rules there:
# its headers, its static library and the ones it fetched (ccd, qhull,
# tinyxml2...), with the header of tinyxml2 for urdfdom.
# cmake -DSOURCE=<mujoco> -DBINARY=<build> -DPREFIX=<prefix> -P install-mujoco.cmake

file(COPY ${SOURCE}/include/mujoco DESTINATION ${PREFIX}/include)
file(GLOB libraries ${BINARY}/lib/*.a)
if(NOT libraries)
    message(FATAL_ERROR "no MuJoCo library in ${BINARY}/lib")
endif()
file(COPY ${libraries} DESTINATION ${PREFIX}/lib)
file(GLOB_RECURSE tinyxml2 ${BINARY}/_deps/tinyxml2-src/tinyxml2.h)
if(NOT tinyxml2)
    message(FATAL_ERROR "no tinyxml2.h in ${BINARY}/_deps")
endif()
file(COPY ${tinyxml2} DESTINATION ${PREFIX}/include)
