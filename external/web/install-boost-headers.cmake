# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Copies the headers of every Boost library in the prefix. Pinocchio uses
# many header-only libraries (math, fusion, variant, asio...); asking CMake
# to build them would also build Boost.Context, whose assembly has no
# WebAssembly version.
# cmake -DSOURCE=<boost> -DPREFIX=<prefix> -P install-boost-headers.cmake

file(GLOB includes ${SOURCE}/libs/*/include ${SOURCE}/libs/numeric/*/include)
foreach(include ${includes})
    if(IS_DIRECTORY ${include}/boost)
        file(COPY ${include}/boost DESTINATION ${PREFIX}/include)
    endif()
endforeach()
