# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# Read by each third party of the web build after its project()
# (CMAKE_PROJECT_INCLUDE). The page links static libraries only, but some
# projects write add_library(x SHARED) (urdfdom), and a recent Emscripten
# can make a shared library. With this property, CMake builds a static
# library instead.
set_property(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS FALSE)
