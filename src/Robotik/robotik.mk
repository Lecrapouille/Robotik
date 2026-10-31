# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The core library: robots, worlds, planning, behavior trees, Petri nets.
# Shared, so that the simulator and the plugins it loads share one copy.
# Paths are relative to this directory.

TARGETS += robotik-core
robotik-core.TYPE := shared
robotik-core.DESCRIPTION := Core library for Robotik
# The backends of the browser, or of the desktop.
ifeq ($(TARGET_OS),Emscripten)
    ROBOTIK_EXCLUDED := %/PinocchioBackend.cpp %/MujocoBackend.cpp %/KairosPlanner.cpp
else
    ROBOTIK_EXCLUDED := %/WebBackends.cpp %/WebPlanner.cpp
endif
robotik-core.SOURCES := $(patsubst $(HERE)%,%,$(filter-out $(ROBOTIK_EXCLUDED),$(call rwildcard,$(patsubst %/,%,$(HERE)),*.cpp)))
robotik-core.PUBLIC_INCLUDES := .. ../../include
robotik-core.DEPENDS := Compages blackthorn TimedPetriNet TimedPetriIO TimedPetriAnalysis \
    TimedPetriMaxPlus pugixml $(if $(filter forge,$(TARGETS)),forge)
robotik-core.CXXFLAGS := -Wno-switch-enum -Wno-old-style-cast -Wno-sign-conversion \
    -Wno-duplicated-branches -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal
ifneq ($(TARGET_OS),Emscripten)
    robotik-core.DEPENDS += kairos
    robotik-core.LIBS := -ldl
    ifeq ($(filter forge,$(TARGETS)),)
        robotik-core.PKG_CONFIG := eigen3
    endif
else
    robotik-core.CXXFLAGS += -Wno-unknown-warning-option -Wno-unused-command-line-argument
    robotik-core.DEPENDS += eigen
endif
robotik-core.PROJECT_INFO := 1
# The executables of BUILD/bin find the asset packs in ../share/robotik/packs,
# as once installed: a link to data/packs.
ROBOTIK_PACKS := $(abspath $(HERE)../../data/packs)
robotik-core.POST_BUILD := mkdir -p $$(BUILD)/share/robotik && ln -sfn $(ROBOTIK_PACKS) $$(BUILD)/share/robotik/packs
