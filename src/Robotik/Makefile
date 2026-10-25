# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. It is available under the GNU GPL v3 or,
# for users who cannot use the GPL, under a commercial license.
# See LICENSING.md for details.

# Location of the project directory and Makefiles
#
P := ../..
M := $(P)/.makefile

###############################################################################
# Project definition
#
include $(P)/Makefile.common
TARGET_NAME := robotik-core
TARGET_DESCRIPTION := Core library for Robotik
include $(M)/project/Makefile

###############################################################################
# Inform Makefile where to find header files
#
INCLUDES += $(P)/src $(P)/include $(THIRD_PARTIES_DIR)

###############################################################################
# Inform Makefile where to find *.cpp files
#
VPATH += $(P)/src

###############################################################################
# Project defines
#
DEFINES +=

###############################################################################
# Make the list of files to compile
#
LIB_FILES += $(call rwildcard,$(P)/src/Robotik,*.cpp)
ifeq ($(OS),Emscripten)
LIB_FILES := $(filter-out %/PinocchioBackend.cpp %/MujocoBackend.cpp %/KairosPlanner.cpp,$(LIB_FILES))
else
LIB_FILES := $(filter-out %/WebBackends.cpp %/WebPlanner.cpp,$(LIB_FILES))
endif
HEADER_FILES := $(call rwildcard,$(P)/src/Robotik,*.hpp)
HEADER_FILES += $(call rwildcard,$(P)/include/Robotik,*.hpp)

###############################################################################
# Set pugixml library (from thirdparties)
#
LIB_FILES += $(THIRD_PARTIES_DIR)/pugixml/src/pugixml.cpp
INCLUDES += $(THIRD_PARTIES_DIR)/pugixml/src
USER_CXXFLAGS += -Wno-switch-enum

###############################################################################
# Eigen3 is set by Makefile.common. These relaxations remain needed by the
# templates it expands inside our own code.
#
USER_CXXFLAGS += -Wno-old-style-cast -Wno-sign-conversion -Wno-duplicated-branches
USER_CXXFLAGS += -Wno-useless-cast -Wno-ctor-dtor-privacy -Wno-float-equal

###############################################################################
# Set BlackThorn behavior tree engine (from thirdparties). Its sources are
# compiled straight into this library, as done for pugixml. rapidyaml, its YAML
# backend, is cloned and built at the Robotik level: point BlackThorn at it.
#
BLACKTHORN_DIR := $(THIRD_PARTIES_DIR)/BlackThorn
RYML_DIR := $(THIRD_PARTIES_DIR)/rapidyaml
# The visualizer client talks TCP through SFML. The web build has neither.
ifeq ($(OS),Emscripten)
BLACKTHORN_WITH_NETWORK := 0
OAKULAR_WITH_SERVER := 0
endif
include $(BLACKTHORN_DIR)/BlackThorn.mk
ifeq ($(OS),Emscripten)
BLACKTHORN_STATIC_DEPS := $(RYML_STATIC)
endif

INCLUDES += $(BLACKTHORN_INCLUDES)
VPATH += $(BLACKTHORN_VPATH)
LIB_FILES += $(BLACKTHORN_SOURCES)
DEFINES += $(BLACKTHORN_DEFINES)
PKG_LIBS += $(BLACKTHORN_PKG_LIBS)
USER_CXXFLAGS += $(BLACKTHORN_CXXFLAGS)
THIRD_PARTIES_LIBS += $(BLACKTHORN_STATIC_DEPS)

ifneq ($(OS),Emscripten)
USER_LDFLAGS += -ldl
endif

###############################################################################
# TimedPetriNet (from thirdparties): nets, GRAFCET engine and (max,+)
# algorithms of TimedPetriNetEditor, compiled into this library like
# BlackThorn. Only src/Robotik/Task/Grafcet includes its headers.
#
TPNE_DIR := $(THIRD_PARTIES_DIR)/TimedPetriNetEditor
include $(TPNE_DIR)/TimedPetriNet.mk

INCLUDES += $(TPNE_INCLUDES)
VPATH += $(TPNE_VPATH)
LIB_FILES += $(TPNE_SOURCES)
USER_CXXFLAGS += $(TPNE_CXXFLAGS)
USER_CCFLAGS += $(TPNE_CFLAGS)

###############################################################################
# Kairos PDDL planner (from thirdparties, built by Makefile.common).
# The web build has no Kairos library: WebPlanner.cpp answers instead.
ifneq ($(OS),Emscripten)
# Link flags in Makefile.common: whoever links the core links Kairos too.
USER_CXXFLAGS += -isystem $(KAIROS_SRC)/include -isystem $(KAIROS_BUILD)/include
# Its generated headers (version, export) must exist before our objects.
pre-build:: $(KAIROS_SHARED)
endif

###############################################################################
# Sharable information between all Makefiles
#
include $(M)/rules/Makefile
# Emscripten is Clang. It ignores the Linux stack-clash flag, and the GCC-only
# suppressions (-Wno-noexcept from TimedPetriNet, -Wno-useless-cast, ...) are
# unknown options.
ifeq ($(OS),Emscripten)
CXXFLAGS := $(filter-out -fstack-clash-protection,$(CXXFLAGS))
USER_CXXFLAGS += -Wno-unknown-warning-option -Wno-unused-command-line-argument
endif
include $(P)/Makefile.relink

###############################################################################
# Build rapidyaml on demand (rule in Makefile.common): a tree where it is
# missing would otherwise fail late with "ar: libryml.a: No such file".
#
ifeq ($(OS),Emscripten)
$(TARGET_STATIC_LIB_NAME) $(TARGET_SHARED_LIB_NAME): $(BLACKTHORN_STATIC_DEPS)
else
$(TARGET_STATIC_LIB_NAME) $(TARGET_SHARED_LIB_NAME): $(BLACKTHORN_STATIC_DEPS) $(KAIROS_SHARED)

# The Kairos adapter includes headers its CMake generates: order-only, so a
# parallel build cannot compile it first.
$(filter %/Planning/Kairos/KairosPlanner.o,$(LIB_OBJS)): | $(KAIROS_SHARED)
endif
