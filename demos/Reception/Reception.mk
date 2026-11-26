# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin Reception: people walk, queue and talk in the hall of a
# reception robot, as its story says. Paths are relative to this directory.

TARGETS += robotik_reception
robotik_reception.DESCRIPTION := People played by a story in the hall of a reception robot
robotik_reception.SOURCES := src/*.cpp
$(eval $(call ROBOTIK_PLUGIN,robotik_reception,Reception))

# The people: Soldier.glb of three.js (a Mixamo character with Idle, Walk and
# Run). It is not in git: copied from Compages-data when it is there, else
# downloaded, and checked by its SHA256.
RECEPTION_SOLDIER := $(abspath $(HERE)data/people/Soldier.glb)
RECEPTION_SOLDIER_URL := https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/models/gltf/Soldier.glb
RECEPTION_SOLDIER_SHA256 := dfb230fc1f942f259dd00281a1186953ad602fc5d69067ce63e24b2aa439736b
RECEPTION_SOLDIER_LOCAL := $(firstword $(wildcard $(COMPAGES_DATA_PATH)/Soldier.glb \
    $(HOME)/MyGithub/Compages/external/Compages-data/Soldier.glb))
$(robotik_reception._PACKAGE_STAMP): $(RECEPTION_SOLDIER)
$(RECEPTION_SOLDIER):
	@mkdir -p $(dir $@)
	$(if $(RECEPTION_SOLDIER_LOCAL),cp $(RECEPTION_SOLDIER_LOCAL) $@,curl -fsSL -o $@ $(RECEPTION_SOLDIER_URL))
	@echo "$(RECEPTION_SOLDIER_SHA256)  $@" | sha256sum -c --quiet - || (rm -f $@; false)
