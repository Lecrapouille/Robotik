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

RECEPTION_DIR := $(abspath $(HERE))

# Soldier.glb of three.js: a Mixamo character with the clips Idle, Walk and
# Run. It is not in git: copied from Compages-data when it is there, else
# downloaded, and checked by its SHA256.
RECEPTION_SOLDIER := $(RECEPTION_DIR)/data/people/Soldier.glb
RECEPTION_SOLDIER_URL := https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/models/gltf/Soldier.glb
RECEPTION_SOLDIER_SHA256 := dfb230fc1f942f259dd00281a1186953ad602fc5d69067ce63e24b2aa439736b
RECEPTION_SOLDIER_LOCAL := $(firstword $(wildcard $(COMPAGES_DATA_PATH)/Soldier.glb \
    $(HOME)/MyGithub/Compages/external/Compages-data/Soldier.glb))

# The people (woman, man, child) are made with MakeHuman, by Blender and MPFB:
#   make reception-people
# It downloads the CC0 asset packs of MakeHuman (about 640 MB) once into
# RECEPTION_PACKS, installs them in MPFB, and writes data/people/<name>.glb
# with the clips of Soldier.glb. Without them, Soldier.glb stands in.
RECEPTION_PEOPLE := $(addprefix $(RECEPTION_DIR)/data/people/,woman.glb man.glb child.glb)
RECEPTION_PACKS ?= $(HOME)/.cache/robotik/mpfb-packs
RECEPTION_PACK_URLS := \
    functional/faceunits01.zip functional/visemes01.zip \
    asset_packs/makehuman_system_assets/makehuman_system_assets_cc0.zip \
    asset_packs/shirts01/shirts01_cc0.zip asset_packs/pants01/pants01_cc0.zip \
    asset_packs/shoes01/shoes01_cc0.zip asset_packs/dress01/dress01_cc0.zip \
    asset_packs/skins01/skins01_cc0.zip asset_packs/skins02/skins02_cc0.zip

$(robotik_reception._PACKAGE_STAMP): $(RECEPTION_SOLDIER) $(RECEPTION_PEOPLE)

$(RECEPTION_SOLDIER):
	@mkdir -p $(dir $@)
	$(if $(RECEPTION_SOLDIER_LOCAL),cp $(RECEPTION_SOLDIER_LOCAL) $@,curl -fsSL -o $@ $(RECEPTION_SOLDIER_URL))
	@echo "$(RECEPTION_SOLDIER_SHA256)  $@" | sha256sum -c --quiet - || (rm -f $@; false)

$(RECEPTION_PEOPLE): | $(RECEPTION_SOLDIER)
	@echo "*** $(notdir $@): Soldier.glb stands in (make reception-people makes it with MakeHuman)"
	@cp $(RECEPTION_SOLDIER) $@

.PHONY: reception-people
reception-people: $(RECEPTION_SOLDIER)
	@mkdir -p $(RECEPTION_PACKS)
	@for u in $(RECEPTION_PACK_URLS); do f=$(RECEPTION_PACKS)/$$(basename $$u); \
	    test -s $$f || curl -fsSL -o $$f https://files.makehumancommunity.org/$$u || exit 1; done
	blender -b --python $(RECEPTION_DIR)/people/make_people.py -- \
	    --people $(RECEPTION_DIR)/people/people.json --packs $(RECEPTION_PACKS) \
	    --clips $(RECEPTION_SOLDIER) --out $(RECEPTION_DIR)/data/people
	$(MAKE) robotik_reception
