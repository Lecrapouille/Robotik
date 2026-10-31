# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin MicroDuck: a microduck walking by a learned policy (built only
# with PLUGINS+=MicroDuck). Paths are relative to this directory.

$(eval $(ROBOTIK_ONNXRUNTIME))
TARGETS += microduck_rl robotik_microduck

### Microduck body and scenes (meshes CC BY-NC-SA 4.0). License: Apache-2.0
microduck_rl.GIT := https://github.com/pollen-robotics/microduck_rl.git
microduck_rl.VERSION := develop

robotik_microduck.DESCRIPTION := Microduck walking by a learned policy
robotik_microduck.SOURCES := src/*.cpp
robotik_microduck.DEPENDS := onnxruntime
$(eval $(call ROBOTIK_PLUGIN,robotik_microduck,MicroDuck))

# The body, the scenes and the policy, made from microduck_rl.
DUCK_DIR := $(HERE)
robotik_microduck: $(abspath $(HERE)data/microduck)/walk.onnx
$(abspath $(HERE)data/microduck)/walk.onnx: $(HERE)prepare_assets.py | microduck_rl
	python3 -I $(DUCK_DIR)prepare_assets.py $(microduck_rl.DIR) $(DUCK_DIR)
