# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
#
# The plugin HumanoidWalk: a LimX humanoid walking by a learned policy
# (built only with PLUGINS+=HumanoidWalk). Paths are relative to this
# directory.

$(eval $(ROBOTIK_ONNXRUNTIME))
TARGETS += humanoid-description humanoid-rl-deploy-cpp robotik_humanoid_walk

### LimX HU_D04 humanoid: URDF and meshes. License: Apache-2.0
humanoid-description.GIT := https://github.com/limxdynamics/humanoid-description.git
### LimX controllers: the trained walking policy. License: Apache-2.0
humanoid-rl-deploy-cpp.GIT := https://github.com/limxdynamics/humanoid-rl-deploy-cpp.git

robotik_humanoid_walk.DESCRIPTION := Humanoid walking by a learned policy
robotik_humanoid_walk.SOURCES := src/*.cpp
robotik_humanoid_walk.DEPENDS := onnxruntime
$(eval $(call ROBOTIK_PLUGIN,robotik_humanoid_walk,HumanoidWalk))

# The body and the policy, made from the two repositories of LimX.
HU_D04_DIR := $(HERE)
robotik_humanoid_walk: $(abspath $(HERE)data/hu_d04)/walk.onnx
$(abspath $(HERE)data/hu_d04)/walk.onnx: | humanoid-description humanoid-rl-deploy-cpp
	python3 -I $(HU_D04_DIR)prepare_assets.py $(humanoid-description.DIR) $(humanoid-rl-deploy-cpp.DIR) $(HU_D04_DIR)data
