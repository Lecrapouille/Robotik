// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyDraw.hpp
// @brief Poses the fly URDF and draws its eye beams in a Compages scene.
#pragma once

#include "FlyEnvironment.hpp"

#include "Compages/World/Entity.hpp"

#include <string_view>

namespace robotik
{
class Robot;
}

namespace compages::renderer
{
class Scene;
}

// ------------------------------------------------------------------------
//! @brief Z-up metres to the Compages view (Y up): (x, y, z) becomes
//! (x, z, -y).
// ------------------------------------------------------------------------
[[nodiscard]] compages::core::Vector3f flyToView(robotik::Vector3 const& p_z_up);

// ------------------------------------------------------------------------
//! @brief Writes the joint targets and the thorax pose.
//!
//! Does not step the world. The caller does, so a camera orbit is not
//! updated once per physics substep.
// ------------------------------------------------------------------------
void poseFlyBody(robotik::Robot& p_robot, FlySnapshot const& p_snapshot);

// ------------------------------------------------------------------------
//! @brief Aims one sensor beam from the eye to the sample.
//!
//! The Scene cone stands on Y (tip at +Y, base at -Y). The base sits on
//! the eye, the tip on the sample. The two spheres mark those ends.
//! @p_strength in [0, 1] thickens the beam and grows the tip.
// ------------------------------------------------------------------------
void aimFlyBeam(compages::world::Entity p_beam,
                compages::world::Entity p_start,
                compages::world::Entity p_end,
                robotik::Vector3 const& p_from,
                robotik::Vector3 const& p_to,
                float p_strength);

// ------------------------------------------------------------------------
//! @brief Parents a camera to an eye link, looking along that link's +X.
//!
//! URDF eye frames use +X as the optical axis and +Z as up. A Compages
//! camera looks down its local -Z, so the mount turns -Z onto +X. The eye
//! sits a few centimetres in front of the link so the head mesh stays
//! behind the near plane. An unknown link returns an empty handle.
// ------------------------------------------------------------------------
[[nodiscard]] compages::world::Entity
mountFlyEye(compages::renderer::Scene& p_scene,
            robotik::Robot& p_robot,
            std::string_view p_link,
            std::string_view p_name);
