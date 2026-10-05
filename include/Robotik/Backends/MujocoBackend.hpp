// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file MujocoBackend.hpp
//! @brief MuJoCo dynamics behind a @ref Robot.
#pragma once

#include "Robotik/Robot/Robot.hpp"

#include <filesystem>
#include <memory>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Rigid-body dynamics with contacts, built on MuJoCo.
//!
//! Each @ref step runs the joint PD loops (@ref JointSet::control) at the
//! physics rate (1 kHz), adds gravity compensation, integrates, and writes the
//! measured joints back. URDF files missing inertial data are copied to a
//! private temporary file with defaults, so Pinocchio and Compages still see
//! the original. Instances are independent: one per parallel environment.
//!
//! @code
//! robotik::RobotSession robot(world, "arm.urdf");
//! robot.connect(std::make_unique<robotik::MujocoBackend>("arm.urdf"));
//! @endcode
// ****************************************************************************
class MujocoBackend final: public RobotBackend
{
public:

    //! @throws std::runtime_error if MuJoCo cannot parse the file.
    explicit MujocoBackend(std::filesystem::path const& p_urdf);
    ~MujocoBackend() override;

    void attach(Robot& p_robot) override;
    void reset(Robot& p_robot) override;
    void step(Robot& p_robot, Seconds p_dt) override;
    [[nodiscard]] int contacts() const override;

    //! @brief Physics time step (default 1 ms).
    void timestep(Seconds p_dt)
    {
        m_timestep = p_dt;
    }

private:

    struct Impl;
    std::unique_ptr<Impl> m_impl;

    struct Binding
    {
        int qpos = -1;
        int dof = -1;
        int actuator = -1;
    };

    //!< Indexed by @ref JointId.
    std::vector<Binding> m_bindings;
    Seconds m_timestep{ 0.001 };
};

} // namespace robotik
