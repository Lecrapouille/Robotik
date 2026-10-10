// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Runtime/Mission.hpp"

#include <string>
#include <string_view>
#include <vector>

// Kinematics behaviours shared by the manipulator scenarios. The YAML file
// selects which one runs; the assertions stay in that file.
class ManipulatorMission final: public robotik::Mission
{
public:

    enum class Mode
    {
        Forward,
        Inverse,
        Limits,
    };

    explicit ManipulatorMission(Mode p_mode) : m_mode(p_mode) {}

    void reset(robotik::Simulation& p_simulation, robotik::Seed p_seed) override;
    void step(robotik::Simulation& p_simulation, Seconds p_dt) override;
    void measure(robotik::Simulation const& p_simulation,
                 robotik::Metrics& p_metrics) const override;
    [[nodiscard]] robotik::Status status(robotik::Simulation const& p_simulation) const override;

    void requestHome()
    {
        m_home = true;
    }

    [[nodiscard]] std::string_view modeName() const;
    [[nodiscard]] double samples() const
    {
        return m_samples;
    }
    [[nodiscard]] double error() const
    {
        return m_error;
    }
    [[nodiscard]] bool reached() const
    {
        return m_reached > 0.0;
    }

private:

    Mode m_mode = Mode::Forward;
    bool m_home = false;
    bool m_have_target = false;
    robotik::Pose m_target{};
    std::vector<double> m_solution;
    double m_samples = 0.0;
    double m_error = 1.0;
    double m_inside = 0.0;
    double m_reached = 0.0;
    robotik::JointId m_joint = robotik::NO_JOINT;
};
