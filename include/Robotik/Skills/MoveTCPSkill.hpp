//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
//! @file MoveTCPSkill.hpp
//! @brief Cartesian skill via Pinocchio IK and joint position tracking.
//=============================================================================

#pragma once

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Moves a frame (typically the tool link) toward a @ref Pose target.
//!
//! IK is solved once per goal change; joint commands follow the solved @c q
//! until every bound joint is within tolerance.
// ****************************************************************************
class MoveTCPSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    //! @brief Constructs with frame name and target pose.
    //! @param p_frame Pinocchio frame name (e.g. @c "link6").
    //! @param p_target Desired pose in the robot base frame.
    //! @param p_joint_tolerance Per-joint success threshold after IK.
    // -------------------------------------------------------------------------
    MoveTCPSkill(std::string p_frame,
                 Pose const& p_target,
                 double p_joint_tolerance = 1e-2);

    // -------------------------------------------------------------------------
    //! @brief Clears cached IK solution so the next tick replans.
    // -------------------------------------------------------------------------
    void reset() override;

    // -------------------------------------------------------------------------
    //! @brief Sets a new Cartesian goal; replans if pose changed.
    //! @param p_frame Frame to control.
    //! @param p_target Target pose.
    // -------------------------------------------------------------------------
    void setGoal(std::string p_frame, Pose const& p_target);

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    //!< Controlled Pinocchio frame.
    std::string m_frame;
    //!< Cartesian setpoint.
    Pose m_target;
    //!< Joint-space success threshold.
    double m_joint_tolerance;
    //!< IK solution applied to @ref ecs::JointCommand.
    std::vector<double> m_target_q;
    //!< True when @ref m_target_q is valid.
    bool m_has_target = false;
};

} // namespace robotik
