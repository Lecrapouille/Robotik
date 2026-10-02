//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
//! @file SkillNodes.hpp
//! @brief Registers @ref robot skill instances as BlackThorn action nodes and
//! traces runs.
//=============================================================================

#pragma once

#include "Robotik/Runtime/Status.hpp"

#include "BlackThorn/BlackThorn.hpp"
#include "Compages/Core/Units.hpp"

#include <memory>
#include <string>
#include <vector>

namespace robotik
{

class Skill;
struct RobotContext;

// ****************************************************************************
//! @brief Lightweight skill-run history for one simulation.
//!
//! One @ref Entry per BT action *run* (not one row per simulation tick). While
//! a skill keeps returning @ref Status::RUNNING, the same row is updated in
//! place—mainly @ref Entry::end so duration grows on the timeline. A new row is
//! appended only when the action starts again after success or failure.
//!
//! Answers operator and developer questions that raw BT ticks do not: which
//! named actions actually ran, for how long, and whether each ended in success
//! or failure. @ref Simulation::trace exposes the buffer to the simulator
//! timeline and to headless prints. This is not a general-purpose logger—no
//! file I/O, no wall-clock timestamps.
// ****************************************************************************
struct SkillTrace
{
    // ------------------------------------------------------------------------
    //! @brief One invocation of a registered BT action.
    // ------------------------------------------------------------------------
    struct Entry
    {
        //!< BlackThorn action name passed to @ref registerSkill.
        std::string name;
        //!< Last status reported by the skill for this run.
        Status status = Status::RUNNING;
        //!< Run start instant on the simulation clock.
        Seconds start{};
        //!< Instant of the last tick of this run.
        Seconds end{};
    };

    //!< Chronological list of skill runs.
    std::vector<Entry> entries;
};

// ----------------------------------------------------------------------------
//! @brief Binds a @ref Skill to a YAML behavior-tree action name.
//!
//! Call this while building @c bt::NodeFactory before @c Builder loads the
//! scenario tree. For each @p_name (e.g. @c "Home" in @c Action: name: Home),
//! BlackThorn will instantiate a @c CallbackLeaf whose tick function runs
//! @ref Skill::tick on @p_skill and returns @c RUNNING / @c SUCCESS / @c
//! FAILURE. Robotik does not subclass @c bt::Node; the lambda passed to
//! @c NodeFactory::registerAction *is* the BT action body (delegates to
//! @c tickSkillAction in @c SkillNodes.cpp).
//!
//! @p_context and @p_trace must outlive the factory and the loaded tree: the
//! stored callback captures them by reference. @p_skill is shared so the same
//! C++ object serves every tick of that action.
//!
//! @param p_factory Factory passed to @c bt::Builder when loading the BT YAML.
//! @param p_name Action name referenced in the behavior tree file.
//! @param p_skill Skill executed when the tree ticks this action.
//! @param p_context World, backends, and timing for each @ref Skill::tick.
//! @param p_trace Run log updated by the registered callback (see @ref
//! SkillTrace).
// ----------------------------------------------------------------------------
void registerSkill(bt::NodeFactory& p_factory,
                   std::string const& p_name,
                   std::shared_ptr<Skill> p_skill,
                   RobotContext& p_context,
                   SkillTrace& p_trace);

} // namespace robotik
