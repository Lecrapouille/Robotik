// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file GrafcetController.hpp
/// A GRAFCET driving the skills of a cell, as a behavior tree or a plan would.
///
/// A GRAFCET (IEC 60848, see doc/Glossary.md) is a chart of steps and
/// transitions, the language of the automation engineers: a step does
/// actions, a transition fires when its condition (receptivity) is true. It
/// is drawn with TimedPetriNetEditor and saved as @c x.grafcet.json. A GEMMA
/// (start and stop modes of a machine) is a GRAFCET of three graphs: safety,
/// control, production.
///
/// @code
/// step X1   action Approach(red_cube)        (N: requested while X1 is active)
///   |  done(Approach(red_cube))              (the skill succeeded)
/// step X2   action Grasp(red_cube)
///   |  holding(red_cube) + 3s/X2             (observer, or 3 s in X2)
/// @endcode
///
/// @par Contract
/// - Promises: an action named like a skill requests the skill when it
///   becomes true, and cancels it when it becomes false while the skill
///   waits or runs. Receptivities read the beliefs only: skill states
///   (@c done(S), @c failed(S), @c running(S)), predicates computed by the
///   observers of the @ref PlanningDomain, and operator inputs (buttons).
///   @c done(S) and @c failed(S) of a skill that is an action only tell
///   about the run requested by the GRAFCET.
/// - Expects: the scheduler and the planning domain outlive the controller.
/// - Errors: @ref load returns a failure for a file that cannot be read, is
///   not a GRAFCET, has a syntax error, an action that is not a skill, or a
///   @c done(S) on an unknown skill.
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Common/Result.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tpne
{
class GrafcetEngine;
class Net;
} // namespace tpne

namespace robotik
{

class SkillScheduler;
class PlanningDomain;
struct RobotContext;
struct GrafcetSpec;

// ****************************************************************************
/// A transition that fired, and when (see @ref GrafcetController::history).
// ****************************************************************************
struct GrafcetFiring
{
    /// Simulated time of the cycle it fired in.
    Seconds time{};
    /// Name of its graph.
    std::string graph;
    /// Transition id in its graph.
    std::size_t transition = 0U;
};

// ****************************************************************************
/// Runs a GRAFCET file on the skills of a cell (see the file comment).
///
/// @code
/// auto loaded = robotik::GrafcetController::load(spec, scheduler, domain);
/// if (!loaded)
///     return robotik::failure(loaded.error());
/// std::unique_ptr<robotik::GrafcetController> grafcet = loaded.take();
/// grafcet->setInput("Dcy", 1);            // the operator presses "cycle"
/// bt::Status status = grafcet->tick(context);  // once per step, before
///                                              // the scheduler update
/// @endcode
// ****************************************************************************
class GrafcetController
{
public:

    // -------------------------------------------------------------------------
    /// Reads the GRAFCET file and checks its names against the skills and
    /// the observers.
    /// @param p_spec File, end conditions and initial inputs.
    /// @param p_skills Skills the actions request.
    /// @param p_domain Observers the receptivities may read.
    /// @return The controller, or what is wrong in the file.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<std::unique_ptr<GrafcetController>>
    load(GrafcetSpec const& p_spec, SkillScheduler& p_skills,
         PlanningDomain const& p_domain);

    ~GrafcetController();
    GrafcetController(GrafcetController const&) = delete;
    GrafcetController& operator=(GrafcetController const&) = delete;

    // -------------------------------------------------------------------------
    /// Back to the initial situation and the initial inputs.
    // -------------------------------------------------------------------------
    void reset();

    // -------------------------------------------------------------------------
    /// One cycle: reads the beliefs, evolves the graphs, requests or cancels
    /// the skills of the actions.
    /// @param p_context Robot API and beliefs (time step: @c p_context.dt).
    /// @return SUCCESS or FAILURE when the end conditions of the scenario
    ///   hold, RUNNING otherwise.
    // -------------------------------------------------------------------------
    [[nodiscard]] bt::Status tick(RobotContext const& p_context);

    // -------------------------------------------------------------------------
    /// Sets an operator input (a GEMMA button: @c Dcy, @c AU ...).
    /// @param p_name Input name, as in the receptivities.
    /// @param p_value New value (0 or 1 for a button).
    /// @return False if the name is not an operator input.
    // -------------------------------------------------------------------------
    bool setInput(std::string const& p_name, int p_value);

    // -------------------------------------------------------------------------
    /// Presses a push button: the operator input is 1 during the next cycle
    /// only, then 0 again (a rising then a falling edge), even if the
    /// simulation is paused until then. Use it for Dcy, Rearm...; use
    /// @ref setInput for a latched button (AU).
    /// @param p_name Input name.
    /// @return False if the name is not an operator input.
    // -------------------------------------------------------------------------
    bool pulse(std::string const& p_name);

    // -------------------------------------------------------------------------
    /// @return Operator inputs and their values (the other inputs are
    ///   computed from the beliefs).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::map<std::string, int> operatorInputs() const;

    // -------------------------------------------------------------------------
    /// @return Active steps as "Graph:Xn", in graph order.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::string> activeSteps() const;

    // -------------------------------------------------------------------------
    /// @return Transitions fired since the start of the run.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t firings() const;

    // -------------------------------------------------------------------------
    /// @return Graphs of the file, with their current marking. For viewers
    ///   that include TimedPetriNetEditor headers (the simulator).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<tpne::Net> const& graphs() const;

    // -------------------------------------------------------------------------
    /// @return The engine (inputs, outputs, action states) for viewers.
    // -------------------------------------------------------------------------
    [[nodiscard]] tpne::GrafcetEngine const& engine() const;

    // -------------------------------------------------------------------------
    /// @return True when the graphs were generated from the workflow of the
    ///   cell (@ref GrafcetSpec::workflow), not read from a file.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool generated() const;

    // -------------------------------------------------------------------------
    /// @return Every transition fired since the last reset, in order, with
    ///   its time (to measure how long the operations of a cell take).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<GrafcetFiring> const& history() const;

    // -------------------------------------------------------------------------
    /// Saves the graphs in their initial situation, as a GRAFCET file that
    /// TimedPetriNetEditor opens (to look at a generated GRAFCET, to replay
    /// it or to change it by hand).
    /// @param p_file A @c x.grafcet.json file.
    /// @return What went wrong while writing.
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<void> save(std::filesystem::path const& p_file) const;

    /// Implementation (hides TimedPetriNetEditor from the users).
    struct Impl;

private:

    GrafcetController();

    std::unique_ptr<Impl> m_impl;
};

} // namespace robotik
