// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// The web build has no Kairos. A goal is reported as a planner that is not
/// there, instead of failing to link a native library.

#include "Robotik/Task/Planning/PlanningEngine.hpp"

#include <memory>

namespace robotik
{

namespace
{

class WebPlanner final: public PlanningEngine
{
public:

    std::string load(std::string const&, std::string const&) override
    {
        return "The PDDL planner is not available in the web build";
    }

    PlanningResult solve(std::string const&, PlanningOptions const&) override
    {
        PlanningResult result;
        result.status = PlanningResult::Status::Invalid;
        result.diagnostics = "The PDDL planner is not available in the web build";
        return result;
    }
};

} // namespace

char const* toString(PlanningResult::Status p_status)
{
    switch (p_status)
    {
        case PlanningResult::Status::Solved:
            return "solved";
        case PlanningResult::Status::Unsolvable:
            return "unsolvable";
        case PlanningResult::Status::NoPlanFound:
            return "no plan found";
        case PlanningResult::Status::Invalid:
            return "invalid";
    }
    return "?";
}

std::unique_ptr<PlanningEngine> makeKairosPlanner()
{
    return std::make_unique<WebPlanner>();
}

} // namespace robotik
