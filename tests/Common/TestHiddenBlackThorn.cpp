// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The compiler checks that the public API runs a cell, a simulation and a
// plan without bringing BlackThorn: only Skills/SkillNodes.hpp may include
// it. Do not include a header here that includes SkillNodes.hpp.

#include "main.hpp"

#include "Robotik/Package/Plugin/PluginAPI.hpp"
#include "Robotik/Package/RunReport.hpp"
#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/Planning/PlanExecutor.hpp"
#include "Robotik/Task/RoboticCell.hpp"

/// True when the type is complete (its definition was included).
template <typename T>
concept Complete = requires { sizeof(T); };

static_assert(!Complete<bt::Tree>,
              "a public header includes BlackThorn: name the tree types with "
              "Robotik/Task/TreeTypes.hpp instead");
static_assert(!Complete<bt::NodeFactory>,
              "a public header includes BlackThorn: name the tree types with "
              "Robotik/Task/TreeTypes.hpp instead");

TEST(HiddenBlackThorn, RoboticCellAnswersWithTheRobotikStatus)
{
    static_assert(
        std::is_same_v<
            decltype(std::declval<robotik::RoboticCell const&>().status()),
            robotik::Status>);
    static_assert(std::is_same_v<decltype(std::declval<robotik::PlanExecutor&>()
                                              .tick(std::declval<robotik::RobotContext const&>())),
                                 robotik::Status>);
    SUCCEED();
}
