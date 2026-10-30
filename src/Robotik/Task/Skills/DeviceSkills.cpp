// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Skills/DeviceSkills.hpp"

namespace robotik
{

Status RunBeltSkill::tick(RobotContext& /*p_context*/, Seconds /*p_dt*/)
{
    m_belt.run(true);
    return Status::RUNNING;
}

void RunBeltSkill::cancel(RobotContext& /*p_context*/)
{
    m_belt.stop();
}

} // namespace robotik
