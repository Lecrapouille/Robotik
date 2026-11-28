// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Robot/Fidelity.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
char const* toString(PhysicsFidelity p_fidelity)
{
    switch (p_fidelity)
    {
        case PhysicsFidelity::Abstract:
            return "abstract";
        case PhysicsFidelity::Rigid:
            return "rigid";
        case PhysicsFidelity::Contact:
            return "contact";
    }
    return "?";
}

// ----------------------------------------------------------------------------
char const* toString(PerceptionFidelity p_fidelity)
{
    switch (p_fidelity)
    {
        case PerceptionFidelity::Sensed:
            return "sensed";
        case PerceptionFidelity::NoisyTruth:
            return "noisy_truth";
        case PerceptionFidelity::Truth:
            return "truth";
    }
    return "?";
}

} // namespace robotik
