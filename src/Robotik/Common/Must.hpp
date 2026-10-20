// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Must.hpp
/// Private to the YAML readers of the library: stops a deep reading at the
/// first problem. The public reader of each file catches it and returns it as
/// its own Result, so no exception leaves the library.
#pragma once

#include "Robotik/Common/Result.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace robotik
{

/// Problem met deep inside a reader.
struct ReadError : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

// ----------------------------------------------------------------------------
/// @param p_result A Result met while reading.
/// @return Its value; stops the reading with its error otherwise.
// ----------------------------------------------------------------------------
template <typename T>
T must(Result<T> p_result)
{
    if (!p_result)
    {
        throw ReadError(p_result.error());
    }
    return p_result.take();
}

} // namespace robotik
