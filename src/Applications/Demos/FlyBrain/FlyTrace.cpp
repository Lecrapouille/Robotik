// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyTrace.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

//! @brief Nine observation fields, comma separated, declaration order.
void writeObservation(std::ostream& p_out, FlyObservation const& p_observation)
{
    p_out << p_observation.visual_left << ',' << p_observation.visual_center
          << ',' << p_observation.visual_right << ','
          << p_observation.angular_velocity << ','
          << p_observation.forward_velocity << ',' << p_observation.altitude
          << ',' << p_observation.distance_to_obstacle << ','
          << p_observation.target_bearing << ','
          << p_observation.target_distance;
}

//! @brief forward, turn, lift.
void writeAction(std::ostream& p_out, FlyAction const& p_action)
{
    p_out << p_action.forward << ',' << p_action.turn << ',' << p_action.lift;
}

//! @brief Splits a comma-separated list of floats. No brackets.
std::vector<float> numbers(std::string const& p_text)
{
    std::vector<float> values;
    std::stringstream stream(p_text);
    std::string item;
    while (std::getline(stream, item, ','))
    {
        values.push_back(std::stof(item));
    }
    return values;
}

} // namespace

void FlyTrace::save(std::filesystem::path const& p_path) const
{
    std::ofstream file(p_path);
    if (!file)
    {
        throw std::runtime_error("Cannot write '" + p_path.string() + "'");
    }
    file << std::setprecision(8);
    file << "{\n  \"seed\": " << seed << ",\n  \"dt\": " << dt
         << ",\n  \"steps\": [\n";
    std::size_t const count = std::min(observations.size(), actions.size());
    for (std::size_t i = 0; i < count; ++i)
    {
        file << "    {\"o\":[";
        writeObservation(file, observations[i]);
        file << "],\"a\":[";
        writeAction(file, actions[i]);
        file << "]}";
        if (i + 1u < count)
        {
            file << ',';
        }
        file << '\n';
    }
    file << "  ]\n}\n";
}

FlyTrace FlyTrace::load(std::filesystem::path const& p_path)
{
    std::ifstream file(p_path);
    if (!file)
    {
        throw std::runtime_error("Cannot read '" + p_path.string() + "'");
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string const text = buffer.str();
    FlyTrace trace;
    auto const seed_at = text.find("\"seed\"");
    auto const dt_at = text.find("\"dt\"");
    if (seed_at == std::string::npos || dt_at == std::string::npos)
    {
        throw std::runtime_error("'" + p_path.string() +
                                 "' is not a fly trace");
    }
    trace.seed = static_cast<std::uint64_t>(
        std::stoull(text.substr(text.find(':', seed_at) + 1)));
    trace.dt = std::stod(text.substr(text.find(':', dt_at) + 1));

    std::size_t cursor = 0;
    while ((cursor = text.find("\"a\":[", cursor)) != std::string::npos)
    {
        std::size_t const begin = cursor + 5;
        std::size_t const end = text.find(']', begin);
        if (end == std::string::npos)
        {
            break;
        }
        std::vector<float> const action =
            numbers(text.substr(begin, end - begin));
        if (action.size() >= FlyAction::SIZE)
        {
            trace.actions.push_back(FlyAction::from(action));
        }
        cursor = end + 1;
    }
    cursor = 0;
    while ((cursor = text.find("\"o\":[", cursor)) != std::string::npos)
    {
        std::size_t const begin = cursor + 5;
        std::size_t const end = text.find(']', begin);
        if (end == std::string::npos)
        {
            break;
        }
        std::vector<float> const observation =
            numbers(text.substr(begin, end - begin));
        if (observation.size() >= FlyObservation::SIZE)
        {
            trace.observations.push_back(FlyObservation::from(observation));
        }
        cursor = end + 1;
    }
    return trace;
}
