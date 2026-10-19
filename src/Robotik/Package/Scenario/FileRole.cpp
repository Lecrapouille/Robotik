// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/FileRole.hpp"

#include <algorithm>
#include <array>
#include <string_view>
#include <utility>

namespace robotik
{

/// Each role, its main extension, then the spelling also accepted (.yml).
struct Spelling
{
    FileRole role;
    std::string_view extension;
};

static constexpr std::array<Spelling, 13> SPELLINGS{ {
    { FileRole::World, ".world.yaml" },
    { FileRole::Scenario, ".scenario.yaml" },
    { FileRole::Rl, ".rl.yaml" },
    { FileRole::Tree, ".bt.yaml" },
    { FileRole::Domain, ".domain.pddl" },
    { FileRole::Trajectory, ".trajectory.yaml" },
    { FileRole::Experiment, ".experiment.yaml" },
    { FileRole::World, ".world.yml" },
    { FileRole::Scenario, ".scenario.yml" },
    { FileRole::Rl, ".rl.yml" },
    { FileRole::Tree, ".bt.yml" },
    { FileRole::Trajectory, ".trajectory.yml" },
    { FileRole::Experiment, ".experiment.yml" },
} };

/// True when the file name ends with .yaml or .yml.
static bool yaml(std::string_view p_name)
{
    return p_name.ends_with(".yaml") || p_name.ends_with(".yml");
}

FileRole fileRole(std::filesystem::path const& p_file)
{
    std::string const name = p_file.filename().string();
    for (Spelling const& spelling : SPELLINGS)
    {
        if (name.ends_with(spelling.extension) &&
            name.size() > spelling.extension.size())
        {
            return spelling.role;
        }
    }
    return FileRole::Unknown;
}

char const* roleExtension(FileRole p_role)
{
    for (Spelling const& spelling : SPELLINGS)
    {
        if (spelling.role == p_role)
        {
            return spelling.extension.data();
        }
    }
    return "";
}

std::string roleStem(std::filesystem::path const& p_file)
{
    std::string name = p_file.filename().string();
    for (std::string_view const extension : { ".yaml", ".yml", ".pddl" })
    {
        if (name.ends_with(extension) && name.size() > extension.size())
        {
            name.resize(name.size() - extension.size());
            break;
        }
    }
    // A scenario is the default role: its id is the bare name.
    constexpr std::string_view SCENARIO = ".scenario";
    if (name.ends_with(SCENARIO) && name.size() > SCENARIO.size())
    {
        name.resize(name.size() - SCENARIO.size());
    }
    std::replace(name.begin(), name.end(), '.', '_');
    return name;
}

bool runnableRole(std::filesystem::path const& p_file)
{
    switch (fileRole(p_file))
    {
        case FileRole::Scenario:
        case FileRole::Rl:
        case FileRole::Experiment:
            return true;
        case FileRole::Unknown:
            return yaml(p_file.filename().string());
        case FileRole::World:
        case FileRole::Tree:
        case FileRole::Domain:
        case FileRole::Trajectory:
            break;
    }
    return false;
}

} // namespace robotik
