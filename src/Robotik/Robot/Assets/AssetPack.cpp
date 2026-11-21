// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Assets/AssetPack.hpp"
#include "Robotik/Common/Config.hpp"
#include "Robotik/Common/File.hpp"

namespace robotik
{

// -------------------------------------------------------------------------
bool licenseVerified(AssetPack const& p_pack)
{
    return !p_pack.license.empty() && p_pack.license != "NOASSERTION";
}

// -----------------------------------------------------------------------------
Result<AssetPack> loadAssetPack(std::filesystem::path const& p_manifest)
{
    ConfigNode node;
    ROBOTIK_TRY_ASSIGN(node, ConfigNode::load(p_manifest));
    std::string const where = p_manifest.string();
    ROBOTIK_TRY(node.checkKeys(
        { "id", "version", "license", "source", "description", "models" },
        where));

    AssetPack pack;
    pack.root = canonicalPath(p_manifest.parent_path());
    pack.id = node.text("id");
    pack.license = node.text("license");
    pack.source = node.text("source");
    pack.description = node.text("description");
    std::optional<Version> const version = parseVersion(node.text("version"));
    if (pack.id.empty() || !version || pack.license.empty())
    {
        return failure(where + ": id, version and license are required");
    }
    pack.version = *version;

    for (auto const& [key, file_node] : node.at("models").entries())
    {
        std::filesystem::path const file = pack.root / file_node.scalar();
        if (!isRegularFile(file) || !pathInsideRoot(pack.root, file))
        {
            return failure(where + ": model '" + key +
                           "' is not a file of the pack");
        }
        pack.models.emplace(key, file_node.scalar());
    }
    return pack;
}

// ----------------------------------------------------------------------------
std::filesystem::path model(AssetPack const& p_pack, std::string const& p_key)
{
    auto const found = p_pack.models.find(p_key);
    return found == p_pack.models.end() ? std::filesystem::path{}
                                        : p_pack.root / found->second;
}

} // namespace robotik
