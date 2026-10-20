// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Assets/AssetPack.hpp"
#include "Robotik/Common/File.hpp"
#include "BlackThorn/Builder/Yaml.hpp"


namespace robotik
{

// ---------------------------------------------------------------------------
/// Reads one scalar child of a YAML node, or empty.
/// @param p_node Node to inspect.
/// @param p_key Key to read.
/// @return The scalar value, or empty when the key is not found.
// ---------------------------------------------------------------------------
static std::string text(bt::YamlNode const& p_node, std::string_view p_key)
{
    bt::YamlNode const child = p_node.child(p_key);
    return child.valid() ? child.scalar() : std::string{};
}

// -------------------------------------------------------------------------
bool licenseVerified(AssetPack const& p_pack)
{
    return !p_pack.license.empty() && p_pack.license != "NOASSERTION";
}

// -----------------------------------------------------------------------------
Result<AssetPack> loadAssetPack(std::filesystem::path const& p_manifest)
{
    auto parsed = bt::YamlDocument::parseFile(p_manifest.string());
    if (!parsed)
    {
        return failure(p_manifest.string() + ": " + parsed.getError());
    }
    bt::YamlNode const node = parsed.getValue().root();

    AssetPack pack;
    pack.root = canonicalPath(p_manifest.parent_path());
    pack.id = text(node, "id");
    pack.license = text(node, "license");
    pack.source = text(node, "source");
    pack.description = text(node, "description");
    std::optional<Version> const version = parseVersion(text(node, "version"));
    if (pack.id.empty() || !version || pack.license.empty())
    {
        return failure(p_manifest.string() + ": id, version and license are required");
    }
    pack.version = *version;

    std::string error;
    if (node.hasKey("models"))
    {
        node.child("models").forEachMap(
            [&](std::string_view p_key, bt::YamlNode p_file)
            {
                std::filesystem::path const file = pack.root / p_file.scalar();
                if (!isRegularFile(file) || !pathInsideRoot(pack.root, file))
                {
                    if (error.empty())
                    {
                        error = p_manifest.string() + ": model '" + std::string(p_key) +
                                "' is not a file of the pack";
                    }
                    return;
                }
                pack.models.emplace(std::string(p_key), p_file.scalar());
            });
    }
    if (!error.empty())
    {
        return failure(error);
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
