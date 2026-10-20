// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Assets/AssetResolver.hpp"

#include "Robotik/Common/File.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace robotik
{

// ----------------------------------------------------------------------------
std::vector<std::filesystem::path> AssetResolver::standardRoots()
{
    std::vector<std::filesystem::path> roots;
    if (char const* env = std::getenv("ROBOTIK_DATA"); env != nullptr)
    {
        std::string_view paths(env);
        while (!paths.empty())
        {
            std::size_t const colon = paths.find(':');
            std::string_view const item = paths.substr(0, colon);
            if (!item.empty())
            {
                roots.emplace_back(item);
            }
            paths = colon == std::string_view::npos ? std::string_view{}
                                                    : paths.substr(colon + 1U);
        }
    }
    if (std::filesystem::path const exe = executableDirectory(); !exe.empty())
    {
        roots.push_back(exe.parent_path() / "data" / "packs");
        roots.push_back(exe.parent_path() / "share" / "robotik" / "packs");
    }
    std::erase_if(roots,
                  [](std::filesystem::path const& p_root)
                  { return !isDirectory(p_root); });
    return roots;
}

// ----------------------------------------------------------------------------
AssetResolver AssetResolver::standard()
{
    AssetResolver resolver;
    for (std::filesystem::path const& root : standardRoots())
    {
        resolver.addRoot(root);
    }
    return resolver;
}

// -----------------------------------------------------------------------------
void AssetResolver::addRoot(std::filesystem::path const& p_root)
{
    if (isRegularFile(p_root / "pack.yaml"))
    {
        addPack(p_root);
        return;
    }
    if (!isDirectory(p_root))
    {
        return;
    }
    std::vector<std::filesystem::path> children;
    std::error_code error;
    for (std::filesystem::directory_iterator cursor(p_root, error), end;
         !error && cursor != end;
         cursor.increment(error))
    {
        if (isRegularFile(cursor->path() / "pack.yaml"))
        {
            children.push_back(cursor->path());
        }
    }
    // Directory order is not portable: sort for a reproducible winner.
    std::ranges::sort(children);
    for (std::filesystem::path const& child : children)
    {
        addPack(child);
    }
}

// -----------------------------------------------------------------------------
void AssetResolver::addPack(std::filesystem::path const& p_root)
{
    Result<AssetPack> loaded = loadAssetPack(p_root / "pack.yaml");
    if (!loaded)
    {
        m_errors.push_back(loaded.error());
        return;
    }
    AssetPack pack = loaded.take();
    if (AssetPack const* known = find(pack.id))
    {
        if (known->root != pack.root && known->version != pack.version)
        {
            m_errors.push_back(
                "pack '" + pack.id + "' " + toString(pack.version) + " in " +
                pack.root.string() + " conflicts with " +
                toString(known->version) + " in " + known->root.string());
        }
        return;
    }
    m_packs.push_back(std::move(pack));
}

// -----------------------------------------------------------------------------
AssetPack const* AssetResolver::find(std::string_view p_id) const
{
    auto const found = std::ranges::find_if(
        m_packs, [p_id](AssetPack const& p_pack) { return p_pack.id == p_id; });
    return found == m_packs.end() ? nullptr : &*found;
}

// -----------------------------------------------------------------------------
AssetPack const* AssetResolver::require(std::string_view p_id,
                                        Version const& p_minimum,
                                        std::string& p_why) const
{
    AssetPack const* pack = find(p_id);
    if (pack == nullptr)
    {
        p_why = "pack '" + std::string(p_id) + "' is not installed";
        return nullptr;
    }
    if (!satisfies(pack->version, p_minimum))
    {
        p_why = "pack '" + std::string(p_id) + "' is " +
                toString(pack->version) + ", " + toString(p_minimum) +
                " compatible is required";
        return nullptr;
    }
    return pack;
}

// -----------------------------------------------------------------------------
Result<std::filesystem::path> AssetResolver::model(std::string_view p_id) const
{
    AssetPack const* best = nullptr;
    for (AssetPack const& pack : m_packs)
    {
        bool const prefix = p_id.size() > pack.id.size() + 1U &&
                            p_id.starts_with(pack.id) &&
                            p_id[pack.id.size()] == '.';
        if (prefix && (best == nullptr || pack.id.size() > best->id.size()))
        {
            best = &pack;
        }
    }
    if (best == nullptr)
    {
        return failure("No asset pack provides '" + std::string(p_id) + "'");
    }
    std::string const key(p_id.substr(best->id.size() + 1U));
    std::filesystem::path file = robotik::model(*best, key);
    if (file.empty())
    {
        return failure("Pack '" + best->id + "' has no model '" + key + "'");
    }
    return file;
}

// -----------------------------------------------------------------------------
Result<std::filesystem::path>
AssetResolver::resolve(std::string const& p_reference,
                       std::filesystem::path const& p_base) const
{
    if (p_reference.empty())
    {
        return std::filesystem::path{};
    }
    bool const path = p_reference.find('/') != std::string::npos ||
                      p_reference.ends_with(".urdf");
    if (path)
    {
        return std::filesystem::path((p_base / p_reference).lexically_normal());
    }
    return model(p_reference);
}

} // namespace robotik
