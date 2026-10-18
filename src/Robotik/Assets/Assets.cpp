// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Assets/AssetPack.hpp"
#include "Robotik/Assets/AssetResolver.hpp"
#include "Robotik/Assets/Version.hpp"

#include "BlackThorn/Builder/Yaml.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdlib>
#include <stdexcept>
#include <system_error>

namespace robotik
{

/// Reads one scalar child of a YAML node, or empty.
static std::string text(bt::YamlNode const& p_node, std::string_view p_key)
{
    bt::YamlNode const child = p_node.child(p_key);
    return child.valid() ? child.scalar() : std::string{};
}

/// True when the path is a directory.
static bool directory(std::filesystem::path const& p_path)
{
    std::error_code error;
    return std::filesystem::is_directory(p_path, error);
}

/// True when the path is a regular file.
static bool regularFile(std::filesystem::path const& p_path)
{
    std::error_code error;
    return std::filesystem::is_regular_file(p_path, error);
}

/// Canonical path, or the normalized absolute path on failure.
static std::filesystem::path canonicalPath(std::filesystem::path const& p_path)
{
    std::error_code error;
    std::filesystem::path result = std::filesystem::weakly_canonical(p_path, error);
    return error ? std::filesystem::absolute(p_path).lexically_normal() : result;
}

/// True when @p_file stays below @p_root once both are made canonical.
static bool inside(std::filesystem::path const& p_root, std::filesystem::path const& p_file)
{
    std::filesystem::path const relative =
        canonicalPath(p_file).lexically_relative(canonicalPath(p_root));
    if (relative.empty())
    {
        return false;
    }
    return std::none_of(relative.begin(),
                        relative.end(),
                        [](std::filesystem::path const& p_part) { return p_part == ".."; });
}

std::filesystem::path executableDirectory()
{
    std::error_code error;
    std::filesystem::path const exe = std::filesystem::read_symlink("/proc/self/exe", error);
    return error ? std::filesystem::path{} : exe.parent_path();
}

// ----------------------------------------------------------------------------
std::optional<Version> Version::parse(std::string_view p_text)
{
    Version version;
    std::array<std::uint32_t*, 3> const parts = { &version.major, &version.minor, &version.patch };
    std::size_t index = 0;
    char const* cursor = p_text.data();
    char const* const end = p_text.data() + p_text.size();
    while (true)
    {
        if (index == 3u || cursor == end)
        {
            return std::nullopt;
        }
        auto const [next, error] = std::from_chars(cursor, end, *parts[index]);
        if (error != std::errc{})
        {
            return std::nullopt;
        }
        cursor = next;
        ++index;
        if (cursor == end)
        {
            return version;
        }
        if (*cursor != '.')
        {
            return std::nullopt;
        }
        ++cursor;
    }
}

std::string Version::toString() const
{
    return std::to_string(major) + "." + std::to_string(minor) + "." +
           std::to_string(patch);
}

// ----------------------------------------------------------------------------
AssetPack AssetPack::load(std::filesystem::path const& p_manifest)
{
    auto parsed = bt::YamlDocument::parseFile(p_manifest.string());
    if (!parsed)
    {
        throw std::runtime_error(p_manifest.string() + ": " + parsed.getError());
    }
    bt::YamlNode const node = parsed.getValue().root();

    AssetPack pack;
    pack.root = canonicalPath(p_manifest.parent_path());
    pack.id = text(node, "id");
    pack.license = text(node, "license");
    pack.source = text(node, "source");
    pack.description = text(node, "description");
    std::optional<Version> const version = Version::parse(text(node, "version"));
    if (pack.id.empty() || !version || pack.license.empty())
    {
        throw std::runtime_error(p_manifest.string() +
                                 ": id, version and license are required");
    }
    pack.version = *version;

    if (node.hasKey("models"))
    {
        node.child("models").forEachMap(
            [&](std::string_view p_key, bt::YamlNode p_file)
            {
                std::filesystem::path const file = pack.root / p_file.scalar();
                if (!regularFile(file) || !inside(pack.root, file))
                {
                    throw std::runtime_error(p_manifest.string() + ": model '" +
                                             std::string(p_key) + "' is not a file of the pack");
                }
                pack.models.emplace(std::string(p_key), p_file.scalar());
            });
    }
    return pack;
}

std::filesystem::path AssetPack::model(std::string const& p_key) const
{
    auto const found = models.find(p_key);
    return found == models.end() ? std::filesystem::path{} : root / found->second;
}

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
                                                    : paths.substr(colon + 1u);
        }
    }
    if (std::filesystem::path const exe = executableDirectory(); !exe.empty())
    {
        roots.push_back(exe.parent_path() / "data" / "packs");
        roots.push_back(exe.parent_path() / "share" / "robotik" / "packs");
    }
    std::erase_if(roots, [](std::filesystem::path const& p_root) { return !directory(p_root); });
    return roots;
}

AssetResolver AssetResolver::standard()
{
    AssetResolver resolver;
    for (std::filesystem::path const& root : standardRoots())
    {
        resolver.addRoot(root);
    }
    return resolver;
}

void AssetResolver::addRoot(std::filesystem::path const& p_root)
{
    if (regularFile(p_root / "pack.yaml"))
    {
        addPack(p_root);
        return;
    }
    if (!directory(p_root))
    {
        return;
    }
    std::vector<std::filesystem::path> children;
    std::error_code error;
    for (std::filesystem::directory_iterator cursor(p_root, error), end;
         !error && cursor != end;
         cursor.increment(error))
    {
        if (regularFile(cursor->path() / "pack.yaml"))
        {
            children.push_back(cursor->path());
        }
    }
    // Directory order is not portable: sort for a reproducible winner.
    std::sort(children.begin(), children.end());
    for (std::filesystem::path const& child : children)
    {
        addPack(child);
    }
}

void AssetResolver::addPack(std::filesystem::path const& p_root)
{
    AssetPack pack;
    try
    {
        pack = AssetPack::load(p_root / "pack.yaml");
    }
    catch (std::exception const& error)
    {
        m_errors.emplace_back(error.what());
        return;
    }
    if (AssetPack const* known = find(pack.id))
    {
        if (known->root != pack.root && known->version != pack.version)
        {
            m_errors.push_back("pack '" + pack.id + "' " + pack.version.toString() +
                               " in " + pack.root.string() + " conflicts with " +
                               known->version.toString() + " in " + known->root.string());
        }
        return;
    }
    m_packs.push_back(std::move(pack));
}

AssetPack const* AssetResolver::find(std::string_view p_id) const
{
    auto const found = std::find_if(m_packs.begin(),
                                    m_packs.end(),
                                    [p_id](AssetPack const& p_pack) { return p_pack.id == p_id; });
    return found == m_packs.end() ? nullptr : &*found;
}

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
    if (!pack->version.satisfies(p_minimum))
    {
        p_why = "pack '" + std::string(p_id) + "' is " + pack->version.toString() +
                ", " + p_minimum.toString() + " compatible is required";
        return nullptr;
    }
    return pack;
}

std::filesystem::path AssetResolver::model(std::string_view p_id) const
{
    AssetPack const* best = nullptr;
    for (AssetPack const& pack : m_packs)
    {
        bool const prefix = p_id.size() > pack.id.size() + 1u &&
                            p_id.starts_with(pack.id) && p_id[pack.id.size()] == '.';
        if (prefix && (best == nullptr || pack.id.size() > best->id.size()))
        {
            best = &pack;
        }
    }
    if (best == nullptr)
    {
        throw std::runtime_error("No asset pack provides '" + std::string(p_id) + "'");
    }
    std::string const key(p_id.substr(best->id.size() + 1u));
    std::filesystem::path file = best->model(key);
    if (file.empty())
    {
        throw std::runtime_error("Pack '" + best->id + "' has no model '" + key + "'");
    }
    return file;
}

std::filesystem::path AssetResolver::resolve(std::string const& p_reference,
                                             std::filesystem::path const& p_base) const
{
    if (p_reference.empty())
    {
        return {};
    }
    bool const path = p_reference.find('/') != std::string::npos ||
                      p_reference.ends_with(".urdf");
    if (path)
    {
        return (p_base / p_reference).lexically_normal();
    }
    return model(p_reference);
}

} // namespace robotik
