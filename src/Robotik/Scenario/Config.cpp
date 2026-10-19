// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/Config.hpp"

#include "BlackThorn/Builder/Yaml.hpp"

#include <algorithm>
#include <charconv>
#include <stdexcept>

namespace robotik
{

/// Copies a YAML node into the config tree the scenario keeps.
static ConfigNode copy(bt::YamlNode const& p_node)
{
    ConfigNode result;
    if (!p_node.valid())
    {
        return result;
    }
    if (p_node.isMap())
    {
        // An empty map stays a null node: both mean "nothing declared".
        p_node.forEachMap(
            [&result](std::string_view p_key, bt::YamlNode p_value)
            { result.put(std::string(p_key), copy(p_value)); });
        return result;
    }
    if (p_node.isSeq())
    {
        p_node.forEachSeq([&result](bt::YamlNode p_item)
                          { result.push(copy(p_item)); });
        return result;
    }
    // A YAML null ("~" or nothing) stays a null node.
    std::string text = p_node.scalar();
    if ((text.empty() || text == "~" || text == "null") &&
        !p_node.isQuotedScalar())
    {
        return result;
    }
    return ConfigNode(std::move(text));
}

/// Empty node returned when a queried key is missing.
static ConfigNode const& nullNode()
{
    static ConfigNode const empty;
    return empty;
}

/// Parses a sequence index, or nothing when the text is not one.
static std::optional<std::size_t> index(std::string_view p_text)
{
    std::size_t value = 0;
    auto const [end, error] =
        std::from_chars(p_text.data(), p_text.data() + p_text.size(), value);
    if (error != std::errc{} || end != p_text.data() + p_text.size())
    {
        return std::nullopt;
    }
    return value;
}

ConfigNode ConfigNode::load(std::filesystem::path const& p_path)
{
    auto parsed = bt::YamlDocument::parseFile(p_path.string());
    if (!parsed)
    {
        throw std::runtime_error("Cannot read '" + p_path.string() +
                                 "': " + parsed.getError());
    }
    return copy(parsed.getValue().root());
}

ConfigNode ConfigNode::parse(std::string const& p_text)
{
    auto parsed = bt::YamlDocument::parseText(p_text);
    if (!parsed)
    {
        throw std::runtime_error("Invalid YAML: " + parsed.getError());
    }
    return copy(parsed.getValue().root());
}

bool ConfigNode::has(std::string_view p_key) const
{
    return !at(p_key).isNull();
}

ConfigNode const& ConfigNode::at(std::string_view p_key) const
{
    for (auto const& [key, value] : m_entries)
    {
        if (key == p_key)
        {
            return value;
        }
    }
    return nullNode();
}

ConfigNode const& ConfigNode::item(std::string_view p_id) const
{
    for (ConfigNode const& item : m_items)
    {
        if (item.isMap() && item.at("id").scalar() == p_id)
        {
            return item;
        }
    }
    if (std::optional<std::size_t> const position = index(p_id);
        position && *position < m_items.size())
    {
        return m_items[*position];
    }
    return nullNode();
}

std::string ConfigNode::text(std::string_view p_key,
                             std::string_view p_default) const
{
    ConfigNode const& value = at(p_key);
    return value.isScalar() ? value.scalar() : std::string(p_default);
}

std::optional<double> ConfigNode::asNumber() const
{
    if (!isScalar())
    {
        return std::nullopt;
    }
    double value = 0.0;
    char const* const begin = m_scalar.data();
    char const* const end = begin + m_scalar.size();
    char const* const start =
        (begin != end && *begin == '+') ? begin + 1 : begin;
    auto const [last, error] = std::from_chars(start, end, value);
    if (error != std::errc{} || last != end)
    {
        return std::nullopt;
    }
    return value;
}

double ConfigNode::number(std::string_view p_key, double p_default) const
{
    ConfigNode const& value = at(p_key);
    if (value.isNull())
    {
        return p_default;
    }
    std::optional<double> const parsed = value.asNumber();
    if (!parsed)
    {
        throw std::runtime_error("'" + std::string(p_key) +
                                 "' is not a number: '" + value.scalar() + "'");
    }
    return *parsed;
}

bool ConfigNode::flag(std::string_view p_key, bool p_default) const
{
    ConfigNode const& value = at(p_key);
    if (value.isNull())
    {
        return p_default;
    }
    std::string const& text = value.scalar();
    if (text == "true" || text == "yes" || text == "on" || text == "1")
    {
        return true;
    }
    if (text == "false" || text == "no" || text == "off" || text == "0")
    {
        return false;
    }
    throw std::runtime_error("'" + std::string(p_key) +
                             "' is not a boolean: '" + text + "'");
}

Vector3 ConfigNode::vector3(std::string_view p_key, Vector3 p_default) const
{
    ConfigNode const& value = at(p_key);
    if (value.isNull())
    {
        return p_default;
    }
    std::optional<double> x;
    std::optional<double> y;
    std::optional<double> z;
    if (value.isSequence() && value.items().size() == 3u)
    {
        x = value.items()[0].asNumber();
        y = value.items()[1].asNumber();
        z = value.items()[2].asNumber();
    }
    if (!x || !y || !z)
    {
        throw std::runtime_error("'" + std::string(p_key) +
                                 "' must be [x, y, z]");
    }
    return { *x, *y, *z };
}

std::array<double, 2> ConfigNode::range(std::string_view p_key) const
{
    ConfigNode const& value = at(p_key);
    if (value.isNull())
    {
        return { 0.0, 0.0 };
    }
    std::optional<double> low;
    std::optional<double> high;
    if (value.isSequence() && value.items().size() == 2u)
    {
        low = value.items()[0].asNumber();
        high = value.items()[1].asNumber();
    }
    if (!low || !high || *high < *low)
    {
        throw std::runtime_error("'" + std::string(p_key) +
                                 "' must be [min, max]");
    }
    return { *low, *high };
}

std::vector<std::string> ConfigNode::strings(std::string_view p_key) const
{
    ConfigNode const& value = at(p_key);
    std::vector<std::string> result;
    if (value.isSequence())
    {
        for (ConfigNode const& item : value.items())
        {
            result.push_back(item.scalar());
        }
    }
    else if (value.isScalar())
    {
        result.push_back(value.scalar());
    }
    return result;
}

Pose ConfigNode::pose(std::string_view p_key) const
{
    ConfigNode const& value = at(p_key);
    if (!value.isNull() && !value.isMap())
    {
        throw std::runtime_error("'" + std::string(p_key) +
                                 "' must be { position, rpy | quaternion }");
    }
    return value.asPose();
}

Pose ConfigNode::asPose() const
{
    Pose pose{ Vector3::ZERO, Quaternion{} };
    if (!isMap())
    {
        return pose;
    }
    pose.position = vector3("position", Vector3::ZERO);
    ConfigNode const& quaternion = at("quaternion");
    if (!quaternion.isNull())
    {
        std::array<std::optional<double>, 4> q;
        if (quaternion.isSequence() && quaternion.items().size() == 4u)
        {
            for (std::size_t i = 0; i < 4u; ++i)
            {
                q[i] = quaternion.items()[i].asNumber();
            }
        }
        if (std::any_of(
                q.begin(), q.end(), [](auto const& p_v) { return !p_v; }))
        {
            throw std::runtime_error("'quaternion' must be [w, x, y, z]");
        }
        pose.rotation = Quaternion(*q[0], *q[1], *q[2], *q[3]).normalized();
        return pose;
    }
    Vector3 const angles = vector3("rpy", Vector3::ZERO);
    pose.rotation =
        Quaternion::fromRpy(Radians(angles.x), Radians(angles.y), Radians(angles.z));
    return pose;
}

ConfigNode* ConfigNode::child(std::string_view p_segment)
{
    if (m_kind == Kind::Map || m_kind == Kind::Null)
    {
        for (auto& [key, value] : m_entries)
        {
            if (key == p_segment)
            {
                return &value;
            }
        }
        put(std::string(p_segment), ConfigNode());
        return &m_entries.back().second;
    }
    if (m_kind == Kind::Sequence)
    {
        for (ConfigNode& item : m_items)
        {
            if (item.isMap() && item.at("id").scalar() == p_segment)
            {
                return &item;
            }
        }
        if (std::optional<std::size_t> const position = index(p_segment);
            position && *position < m_items.size())
        {
            return &m_items[*position];
        }
    }
    return nullptr;
}

void ConfigNode::set(std::string_view p_path, ConfigNode p_value)
{
    ConfigNode* node = this;
    std::string_view rest = p_path;
    while (!rest.empty())
    {
        std::size_t const dot = rest.find('.');
        std::string_view const segment = rest.substr(0, dot);
        ConfigNode* const next = node->child(segment);
        if (next == nullptr)
        {
            throw std::runtime_error("Override '" + std::string(p_path) +
                                     "': no '" + std::string(segment) +
                                     "' here");
        }
        node = next;
        rest = dot == std::string_view::npos ? std::string_view{}
                                             : rest.substr(dot + 1u);
    }
    *node = std::move(p_value);
}

void ConfigNode::push(ConfigNode p_value)
{
    m_kind = Kind::Sequence;
    m_items.push_back(std::move(p_value));
}

void ConfigNode::put(std::string p_key, ConfigNode p_value)
{
    m_kind = Kind::Map;
    for (auto& [key, value] : m_entries)
    {
        if (key == p_key)
        {
            value = std::move(p_value);
            return;
        }
    }
    m_entries.emplace_back(std::move(p_key), std::move(p_value));
}

} // namespace robotik
