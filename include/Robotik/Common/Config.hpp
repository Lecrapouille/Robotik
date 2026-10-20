// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Config.hpp
/// Editable tree of a declarative file (world, scenario, experiment).
///
/// The YAML parser gives a read-only view. Experiments need to change a
/// value before the file is interpreted (@c overrides), so files are first
/// copied into this tree, edited, then read.
///
/// @code
/// robotik::ConfigNode world =
///     robotik::ConfigNode::load("painting_lab.world.yaml").take();
/// world.set("world.entities.easel.friction", robotik::ConfigNode("0.3"));
/// double const friction = world.at("world").at("entities")
///                              .item("easel").number("friction", 1.0);
/// @endcode
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Common/Result.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One node: nothing, a scalar, a sequence or an ordered map.
///
/// Reading a missing key never throws: it gives a null node, and the typed
/// reads return their default. Only a value of the wrong shape throws.
///
/// @code
/// robotik::ConfigNode const node = robotik::ConfigNode::parse(
///     "{ mass: 0.2, size: [0.04, 0.04, 0.04], tags: [red, small] }").take();
/// double m = node.number("mass", 1.0);                          // 0.2
/// robotik::Vector3 s = node.vector3("size", robotik::Vector3::ZERO);
/// std::vector<std::string> tags = node.strings("tags");         // red, small
/// bool fixed = node.flag("fixed", false);                       // missing:
/// false
/// @endcode
// ****************************************************************************
class ConfigNode
{
public:

    // ------------------------------------------------------------------------
    /// What the node holds.
    // ------------------------------------------------------------------------
    enum class Kind : std::uint8_t
    {
        Null,     ///< Nothing (a missing key).
        Scalar,   ///< A text, a number or a boolean, kept as text.
        Sequence, ///< A list of nodes.
        Map,      ///< Keys and nodes, in file order.
    };

    /// Null node.
    ConfigNode() = default;

    /// Scalar node.
    /// @param p_scalar Text of the scalar.
    explicit ConfigNode(std::string p_scalar)
        : m_kind(Kind::Scalar), m_scalar(std::move(p_scalar))
    {
    }

    // -------------------------------------------------------------------------
    /// Reads a YAML file.
    /// @param p_path File to read.
    /// @return The root node; fails when the file cannot be parsed.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<ConfigNode> load(std::filesystem::path const& p_path);

    // -------------------------------------------------------------------------
    /// Parses YAML text (tests, inline values).
    /// @param p_text YAML text.
    /// @return The root node; fails when the text cannot be parsed.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<ConfigNode> parse(std::string const& p_text);

    /// @return What the node holds.
    [[nodiscard]] Kind kind() const
    {
        return m_kind;
    }

    /// @return True for a null node (e.g. a missing key).
    [[nodiscard]] bool isNull() const
    {
        return m_kind == Kind::Null;
    }

    /// @return True for a map.
    [[nodiscard]] bool isMap() const
    {
        return m_kind == Kind::Map;
    }

    /// @return True for a sequence.
    [[nodiscard]] bool isSequence() const
    {
        return m_kind == Kind::Sequence;
    }

    /// @return True for a scalar.
    [[nodiscard]] bool isScalar() const
    {
        return m_kind == Kind::Scalar;
    }

    /// @return Text of a scalar, empty otherwise.
    [[nodiscard]] std::string const& scalar() const
    {
        return m_scalar;
    }

    // --- Maps ---------------------------------------------------------------

    /// @return True when this map has the key @p_key.
    [[nodiscard]] bool has(std::string_view p_key) const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key.
    /// @return Child @p_key, or a null node.
    // -------------------------------------------------------------------------
    [[nodiscard]] ConfigNode const& at(std::string_view p_key) const;

    /// @return Keys and values, in file order.
    [[nodiscard]] std::vector<std::pair<std::string, ConfigNode>> const&
    entries() const
    {
        return m_entries;
    }

    // --- Sequences ----------------------------------------------------------

    /// @return The items of a sequence.
    [[nodiscard]] std::vector<ConfigNode> const& items() const
    {
        return m_items;
    }

    // -------------------------------------------------------------------------
    /// Finds an item of a sequence.
    /// @param p_id Value of the @c id field of the item, or an index.
    /// @return The item whose @c id is @p_id, or the item at that index when
    ///   @p_id is a number; a null node otherwise.
    // -------------------------------------------------------------------------
    [[nodiscard]] ConfigNode const& item(std::string_view p_id) const;

    // --- Typed reads (the node is a map) -------------------------------------

    // -------------------------------------------------------------------------
    /// @param p_key Map key.
    /// @param p_default Value when the key is missing.
    /// @return The scalar text.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string text(std::string_view p_key,
                                   std::string_view p_default = {}) const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key.
    /// @param p_default Value when the key is missing.
    /// @return The number.
    /// @throws std::runtime_error when the value is present but not a number.
    // -------------------------------------------------------------------------
    [[nodiscard]] double number(std::string_view p_key, double p_default) const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key.
    /// @param p_default Value when the key is missing.
    /// @return The boolean.
    /// @throws std::runtime_error when the value is present but not a boolean.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool flag(std::string_view p_key, bool p_default) const;

    /// @return This scalar as a number, or nothing.
    [[nodiscard]] std::optional<double> asNumber() const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key of a @c [x, y, z] list.
    /// @param p_default Value when the key is missing.
    /// @return The three numbers.
    /// @throws std::runtime_error on another shape.
    // -------------------------------------------------------------------------
    [[nodiscard]] Vector3 vector3(std::string_view p_key,
                                  Vector3 p_default) const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key of a @c [min, max] list.
    /// @return The two numbers.
    /// @throws std::runtime_error on another shape.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::array<double, 2> range(std::string_view p_key) const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key.
    /// @return Scalar items of a sequence, or the scalar itself.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::string>
    strings(std::string_view p_key) const;

    // -------------------------------------------------------------------------
    /// Reads a pose: @c { position: [x, y, z], rpy: [r, p, y] } (radians), or
    /// @c quaternion: [w, x, y, z] instead of @c rpy.
    /// @param p_key Map key.
    /// @return The pose; identity when missing.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose pose(std::string_view p_key) const;

    /// @return This map read as a pose (see @ref pose).
    [[nodiscard]] Pose asPose() const;

    // --- Edition ------------------------------------------------------------

    // -------------------------------------------------------------------------
    /// Replaces (or creates) a node deep in the tree. Missing map levels are
    /// created; a missing sequence item is an error.
    /// @param p_path Dotted path; segments are map keys or, in a sequence,
    ///   an item @c id or index (see @ref item).
    /// @param p_value New node.
    /// @throws std::runtime_error when the path cannot be followed.
    // -------------------------------------------------------------------------
    void set(std::string_view p_path, ConfigNode p_value);

    // -------------------------------------------------------------------------
    /// Appends to a sequence (a null node becomes a sequence).
    /// @param p_value Item to append.
    // -------------------------------------------------------------------------
    void push(ConfigNode p_value);

    // -------------------------------------------------------------------------
    /// Sets a map entry (a null node becomes a map).
    /// @param p_key Map key.
    /// @param p_value New node.
    // -------------------------------------------------------------------------
    void put(std::string p_key, ConfigNode p_value);

private:

    /// Child of one @ref set path segment, created for a missing map key.
    [[nodiscard]] ConfigNode* child(std::string_view p_segment);

    /// What the node holds.
    Kind m_kind = Kind::Null;
    /// Text of a scalar.
    std::string m_scalar;
    /// Items of a sequence.
    std::vector<ConfigNode> m_items;
    /// Entries of a map, in file order.
    std::vector<std::pair<std::string, ConfigNode>> m_entries;
};

} // namespace robotik
