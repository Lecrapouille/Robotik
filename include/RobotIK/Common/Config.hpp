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
/// (void)world.set("world.entities.easel.friction", robotik::ConfigNode("0.3"));
/// double const friction = world.at("world").at("entities")
///                              .item("easel").number("friction", 1.0).take();
/// @endcode
///
/// @par Contract
/// - Promises: @c load and @c parse return a @c Result — a message names the
///   file and the line when the YAML is wrong.
/// - Expects: an override path (@c set) names a field that exists, or a new key
///   of a map.
/// - Errors: nothing throws. A field of the wrong type (@c number on a
///   word) and an unknown key (@ref robotik::ConfigNode::checkKeys) give a
///   failed @c Result; the readers pass it on with @c ROBOTIK_TRY.
#pragma once

#include "RobotIK/Common/Math.hpp"
#include "RobotIK/Common/Result.hpp"

#include <array>
#include <filesystem>
#include <initializer_list>
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
/// Every YAML file of RobotIK (world, scenario, experiment, trajectory) is
/// read through this class, so they share one set of rules and one kind of
/// message. A missing key gives a null node, and the typed reads return
/// their default. A value of the wrong shape gives a failed @c Result: a
/// file is user input, so its errors are values, not exceptions (see
/// Result.hpp). Because a default hides a typo, each reader also lists the
/// keys it knows with @ref checkKeys: any other key stops the load, with the
/// closest known key in the message. The tree can be changed before it is
/// read (@ref set), which is how the overrides of an experiment work.
///
/// @code
/// robotik::ConfigNode const node = robotik::ConfigNode::parse(
///     "{ mass: 0.2, size: [0.04, 0.04, 0.04], tags: [red, small] }").take();
/// robotik::Result<double> m = node.number("mass", 1.0);       // 0.2
/// robotik::Result<robotik::Vector3> s =
///     node.vector3("size", robotik::Vector3::ZERO);
/// std::vector<std::string> tags = node.strings("tags");      // red, small
/// robotik::Result<bool> fixed = node.flag("fixed", false);   // missing: false
///
/// // In a reader that returns a Result:
/// ROBOTIK_TRY(node.checkKeys({ "mass", "size", "tags", "fixed" }, "box"));
/// ROBOTIK_TRY_ASSIGN(spec.mass, node.number("mass", spec.mass));
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

    // -------------------------------------------------------------------------
    /// Refuses a key that the reader does not know.
    ///
    /// A missing key gives a default value, so a typo (@c phse for
    /// @c phase) would change the meaning of a file without a word. Each
    /// reader names the keys of its map; any other key stops the load.
    ///
    /// @code
    /// ROBOTIK_TRY(p_node.checkKeys(
    ///     { "condition", "phase", "at", "within", "stop" }, "assertion"));
    /// // phse: always  ->  "assertion: unknown key 'phse' (did you mean
    /// //                    'phase'?)"
    /// @endcode
    ///
    /// A null node and a scalar pass: the reader checks the shape itself.
    /// @param p_keys Keys the reader knows.
    /// @param p_where Name of the map, for the message (e.g. @c "planner").
    /// @return Fails naming the key, the closest known key, and the known
    ///   keys when none is close.
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<void>
    checkKeys(std::initializer_list<std::string_view> p_keys,
              std::string_view p_where) const;

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
    /// @return The number; fails when the value is present but not a number.
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<double> number(std::string_view p_key,
                                        double p_default) const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key.
    /// @param p_default Value when the key is missing.
    /// @return The boolean; fails when the value is present but not a
    ///   boolean (true, false, yes, no, on, off, 1, 0).
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<bool> flag(std::string_view p_key,
                                    bool p_default) const;

    /// @return This scalar as a number, or nothing.
    [[nodiscard]] std::optional<double> asNumber() const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key of a @c [x, y, z] list.
    /// @param p_default Value when the key is missing.
    /// @return The three numbers; fails on another shape.
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<Vector3> vector3(std::string_view p_key,
                                          Vector3 p_default) const;

    // -------------------------------------------------------------------------
    /// @param p_key Map key of a @c [min, max] list.
    /// @return The two numbers, [0, 0] when missing; fails on another shape
    ///   or when max < min.
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<std::array<double, 2>>
    range(std::string_view p_key) const;

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
    /// @return The pose, identity when missing; fails on another shape or an
    ///   unknown key.
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<Pose> pose(std::string_view p_key) const;

    /// @return This map read as a pose (see @ref pose), without checking its
    ///   keys: a sensor mount shares its map with other keys.
    [[nodiscard]] Result<Pose> asPose() const;

    // --- Edition ------------------------------------------------------------

    // -------------------------------------------------------------------------
    /// Replaces (or creates) a node deep in the tree. Missing map levels are
    /// created; a missing sequence item is an error.
    /// @param p_path Dotted path; segments are map keys or, in a sequence,
    ///   an item @c id or index (see @ref item).
    /// @param p_value New node.
    /// @return Fails when the path cannot be followed.
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<void> set(std::string_view p_path, ConfigNode p_value);

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
