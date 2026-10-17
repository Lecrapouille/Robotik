// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Config.hpp
//! @brief Editable tree of a declarative file (world, scenario, experiment).
//!
//! The YAML parser gives a read-only view. Experiments need to change a
//! value before the file is interpreted (@c overrides), so files are first
//! copied into this tree, edited, then read.
//!
//! @code
//! robotik::ConfigNode world = robotik::ConfigNode::load("painting_lab.world.yaml");
//! world.set("world.entities.easel.friction", robotik::ConfigNode("0.3"));
//! double const friction = world.at("world").at("entities")
//!                              .item("easel").number("friction", 1.0);
//! @endcode
#pragma once

#include "Robotik/Math/Geometry.hpp"

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
//! @brief One node: nothing, a scalar, a sequence or an ordered map.
// ****************************************************************************
class ConfigNode
{
public:

    enum class Kind : std::uint8_t
    {
        Null,
        Scalar,
        Sequence,
        Map,
    };

    ConfigNode() = default;

    //! @brief Scalar node.
    explicit ConfigNode(std::string p_scalar)
        : m_kind(Kind::Scalar), m_scalar(std::move(p_scalar))
    {
    }

    // -------------------------------------------------------------------------
    //! @brief Reads a YAML file.
    //! @throws std::runtime_error when the file cannot be parsed.
    // -------------------------------------------------------------------------
    [[nodiscard]] static ConfigNode load(std::filesystem::path const& p_path);

    //! @brief Parses YAML text (tests, inline values).
    [[nodiscard]] static ConfigNode parse(std::string const& p_text);

    [[nodiscard]] Kind kind() const
    {
        return m_kind;
    }

    [[nodiscard]] bool isNull() const
    {
        return m_kind == Kind::Null;
    }

    [[nodiscard]] bool isMap() const
    {
        return m_kind == Kind::Map;
    }

    [[nodiscard]] bool isSequence() const
    {
        return m_kind == Kind::Sequence;
    }

    [[nodiscard]] bool isScalar() const
    {
        return m_kind == Kind::Scalar;
    }

    //! @brief Text of a scalar, empty otherwise.
    [[nodiscard]] std::string const& scalar() const
    {
        return m_scalar;
    }

    // --- Maps ---------------------------------------------------------------

    [[nodiscard]] bool has(std::string_view p_key) const;

    //! @brief Child @p_key, or a null node.
    [[nodiscard]] ConfigNode const& at(std::string_view p_key) const;

    //! @brief Keys and values, in file order.
    [[nodiscard]] std::vector<std::pair<std::string, ConfigNode>> const& entries() const
    {
        return m_entries;
    }

    // --- Sequences ----------------------------------------------------------

    [[nodiscard]] std::vector<ConfigNode> const& items() const
    {
        return m_items;
    }

    // -------------------------------------------------------------------------
    //! @brief Item of a sequence whose @c id is @p_id, or the item at that
    //! index when @p_id is a number; a null node otherwise.
    // -------------------------------------------------------------------------
    [[nodiscard]] ConfigNode const& item(std::string_view p_id) const;

    // --- Typed reads (the node is a map) -------------------------------------

    [[nodiscard]] std::string text(std::string_view p_key,
                                   std::string_view p_default = {}) const;

    //! @throws std::runtime_error when the value is present but not a number.
    [[nodiscard]] double number(std::string_view p_key, double p_default) const;

    //! @throws std::runtime_error when the value is present but not a boolean.
    [[nodiscard]] bool flag(std::string_view p_key, bool p_default) const;

    //! @brief This scalar as a number, or nothing.
    [[nodiscard]] std::optional<double> asNumber() const;

    //! @brief Three numbers. @throws std::runtime_error on another shape.
    [[nodiscard]] Vector3 vector3(std::string_view p_key, Vector3 p_default) const;

    //! @brief @c [min, max]. @throws std::runtime_error on another shape.
    [[nodiscard]] std::array<double, 2> range(std::string_view p_key) const;

    //! @brief Scalar items of a sequence, or the scalar itself.
    [[nodiscard]] std::vector<std::string> strings(std::string_view p_key) const;

    // -------------------------------------------------------------------------
    //! @brief @c { position: [x, y, z], rpy: [r, p, y] } (radians), or
    //! @c quaternion: [w, x, y, z] instead of @c rpy. Missing: identity.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose pose(std::string_view p_key) const;

    //! @brief This map read as a pose (see @ref pose).
    [[nodiscard]] Pose asPose() const;

    // --- Edition ------------------------------------------------------------

    // -------------------------------------------------------------------------
    //! @brief Replaces (or creates) the node at @p_path. Segments are map keys
    //! or, in a sequence, an item @c id or index (see @ref item). Missing map
    //! levels are created; a missing sequence item is an error.
    //! @throws std::runtime_error when the path cannot be followed.
    // -------------------------------------------------------------------------
    void set(std::string_view p_path, ConfigNode p_value);

    //! @brief Appends to a sequence (a null node becomes a sequence).
    void push(ConfigNode p_value);

    //! @brief Sets a map entry (a null node becomes a map).
    void put(std::string p_key, ConfigNode p_value);

private:

    [[nodiscard]] ConfigNode* child(std::string_view p_segment);

    Kind m_kind = Kind::Null;
    std::string m_scalar;
    std::vector<ConfigNode> m_items;
    std::vector<std::pair<std::string, ConfigNode>> m_entries;
};

} // namespace robotik
