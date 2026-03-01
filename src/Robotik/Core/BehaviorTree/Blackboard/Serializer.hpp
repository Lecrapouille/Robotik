/**
 * @file Serializer.hpp
 * @brief Blackboard serialization to/from YAML.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Blackboard/Blackboard.hpp"

#include <regex>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace robotik::bt {

// ****************************************************************************
//! \brief Utility for loading/storing blackboard content from/to YAML.
//!
//! The blackboard serializer is a utility class that is used to load the
//! blackboard content from a YAML node and to store the blackboard content into
//! a YAML node.
//!
//! Key features:
//! - Load the blackboard content from a YAML node.
//! - Store the blackboard content into a YAML node.
//! - Resolve ${var} references in the blackboard content.
// ****************************************************************************
class BlackboardSerializer
{
public:

    // ------------------------------------------------------------------------
    //! \brief Populate a blackboard from a YAML node.
    //! \param[in,out] p_target Blackboard to populate.
    //! \param[in] p_node YAML map containing key/value pairs.
    //! \param[in] p_reference Optional scope used to resolve ${var} references.
    // ------------------------------------------------------------------------
    static void load(Blackboard& p_target,
                     YAML::Node const& p_node,
                     Blackboard const* p_reference = nullptr)
    {
        if (!p_node || !p_node.IsMap())
        {
            return;
        }

        Blackboard const* scope =
            p_reference != nullptr ? p_reference : &p_target;

        for (auto const& entry : p_node)
        {
            auto key = entry.first.as<std::string>();
            p_target.m_data[key] = toAny(entry.second, scope);
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Serialize the content of a blackboard into YAML.
    //! \param[in] p_source Blackboard to serialize.
    //! \return YAML node representing the stored data.
    // ------------------------------------------------------------------------
    [[nodiscard]] static YAML::Node dump(Blackboard const& p_source)
    {
        YAML::Node node(YAML::NodeType::Map);
        for (auto const& [key, value] : p_source.m_data)
        {
            node[key] = toYaml(value);
        }
        return node;
    }

private:

    static bool isReference(std::string const& p_literal, std::string& p_key)
    {
        static std::regex pattern(R"(\$\{([^}]+)\})");
        std::smatch match;
        if (std::regex_match(p_literal, match, pattern))
        {
            p_key = match[1].str();
            return true;
        }
        return false;
    }

    static std::any toAny(YAML::Node const& p_node, Blackboard const* p_scope)
    {
        if (!p_node)
        {
            return {};
        }

        if (p_node.IsScalar())
        {
            std::string literal = p_node.Scalar();
            std::string key;
            if (p_scope != nullptr && isReference(literal, key))
            {
                if (auto value = p_scope->raw(key))
                {
                    return *value;
                }
            }

            try
            {
                int i = p_node.as<int>();
                return i;
            }
            catch (...)
            {
            }

            try
            {
                double d = p_node.as<double>();
                return d;
            }
            catch (...)
            {
            }

            try
            {
                bool b = p_node.as<bool>();
                return b;
            }
            catch (...)
            {
            }

            return literal;
        }

        if (p_node.IsSequence())
        {
            std::vector<std::any> generic;
            generic.reserve(p_node.size());

            bool numeric_only = true;
            std::vector<double> numeric_values;
            numeric_values.reserve(p_node.size());

            for (auto const& element : p_node)
            {
                auto entry = toAny(element, p_scope);

                if (entry.type() == typeid(int))
                {
                    numeric_values.push_back(
                        static_cast<double>(std::any_cast<int>(entry)));
                }
                else if (entry.type() == typeid(double))
                {
                    numeric_values.push_back(std::any_cast<double>(entry));
                }
                else
                {
                    numeric_only = false;
                }

                generic.push_back(std::move(entry));
            }

            if (numeric_only)
            {
                return numeric_values;
            }

            return generic;
        }

        if (p_node.IsMap())
        {
            std::unordered_map<std::string, std::any> map;
            for (auto const& element : p_node)
            {
                map.emplace(element.first.as<std::string>(),
                            toAny(element.second, p_scope));
            }
            return map;
        }

        return {};
    }

    static YAML::Node toYaml(std::any const& p_value)
    {
        if (!p_value.has_value())
        {
            return YAML::Node();
        }

        if (p_value.type() == typeid(int))
        {
            return YAML::Node(std::any_cast<int>(p_value));
        }
        if (p_value.type() == typeid(double))
        {
            return YAML::Node(std::any_cast<double>(p_value));
        }
        if (p_value.type() == typeid(bool))
        {
            return YAML::Node(std::any_cast<bool>(p_value));
        }
        if (p_value.type() == typeid(std::string))
        {
            return YAML::Node(std::any_cast<std::string>(p_value));
        }
        if (p_value.type() == typeid(std::vector<double>))
        {
            YAML::Node node(YAML::NodeType::Sequence);
            for (double entry :
                 std::any_cast<std::vector<double> const&>(p_value))
            {
                node.push_back(entry);
            }
            return node;
        }
        if (p_value.type() == typeid(std::vector<std::any>))
        {
            YAML::Node node(YAML::NodeType::Sequence);
            for (auto const& entry :
                 std::any_cast<std::vector<std::any> const&>(p_value))
            {
                node.push_back(toYaml(entry));
            }
            return node;
        }
        if (p_value.type() == typeid(std::unordered_map<std::string, std::any>))
        {
            YAML::Node node(YAML::NodeType::Map);
            for (auto const& [key, entry] : std::any_cast<
                     std::unordered_map<std::string, std::any> const&>(p_value))
            {
                node[key] = toYaml(entry);
            }
            return node;
        }

        return YAML::Node();
    }
};

} // namespace robotik::bt
