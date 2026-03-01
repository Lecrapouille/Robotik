/**
 * @file Resolver.hpp
 * @brief Variable resolver for blackboard references.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Blackboard/Blackboard.hpp"

#include <optional>
#include <regex>
#include <string>
#include <type_traits>

namespace robotik::bt {

// ****************************************************************************
//! \brief Class representing a variable resolver.
//!
//! The VariableResolver handles resolution of ${variable} syntax in strings
//! and values, looking up the actual values from a Blackboard.
// ****************************************************************************
class VariableResolver
{
public:

    // ------------------------------------------------------------------------
    //! \brief Resolve a variable.
    //! \param[in] p_str The string to resolve.
    //! \param[in] p_bb The blackboard to resolve the variable.
    //! \return The resolved string.
    // ------------------------------------------------------------------------
    static std::string resolve(const std::string& p_str, const Blackboard& p_bb)
    {
        std::regex pattern(R"(\$\{([^}]+)\})");
        std::string result = p_str;
        std::smatch match;

        auto search_start = result.cbegin();
        while (std::regex_search(search_start, result.cend(), match, pattern))
        {
            std::string key = match[1].str();

            if (auto value = p_bb.get<std::string>(key))
            {
                const auto offset = static_cast<std::string::size_type>(
                    std::distance(result.cbegin(), search_start));
                const auto pos = offset + static_cast<std::string::size_type>(
                                              match.position(0));
                const auto len =
                    static_cast<std::string::size_type>(match.length(0));
                result.replace(pos, len, *value);
                const std::string::size_type new_index = pos + value->length();
                search_start =
                    result.cbegin() + static_cast<std::ptrdiff_t>(new_index);
            }
            else
            {
                search_start = match.suffix().first;
            }
        }

        return result;
    }

    // ------------------------------------------------------------------------
    //! \brief Resolve a value.
    //! \param[in] p_expr The expression to resolve.
    //! \param[in] p_bb The blackboard to resolve the value.
    //! \return The resolved value.
    // ------------------------------------------------------------------------
    template <typename T>
    static std::optional<T> resolveValue(const std::string& p_expr,
                                         const Blackboard& p_bb)
    {
        // If it is a reference ${key}
        std::regex pattern(R"(\$\{([^}]+)\})");
        std::smatch match;

        if (std::regex_match(p_expr, match, pattern))
        {
            std::string key = match[1].str();
            return p_bb.get<T>(key);
        }

        // Otherwise, it is a literal value
        return parseLiteral<T>(p_expr);
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Parse a literal value.
    //! \param[in] p_str The string to parse.
    //! \return The parsed value.
    // ------------------------------------------------------------------------
    template <typename T>
    static std::optional<T> parseLiteral(const std::string& p_str)
    {
        if constexpr (std::is_same_v<T, std::string>)
        {
            return p_str;
        }
        else if constexpr (std::is_same_v<T, int>)
        {
            try
            {
                return std::stoi(p_str);
            }
            catch (...)
            {
                return std::nullopt;
            }
        }
        else if constexpr (std::is_same_v<T, double>)
        {
            try
            {
                return std::stod(p_str);
            }
            catch (...)
            {
                return std::nullopt;
            }
        }
        else if constexpr (std::is_same_v<T, bool>)
        {
            if (p_str == "true" || p_str == "1")
                return true;
            if (p_str == "false" || p_str == "0")
                return false;
            return std::nullopt;
        }
        return std::nullopt;
    }
};

} // namespace robotik::bt
