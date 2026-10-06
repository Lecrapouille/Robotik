// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Metrics.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <sstream>

namespace robotik
{

namespace
{

std::string_view trim(std::string_view p_text)
{
    std::size_t const first = p_text.find_first_not_of(" \t");
    if (first == std::string_view::npos)
    {
        return {};
    }
    std::size_t const last = p_text.find_last_not_of(" \t");
    return p_text.substr(first, last - first + 1u);
}

struct Comparison
{
    std::string_view metric;
    std::string_view op;
    std::string_view number;
};

//! Splits at the first comparison operator found outside quotes.
std::optional<Comparison> split(std::string_view p_text)
{
    static constexpr std::array<std::string_view, 6> operators = {
        "==", "!=", "<=", ">=", "<", ">"
    };
    bool quoted = false;
    for (std::size_t i = 0; i < p_text.size(); ++i)
    {
        if (p_text[i] == '"')
        {
            quoted = !quoted;
            continue;
        }
        if (quoted)
        {
            continue;
        }
        for (std::string_view op : operators)
        {
            if (p_text.substr(i, op.size()) == op)
            {
                return Comparison{ trim(p_text.substr(0, i)),
                                   op,
                                   trim(p_text.substr(i + op.size())) };
            }
        }
    }
    return std::nullopt;
}

bool compare(double p_value, std::string_view p_op, double p_reference)
{
    if (p_op == "==")
        return std::abs(p_value - p_reference) <= 1e-9;
    if (p_op == "!=")
        return std::abs(p_value - p_reference) > 1e-9;
    if (p_op == "<=")
        return p_value <= p_reference;
    if (p_op == ">=")
        return p_value >= p_reference;
    if (p_op == "<")
        return p_value < p_reference;
    return p_value > p_reference;
}

std::string format(double p_value)
{
    std::ostringstream stream;
    stream << p_value;
    return stream.str();
}

} // namespace

void Metrics::set(std::string_view p_name, double p_value)
{
    for (std::size_t i = 0; i < m_names.size(); ++i)
    {
        if (m_names[i] == p_name)
        {
            m_values[i] = p_value;
            return;
        }
    }
    m_names.emplace_back(p_name);
    m_values.push_back(p_value);
}

std::optional<double> Metrics::get(std::string_view p_name) const
{
    for (std::size_t i = 0; i < m_names.size(); ++i)
    {
        if (m_names[i] == p_name)
        {
            return m_values[i];
        }
    }
    return std::nullopt;
}

Check evaluate(std::string_view p_assertion,
               Metrics const& p_metrics,
               MetricResolver const& p_resolver)
{
    Check check{ std::string(p_assertion), false, {} };
    std::optional<Comparison> const comparison = split(p_assertion);
    std::string_view const metric =
        comparison ? comparison->metric : trim(p_assertion);

    std::optional<double> value = p_metrics.get(metric);
    if (!value && p_resolver)
    {
        value = p_resolver(metric);
    }
    if (!value)
    {
        check.detail = "unknown metric '" + std::string(metric) + "'";
        return check;
    }
    check.detail = std::string(metric) + " = " + format(*value);

    if (!comparison)
    {
        check.passed = *value != 0.0;
        return check;
    }
    double reference = 0.0;
    std::string_view const number = comparison->number;
    auto const [end, error] =
        std::from_chars(number.data(), number.data() + number.size(), reference);
    if (error != std::errc{} || end != number.data() + number.size())
    {
        check.detail = "not a number: '" + std::string(number) + "'";
        return check;
    }
    check.passed = compare(*value, comparison->op, reference);
    return check;
}

} // namespace robotik
