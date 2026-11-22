// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Condition.hpp"

#include "Robotik/Common/SExpression.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace robotik
{

using Op = ConditionNode::Op;

/// Words of the notation, with what they compute.
static constexpr std::array<std::pair<std::string_view, Op>, 10> OPERATORS = { {
    { "and", Op::And },
    { "or", Op::Or },
    { "not", Op::Not },
    { "<", Op::Less },
    { "<=", Op::LessEqual },
    { ">", Op::Greater },
    { ">=", Op::GreaterEqual },
    { "=", Op::Equal },
    { "==", Op::Equal },
    { "!=", Op::NotEqual },
} };

/// Two values closer than this are equal for @c = and @c !=.
static constexpr double EQUAL_TOLERANCE = 1e-9;

/// True for the comparisons (two numbers give a truth).
static bool comparison(Op p_op)
{
    return p_op == Op::Less || p_op == Op::LessEqual || p_op == Op::Greater ||
           p_op == Op::GreaterEqual || p_op == Op::Equal || p_op == Op::NotEqual;
}

/// Prints a number the way a condition shows it.
static std::string format(double p_value)
{
    std::ostringstream stream;
    stream << p_value;
    return stream.str();
}

/// Reads a whole word as a number, or nothing.
static std::optional<double> numberOf(std::string_view p_word)
{
    double value = 0.0;
    std::string_view word = p_word;
    if (!word.empty() && word.front() == '+')
    {
        word.remove_prefix(1U);
    }
    auto const [end, error] =
        std::from_chars(word.data(), word.data() + word.size(), value);
    if (word.empty() || error != std::errc{} ||
        end != word.data() + word.size())
    {
        return std::nullopt;
    }
    return value;
}

namespace
{

/// Turns an S-expression into the flat nodes of a @ref Condition.
class Compiler
{
public:

    /// @param p_source Text of the condition, for the messages.
    /// @param p_vocabulary What the condition may name.
    /// @param p_out Condition receiving the nodes.
    Compiler(std::string_view p_source,
             ConditionVocabulary const& p_vocabulary,
             Condition& p_out)
        : m_source(p_source), m_vocabulary(p_vocabulary), m_out(p_out)
    {
    }

    /// Compiles @p_node and its children; returns the index of its node.
    /// @throws std::runtime_error "column N: why".
    std::uint32_t compile(SExpression const& p_node)
    {
        std::uint32_t const index = add(p_node);
        if (p_node.kind == SExpression::Kind::Text)
        {
            fail(p_node, "a quoted text is only a query argument");
        }
        if (p_node.kind == SExpression::Kind::Atom)
        {
            atom(index, p_node);
            return index;
        }
        SExpression const& head = p_node.items.front();
        if (head.kind != SExpression::Kind::Atom || numberOf(head.text))
        {
            fail(head, "a list starts with an operator or a query name");
        }
        for (auto const& [word, op] : OPERATORS)
        {
            if (head.text == word)
            {
                operation(index, op, p_node);
                return index;
            }
        }
        if (ConditionQuery const* query = findQuery(head.text))
        {
            bind(index, *query, p_node);
            return index;
        }
        std::vector<std::string_view> known;
        for (auto const& [word, op] : OPERATORS)
        {
            known.push_back(word);
        }
        for (ConditionQuery const& query : m_vocabulary.queries)
        {
            known.push_back(query.name);
        }
        fail(head, "unknown operator or query '" + head.text + "'" +
                       suggestion(head.text, known));
    }

private:

    /// Throws the error of a node.
    [[noreturn]] static void fail(SExpression const& p_node,
                                  std::string const& p_why)
    {
        throw std::runtime_error("column " + std::to_string(p_node.begin + 1U) +
                                 ": " + p_why);
    }

    /// " (did you mean 'x'?)", or empty.
    static std::string suggestion(std::string_view p_word,
                                  std::vector<std::string_view> const& p_known)
    {
        std::string const closest = closestWord(p_word, p_known);
        return closest.empty() ? std::string{}
                               : " (did you mean '" + closest + "'?)";
    }

    /// Appends the node of @p_node; returns its index.
    std::uint32_t add(SExpression const& p_node)
    {
        ConditionNode node;
        node.text = std::string(
            m_source.substr(p_node.begin, p_node.end - p_node.begin));
        m_out.nodes.push_back(std::move(node));
        return static_cast<std::uint32_t>(m_out.nodes.size() - 1U);
    }

    /// The query named @p_name, or null.
    ConditionQuery const* findQuery(std::string_view p_name) const
    {
        for (ConditionQuery const& query : m_vocabulary.queries)
        {
            if (query.name == p_name)
            {
                return &query;
            }
        }
        return nullptr;
    }

    /// A number or a metric name.
    void atom(std::uint32_t p_index, SExpression const& p_node)
    {
        ConditionNode& node = m_out.nodes[p_index];
        if (std::optional<double> const number = numberOf(p_node.text))
        {
            node.op = Op::Number;
            node.number = *number;
            return;
        }
        for (std::string const& metric : m_vocabulary.metrics)
        {
            if (metric == p_node.text)
            {
                node.op = Op::Metric;
                node.metric = metric;
                return;
            }
        }
        if (findQuery(p_node.text) != nullptr)
        {
            fail(p_node, "'" + p_node.text + "' is a query: write (" +
                             p_node.text + " ...) with its arguments");
        }
        std::vector<std::string_view> known(m_vocabulary.metrics.begin(),
                                            m_vocabulary.metrics.end());
        fail(p_node, "unknown value '" + p_node.text + "'" +
                         suggestion(p_node.text, known));
    }

    /// @c and, @c or, @c not or a comparison, with its operands.
    void operation(std::uint32_t p_index, Op p_op, SExpression const& p_node)
    {
        std::size_t const operands = p_node.items.size() - 1U;
        std::string const& word = p_node.items.front().text;
        if (p_op == Op::Not && operands != 1U)
        {
            fail(p_node, "(not ...) takes one condition");
        }
        if ((p_op == Op::And || p_op == Op::Or) && operands < 1U)
        {
            fail(p_node, "(" + word + " ...) takes one condition or more");
        }
        if (comparison(p_op) && operands != 2U)
        {
            fail(p_node, "(" + word + " a b) compares two values");
        }
        std::vector<std::uint32_t> children;
        for (std::size_t i = 1; i < p_node.items.size(); ++i)
        {
            children.push_back(compile(p_node.items[i]));
        }
        // compile() may grow the nodes: index again, do not keep a reference.
        m_out.nodes[p_index].op = p_op;
        m_out.nodes[p_index].children = std::move(children);
    }

    /// A query: checks its arguments and binds them.
    void bind(std::uint32_t p_index,
              ConditionQuery const& p_query,
              SExpression const& p_node)
    {
        std::vector<std::string> arguments;
        std::vector<SExpression> const* words = &p_node.items;
        std::size_t first = 1U;
        if (p_query.literal)
        {
            // (believed (in cube box)): the words of the literal.
            if (p_node.items.size() != 2U ||
                p_node.items[1].kind != SExpression::Kind::List ||
                p_node.items[1].items.empty())
            {
                fail(p_node,
                     "(" + p_query.name + " ...) takes one literal, e.g. (" +
                         p_query.name + " (in cube box))");
            }
            words = &p_node.items[1].items;
            first = 0U;
        }
        for (std::size_t i = first; i < words->size(); ++i)
        {
            SExpression const& argument = (*words)[i];
            if (argument.kind == SExpression::Kind::List)
            {
                fail(argument, "the arguments of (" + p_query.name +
                                   " ...) are names, not expressions");
            }
            arguments.push_back(argument.text);
        }
        if (arguments.size() < p_query.min_args ||
            arguments.size() > p_query.max_args)
        {
            std::string const count =
                p_query.min_args == p_query.max_args
                    ? std::to_string(p_query.min_args)
                    : std::to_string(p_query.min_args) + " to " +
                          std::to_string(p_query.max_args);
            fail(p_node, "(" + p_query.name + " ...) takes " + count +
                             " argument(s), not " +
                             std::to_string(arguments.size()));
        }
        try
        {
            // Once, to refuse an unknown entity at load time.
            (void)p_query.answer(arguments);
        }
        catch (std::invalid_argument const& error)
        {
            fail(p_node, error.what());
        }
        ConditionNode& node = m_out.nodes[p_index];
        node.op = Op::Query;
        node.query = [answer = p_query.answer, arguments = std::move(arguments)]()
        { return answer(arguments); };
    }

    /// Text of the condition.
    std::string_view m_source;
    /// What the condition may name.
    ConditionVocabulary const& m_vocabulary;
    /// Condition being built.
    Condition& m_out;
};

/// Evaluates the nodes of a condition on the metrics of one step.
class Evaluator
{
public:

    /// @param p_condition Compiled condition.
    /// @param p_metrics Metrics of this step.
    Evaluator(Condition const& p_condition, Metrics const& p_metrics)
        : m_nodes(p_condition.nodes), m_metrics(p_metrics)
    {
    }

    /// Evaluation of node @p_index, with the text of the reports.
    Evaluation evaluation(std::uint32_t p_index) const
    {
        ConditionNode const& node = m_nodes[p_index];
        Evaluation result;
        switch (node.op)
        {
            case Op::Not:
            {
                result = evaluation(node.children[0]);
                result.holds = !result.holds;
                result.expected = "not " + result.expected;
                return result;
            }
            case Op::And:
            {
                // Shows the first child that fails, or the last one.
                for (std::uint32_t child : node.children)
                {
                    Evaluation const e = evaluation(child);
                    if (!e.known || !e.holds)
                    {
                        return e;
                    }
                }
                return evaluation(node.children.back());
            }
            case Op::Or:
            {
                // Shows the first child that holds, else the first unknown,
                // else the first one.
                std::optional<Evaluation> unknown;
                for (std::uint32_t child : node.children)
                {
                    Evaluation e = evaluation(child);
                    if (e.known && e.holds)
                    {
                        return e;
                    }
                    if (!e.known && !unknown)
                    {
                        unknown = std::move(e);
                    }
                }
                return unknown ? *unknown : evaluation(node.children.front());
            }
            default:
                break;
        }
        if (comparison(node.op))
        {
            Value const left = value(node.children[0]);
            Value const right = value(node.children[1]);
            ConditionNode const& rhs = m_nodes[node.children[1]];
            result.expected = operatorWord(node.op) + " " +
                              (rhs.op == Op::Number
                                   ? rhs.text
                                   : rhs.text + " (" + format(right.value) + ")");
            if (!left.known || !right.known)
            {
                result.detail = left.known ? right.why : left.why;
                return result;
            }
            result.known = true;
            result.value = left.value;
            result.detail = m_nodes[node.children[0]].text + " = " +
                            format(left.value);
            result.holds = compare(node.op, left.value, right.value);
            return result;
        }
        Value const v = value(p_index);
        result.expected = "!= 0";
        if (!v.known)
        {
            result.detail = v.why;
            return result;
        }
        result.known = true;
        result.value = v.value;
        result.holds = v.value != 0.0;
        result.detail = node.text + " = " + format(v.value);
        return result;
    }

private:

    /// A number, or why there is none.
    struct Value
    {
        bool known = false;
        double value = 0.0;
        std::string why;
    };

    /// The word of a comparison, for the messages.
    static std::string operatorWord(Op p_op)
    {
        for (auto const& [word, op] : OPERATORS)
        {
            if (op == p_op)
            {
                return std::string(word);
            }
        }
        return "?";
    }

    /// Compares two values under a comparison operator.
    static bool compare(Op p_op, double p_left, double p_right)
    {
        switch (p_op)
        {
            case Op::Less:
                return p_left < p_right;
            case Op::LessEqual:
                return p_left <= p_right;
            case Op::Greater:
                return p_left > p_right;
            case Op::GreaterEqual:
                return p_left >= p_right;
            case Op::Equal:
                return std::abs(p_left - p_right) <= EQUAL_TOLERANCE;
            case Op::NotEqual:
                return std::abs(p_left - p_right) > EQUAL_TOLERANCE;
            default:
                return false;
        }
    }

    /// Number of node @p_index; a truth counts 1 or 0.
    Value value(std::uint32_t p_index) const
    {
        ConditionNode const& node = m_nodes[p_index];
        Value result;
        switch (node.op)
        {
            case Op::Number:
                result.known = true;
                result.value = node.number;
                return result;
            case Op::Metric:
                if (std::optional<double> const v = m_metrics.get(node.metric))
                {
                    result.known = true;
                    result.value = *v;
                }
                else
                {
                    result.why = "metric '" + node.metric +
                                 "' is not published at this step";
                }
                return result;
            case Op::Query:
                try
                {
                    if (std::optional<double> const v = node.query())
                    {
                        result.known = true;
                        result.value = *v;
                    }
                    else
                    {
                        result.why = node.text + " has no answer now";
                    }
                }
                catch (std::exception const& error)
                {
                    result.why = node.text + ": " + error.what();
                }
                return result;
            default:
            {
                Evaluation const e = evaluation(p_index);
                result.known = e.known;
                result.value = e.holds ? 1.0 : 0.0;
                result.why = e.detail;
                return result;
            }
        }
    }

    /// Nodes of the condition.
    std::vector<ConditionNode> const& m_nodes;
    /// Metrics of this step.
    Metrics const& m_metrics;
};

} // namespace

Result<Condition> compileCondition(std::string_view p_text,
                                   ConditionVocabulary const& p_vocabulary)
{
    Result<SExpression> parsed = parseSExpression(p_text);
    if (!parsed)
    {
        return failure("condition '" + std::string(p_text) + "': " +
                       parsed.error());
    }
    Condition condition;
    condition.text = std::string(p_text);
    try
    {
        Compiler(p_text, p_vocabulary, condition).compile(parsed.value());
    }
    catch (std::runtime_error const& error)
    {
        return failure("condition '" + std::string(p_text) + "': " +
                       error.what());
    }
    return condition;
}

Evaluation evaluate(Condition const& p_condition, Metrics const& p_metrics)
{
    if (p_condition.nodes.empty())
    {
        Evaluation always;
        always.known = true;
        always.holds = true;
        return always;
    }
    return Evaluator(p_condition, p_metrics).evaluation(0U);
}

Evaluation evaluate(std::string_view p_text, Metrics const& p_metrics)
{
    ConditionVocabulary vocabulary;
    for (std::string const& name : p_metrics.names())
    {
        vocabulary.metrics.push_back(name);
    }
    Result<Condition> condition = compileCondition(p_text, vocabulary);
    if (!condition)
    {
        Evaluation failed;
        failed.detail = condition.error();
        return failed;
    }
    return evaluate(condition.value(), p_metrics);
}

} // namespace robotik
