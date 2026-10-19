// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Task/Planning/Pddl.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace robotik
{

// ****************************************************************************
/// @brief An S-expression: a lower-case atom or a list.
// ****************************************************************************
struct Sexpr
{
    std::string atom;
    std::vector<Sexpr> list;
    bool is_list = false;

    [[nodiscard]] bool isAtom(std::string_view p_text) const
    {
        return !is_list && atom == p_text;
    }

    /// @brief Head of a list: @c "and" for @c (and ...), empty otherwise.
    [[nodiscard]] std::string_view head() const
    {
        return is_list && !list.empty() && !list[0].is_list
                   ? std::string_view(list[0].atom)
                   : std::string_view();
    }
};

// ****************************************************************************
/// @brief Reads PDDL text into S-expressions; comments start with ';'.
// ****************************************************************************
class Reader
{
public:

    explicit Reader(std::string_view p_text) : m_text(p_text) {}

    Sexpr read()
    {
        skip();
        if (m_at >= m_text.size())
        {
            throw std::runtime_error("PDDL: empty text");
        }
        Sexpr result = next();
        skip();
        if (m_at < m_text.size())
        {
            throw std::runtime_error(
                "PDDL: text after the closing parenthesis, line " +
                std::to_string(line()));
        }
        return result;
    }

private:

    void skip()
    {
        while (m_at < m_text.size())
        {
            char const c = m_text[m_at];
            if (c == ';')
            {
                while (m_at < m_text.size() && m_text[m_at] != '\n')
                {
                    ++m_at;
                }
            }
            else if (std::isspace(static_cast<unsigned char>(c)) != 0)
            {
                ++m_at;
            }
            else
            {
                return;
            }
        }
    }

    [[nodiscard]] std::size_t line() const
    {
        return 1u + static_cast<std::size_t>(
                        std::count(m_text.begin(),
                                   m_text.begin() + static_cast<long>(m_at),
                                   '\n'));
    }

    Sexpr next()
    {
        skip();
        if (m_at >= m_text.size())
        {
            throw std::runtime_error("PDDL: missing ')' at the end");
        }
        Sexpr result;
        if (m_text[m_at] == '(')
        {
            result.is_list = true;
            ++m_at;
            for (;;)
            {
                skip();
                if (m_at >= m_text.size())
                {
                    throw std::runtime_error("PDDL: missing ')' at the end");
                }
                if (m_text[m_at] == ')')
                {
                    ++m_at;
                    return result;
                }
                result.list.push_back(next());
            }
        }
        if (m_text[m_at] == ')')
        {
            throw std::runtime_error("PDDL: unexpected ')' line " +
                                     std::to_string(line()));
        }
        std::size_t const start = m_at;
        while (m_at < m_text.size() && m_text[m_at] != '(' &&
               m_text[m_at] != ')' && m_text[m_at] != ';' &&
               std::isspace(static_cast<unsigned char>(m_text[m_at])) == 0)
        {
            ++m_at;
        }
        result.atom = lowerCase(m_text.substr(start, m_at - start));
        return result;
    }

    std::string_view m_text;
    std::size_t m_at = 0u;
};

// ----------------------------------------------------------------------------
/// @brief Reads a typed list: @c "a b - t c - u d" gives a:t b:t c:u d:object.
/// @c (either ...) types are read as @c object.
// ----------------------------------------------------------------------------
static std::vector<Parameter> typedList(std::vector<Sexpr> const& p_items,
                                        std::size_t p_from)
{
    std::vector<Parameter> result;
    std::size_t pending = result.size();
    for (std::size_t i = p_from; i < p_items.size(); ++i)
    {
        Sexpr const& item = p_items[i];
        if (item.isAtom("-") && i + 1u < p_items.size())
        {
            Sexpr const& type = p_items[++i];
            std::string const name =
                type.is_list ? std::string("object") : type.atom;
            for (std::size_t k = pending; k < result.size(); ++k)
            {
                result[k].type = name;
            }
            pending = result.size();
        }
        else if (!item.is_list)
        {
            std::string name = item.atom;
            if (!name.empty() && name.front() == '?')
            {
                name.erase(0, 1);
            }
            result.push_back({ std::move(name), "object" });
        }
    }
    return result;
}

// ----------------------------------------------------------------------------
/// @brief @c (p a ?x) as an atom.
// ----------------------------------------------------------------------------
static Atom atomOf(Sexpr const& p_node)
{
    Atom atom;
    atom.predicate = std::string(p_node.head());
    for (std::size_t i = 1u; i < p_node.list.size(); ++i)
    {
        atom.args.push_back(p_node.list[i].atom);
    }
    return atom;
}

// ----------------------------------------------------------------------------
/// @brief Collects the plain effects of @p_node into @p_action. Conditional,
/// quantified and numeric effects mark the action as partial. For a durative
/// action, only @c at @c end effects are kept: they hold once it is done.
// ----------------------------------------------------------------------------
static void collectEffects(Sexpr const& p_node, ActionSchema& p_action)
{
    std::string_view const head = p_node.head();
    if (!p_node.is_list || p_node.list.empty())
    {
        return;
    }
    if (head == "and")
    {
        for (std::size_t i = 1u; i < p_node.list.size(); ++i)
        {
            collectEffects(p_node.list[i], p_action);
        }
    }
    else if (head == "at" && p_node.list.size() == 3u)
    {
        if (p_node.list[1].isAtom("end"))
        {
            collectEffects(p_node.list[2], p_action);
        }
    }
    else if (head == "not" && p_node.list.size() == 2u &&
             p_node.list[1].is_list)
    {
        p_action.effects.push_back({ atomOf(p_node.list[1]), false });
    }
    else if (head == "forall" || head == "when" || head == "increase" ||
             head == "decrease" || head == "assign" || head == "scale-up" ||
             head == "scale-down")
    {
        p_action.partial = true;
    }
    else if (!head.empty())
    {
        p_action.effects.push_back({ atomOf(p_node), true });
    }
}

// ----------------------------------------------------------------------------
/// @brief True for the words of a PDDL formula that are not atoms: logic,
/// time, comparison, arithmetic and assignment.
// ----------------------------------------------------------------------------
static bool connective(std::string_view p_head)
{
    static constexpr std::string_view WORDS[] = {
        "and",    "or",       "not",        "imply",      "forall",
        "exists", "when",     "at",         "over",       "=",
        "<",      ">",        "<=",         ">=",         "+",
        "-",      "*",        "/",          "increase",   "decrease",
        "assign", "scale-up", "scale-down", "preference",
    };
    for (std::string_view const word : WORDS)
    {
        if (p_head == word)
        {
            return true;
        }
    }
    return false;
}

/// @brief When a part of a formula applies.
enum class When
{
    Untimed, ///< Plain action: conditions at start, effects at end.
    Start,
    End,
    Always, ///< over all: conditions at both ends.
};

// ----------------------------------------------------------------------------
/// @brief Every atom named in a formula, at any depth: @c (p ?a b) and
/// @c (f ?a) inside a comparison or an assignment, sorted by the phase it
/// applies to. The variable lists of @c forall / @c exists are skipped.
// ----------------------------------------------------------------------------
static void collectAtoms(Sexpr const& p_node,
                         ActionSchema& p_action,
                         bool p_effect,
                         When p_when)
{
    if (!p_node.is_list || p_node.list.empty())
    {
        return;
    }
    std::string_view const head = p_node.head();
    // (at ?r ?room) is an atom; (at start φ) and (over all φ) are not.
    bool const timed =
        p_node.list.size() == 3u && !p_node.list[1].is_list &&
        (p_node.list[1].isAtom("start") || p_node.list[1].isAtom("end") ||
         p_node.list[1].isAtom("all"));
    bool const word =
        (head == "at" || head == "over") ? timed : connective(head);
    if (head.empty())
    {
        return;
    }
    if (!word)
    {
        Atom atom = atomOf(p_node);
        auto add = [&](ActionSchema::Phase& p_phase)
        { (p_effect ? p_phase.writes : p_phase.reads).push_back(atom); };
        switch (p_when)
        {
            case When::Untimed:
                add(p_effect ? p_action.end : p_action.start);
                break;
            case When::Start:
                add(p_action.start);
                break;
            case When::End:
                add(p_action.end);
                break;
            case When::Always:
                add(p_action.start);
                add(p_action.end);
                break;
        }
        return;
    }
    if (timed)
    {
        When const when = p_node.list[1].isAtom("start") ? When::Start
                          : p_node.list[1].isAtom("end") ? When::End
                                                         : When::Always;
        collectAtoms(p_node.list[2], p_action, p_effect, when);
        return;
    }
    std::size_t const first = head == "forall" || head == "exists" ? 2u : 1u;
    for (std::size_t i = first; i < p_node.list.size(); ++i)
    {
        collectAtoms(p_node.list[i], p_action, p_effect, p_when);
    }
}

// ----------------------------------------------------------------------------
/// @brief Value following @p_key in a flat list (@c :parameters (...)).
// ----------------------------------------------------------------------------
static Sexpr const* keyed(Sexpr const& p_node, std::string_view p_key)
{
    for (std::size_t i = 0u; i + 1u < p_node.list.size(); ++i)
    {
        if (p_node.list[i].isAtom(p_key))
        {
            return &p_node.list[i + 1u];
        }
    }
    return nullptr;
}

/// Drops leading and trailing whitespace.
static std::string trim(std::string_view p_text)
{
    std::size_t first = 0u;
    std::size_t last = p_text.size();
    while (first < last &&
           std::isspace(static_cast<unsigned char>(p_text[first])) != 0)
    {
        ++first;
    }
    while (last > first &&
           std::isspace(static_cast<unsigned char>(p_text[last - 1u])) != 0)
    {
        --last;
    }
    return std::string(p_text.substr(first, last - first));
}

/// True when a PDDL name starts with a letter and stays a token.
static bool validName(std::string_view p_name)
{
    if (p_name.empty() ||
        std::isalpha(static_cast<unsigned char>(p_name.front())) == 0)
    {
        return false;
    }
    return std::all_of(p_name.begin(),
                       p_name.end(),
                       [](char p_c)
                       {
                           return std::isalnum(
                                      static_cast<unsigned char>(p_c)) != 0 ||
                                  p_c == '_' || p_c == '-';
                       });
}

std::string lowerCase(std::string_view p_text)
{
    std::string result(p_text);
    std::transform(result.begin(),
                   result.end(),
                   result.begin(),
                   [](unsigned char p_c)
                   { return static_cast<char>(std::tolower(p_c)); });
    return result;
}

char const* toString(Truth p_truth)
{
    switch (p_truth)
    {
        case Truth::False:
            return "false";
        case Truth::True:
            return "true";
        case Truth::Unknown:
            return "unknown";
    }
    return "?";
}

// ----------------------------------------------------------------------------
std::string Atom::toPddl() const
{
    std::string text = "(" + predicate;
    for (std::string const& arg : args)
    {
        text += " " + arg;
    }
    return text + ")";
}

// ----------------------------------------------------------------------------
std::string Atom::toString() const
{
    if (args.empty())
    {
        return predicate;
    }
    std::string text = predicate + "(";
    for (std::size_t i = 0u; i < args.size(); ++i)
    {
        text += (i == 0u ? "" : ", ") + args[i];
    }
    return text + ")";
}

// ----------------------------------------------------------------------------
Literal Literal::parse(std::string_view p_text)
{
    std::string text = trim(p_text);
    Literal literal;
    if (text.starts_with("not ") || text.starts_with("not\t"))
    {
        literal.positive = false;
        text = trim(std::string_view(text).substr(4u));
    }
    else if (text.starts_with("!"))
    {
        literal.positive = false;
        text = trim(std::string_view(text).substr(1u));
    }
    std::size_t const open = text.find('(');
    std::string const name = trim(std::string_view(text).substr(0u, open));
    if (!validName(name))
    {
        throw std::runtime_error(
            "'" + std::string(p_text) +
            "' is not a literal such as in(red_cube, box)");
    }
    literal.atom.predicate = lowerCase(name);
    if (open == std::string::npos)
    {
        return literal;
    }
    if (text.back() != ')' || text.find('(', open + 1u) != std::string::npos)
    {
        throw std::runtime_error(
            "'" + std::string(p_text) +
            "' is not a literal such as in(red_cube, box)");
    }
    std::string_view const inside =
        std::string_view(text).substr(open + 1u, text.size() - open - 2u);
    if (!trim(inside).empty())
    {
        std::size_t start = 0u;
        for (;;)
        {
            std::size_t const comma = inside.find(',', start);
            std::string const arg = trim(inside.substr(start, comma - start));
            if (!validName(arg))
            {
                throw std::runtime_error("'" + std::string(p_text) +
                                         "': bad argument '" + arg + "'");
            }
            literal.atom.args.push_back(lowerCase(arg));
            if (comma == std::string_view::npos)
            {
                break;
            }
            start = comma + 1u;
        }
    }
    return literal;
}

// ----------------------------------------------------------------------------
std::string Literal::toPddl() const
{
    return positive ? atom.toPddl() : "(not " + atom.toPddl() + ")";
}

// ----------------------------------------------------------------------------
std::string Literal::toString() const
{
    return positive ? atom.toString() : "not " + atom.toString();
}

// ----------------------------------------------------------------------------
std::string GroundAction::toPddl() const
{
    return Atom{ name, args }.toPddl();
}

// ----------------------------------------------------------------------------
bool DomainSchema::isA(std::string_view p_type,
                       std::string_view p_ancestor) const
{
    std::string current(p_type);
    // The depth bound stops a cycle in a malformed hierarchy.
    for (std::size_t depth = 0u; depth <= types.size() + 1u; ++depth)
    {
        if (current == p_ancestor || p_ancestor == "object")
        {
            return true;
        }
        auto const parent = types.find(current);
        if (parent == types.end() || parent->second == current)
        {
            return false;
        }
        current = parent->second;
    }
    return false;
}

// ----------------------------------------------------------------------------
DomainSchema DomainSchema::parse(std::string_view p_text)
{
    Sexpr const root = Reader(p_text).read();
    if (root.head() != "define" || root.list.size() < 2u ||
        root.list[1].head() != "domain" || root.list[1].list.size() != 2u)
    {
        throw std::runtime_error(
            "PDDL: a domain starts with (define (domain <name>)");
    }
    DomainSchema schema;
    schema.name = root.list[1].list[1].atom;
    for (std::size_t i = 2u; i < root.list.size(); ++i)
    {
        Sexpr const& section = root.list[i];
        std::string_view const head = section.head();
        if (head == ":types")
        {
            for (Parameter const& type : typedList(section.list, 1u))
            {
                schema.types[type.name] = type.type;
            }
        }
        else if (head == ":constants")
        {
            for (Parameter const& constant : typedList(section.list, 1u))
            {
                schema.constants[constant.name] = constant.type;
            }
        }
        else if (head == ":predicates")
        {
            for (std::size_t k = 1u; k < section.list.size(); ++k)
            {
                Sexpr const& item = section.list[k];
                if (item.head().empty())
                {
                    continue;
                }
                PredicateSchema predicate;
                predicate.name = std::string(item.head());
                for (Parameter const& parameter : typedList(item.list, 1u))
                {
                    predicate.types.push_back(parameter.type);
                }
                schema.predicates[predicate.name] = std::move(predicate);
            }
        }
        else if (head == ":derived" && section.list.size() >= 2u)
        {
            Sexpr const& item = section.list[1];
            PredicateSchema& predicate =
                schema.predicates[std::string(item.head())];
            predicate.name = std::string(item.head());
            predicate.derived = true;
            if (predicate.types.empty())
            {
                for (Parameter const& parameter : typedList(item.list, 1u))
                {
                    predicate.types.push_back(parameter.type);
                }
            }
        }
        else if ((head == ":action" || head == ":durative-action") &&
                 section.list.size() >= 2u)
        {
            ActionSchema action;
            action.name = section.list[1].atom;
            action.durative = head == ":durative-action";
            if (Sexpr const* parameters = keyed(section, ":parameters"))
            {
                action.parameters = typedList(parameters->list, 0u);
            }
            if (Sexpr const* effect = keyed(section, ":effect"))
            {
                collectEffects(*effect, action);
                collectAtoms(*effect, action, true, When::Untimed);
            }
            for (std::string_view const key : { ":precondition", ":condition" })
            {
                if (Sexpr const* condition = keyed(section, key))
                {
                    collectAtoms(*condition, action, false, When::Untimed);
                }
            }
            schema.actions[action.name] = std::move(action);
        }
    }
    return schema;
}

} // namespace robotik
