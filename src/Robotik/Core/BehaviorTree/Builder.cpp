/**
 * @file Builder.cpp
 * @brief Builder class for creating behavior trees from YAML.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/BehaviorTree/Builder.hpp"

#include <functional>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include <iostream>

namespace bt
{

// ****************************************************************************
/// \brief Context structure for parsing, containing factory and blackboard
// ****************************************************************************
struct ParsingContext
{
    NodeFactory const& factory;
    Blackboard::Ptr blackboard;
};

using NodeCreatorMap = std::unordered_map<
    std::string,
    std::function<robotik::Return<Node::Ptr>(ParsingContext const&,
                                             YAML::Node const&)>>;

// ----------------------------------------------------------------------------
//! \brief Resolve ${key} references in string values
// ----------------------------------------------------------------------------
bool resolveVariableReference(std::string const& p_value,
                              Blackboard::Ptr p_blackboard,
                              YAML::Node& p_resolved)
{
    // Check if value is a variable reference: ${key}
    if (p_value.size() > 3 && p_value[0] == '$' && p_value[1] == '{' &&
        p_value.back() == '}')
    {
        std::string key = p_value.substr(2, p_value.size() - 3);

        std::cout << "🔍 Resolving variable reference: ${" << key << "}"
                  << std::endl;

        // Try to get as vector<double>
        auto [vec, found_vec] = p_blackboard->get<std::vector<double>>(key);
        if (found_vec)
        {
            std::cout << "✅ Found vector with " << vec.size() << " elements"
                      << std::endl;
            // Convert vector to YAML sequence
            p_resolved = YAML::Node(YAML::NodeType::Sequence);
            for (double val : vec)
            {
                p_resolved.push_back(val);
            }
            return true;
        }

        // Try to get as double
        auto [d, found_double] = p_blackboard->get<double>(key);
        if (found_double)
        {
            p_resolved = d;
            return true;
        }

        // Try to get as int
        auto [i, found_int] = p_blackboard->get<int>(key);
        if (found_int)
        {
            p_resolved = i;
            return true;
        }

        // Try to get as string
        auto [s, found_string] = p_blackboard->get<std::string>(key);
        if (found_string)
        {
            p_resolved = s;
            return true;
        }

        // Variable not found
        std::cerr << "❌ Variable '${" << key << "}' not found in blackboard"
                  << std::endl;
        return false;
    }

    return false;
}

// ----------------------------------------------------------------------------
//! \brief Populate blackboard from YAML node
// ----------------------------------------------------------------------------
void populateBlackboard(Blackboard::Ptr p_blackboard,
                        YAML::Node const& p_params)
{
    if (!p_blackboard)
        return;

    std::cout << "📦 populateBlackboard called with " << p_params.size()
              << " parameters" << std::endl;

    for (auto const& param : p_params)
    {
        std::string key = param.first.as<std::string>();
        YAML::Node const& value = param.second;

        std::cout << "  Processing key: '" << key << "'" << std::endl;

        if (value.IsSequence())
        {
            // Handle arrays - convert to std::vector<double>
            std::vector<double> vec;
            for (auto const& item : value)
            {
                vec.push_back(item.as<double>());
            }
            p_blackboard->set(key, vec);
        }
        else if (value.IsScalar())
        {
            std::string str_value = value.as<std::string>();
            std::cout << "    Scalar value: '" << str_value << "'" << std::endl;

            // Check if it's a variable reference
            YAML::Node resolved;
            if (resolveVariableReference(str_value, p_blackboard, resolved))
            {
                // Resolved to another value - store it
                if (resolved.IsSequence())
                {
                    std::vector<double> vec;
                    for (auto const& item : resolved)
                    {
                        vec.push_back(item.as<double>());
                    }
                    p_blackboard->set(key, vec);
                }
                else if (resolved.IsScalar())
                {
                    try
                    {
                        double d = resolved.as<double>();
                        p_blackboard->set(key, d);
                    }
                    catch (...)
                    {
                        try
                        {
                            int i = resolved.as<int>();
                            p_blackboard->set(key, i);
                        }
                        catch (...)
                        {
                            std::string s = resolved.as<std::string>();
                            p_blackboard->set(key, s);
                        }
                    }
                }
            }
            else
            {
                // Not a variable reference - try different scalar types
                try
                {
                    // Try as double first
                    double d = value.as<double>();
                    p_blackboard->set(key, d);
                }
                catch (...)
                {
                    try
                    {
                        // Try as int
                        int i = value.as<int>();
                        p_blackboard->set(key, i);
                    }
                    catch (...)
                    {
                        try
                        {
                            // Try as bool
                            bool b = value.as<bool>();
                            p_blackboard->set(key, b);
                        }
                        catch (...)
                        {
                            // Default to string
                            p_blackboard->set(key, str_value);
                        }
                    }
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
//! \brief Get the name of a node from YAML content
// ----------------------------------------------------------------------------
std::string getNodeName(YAML::Node const& p_content)
{
    return p_content["name"] ? p_content["name"].as<std::string>()
                             : p_content.begin()->first.as<std::string>();
}

// ----------------------------------------------------------------------------
//! \brief Parse children nodes from YAML content
// ----------------------------------------------------------------------------
robotik::Return<std::vector<Node::Ptr>>
parseChildren(ParsingContext const& p_context,
              YAML::Node const& p_content,
              std::string const& p_field_name)
{
    if (!p_content[p_field_name])
    {
        return robotik::Return<std::vector<Node::Ptr>>::error(
            "Node '" + getNodeName(p_content) + "' missing '" + p_field_name +
            "' field");
    }

    if (!p_content[p_field_name].IsSequence())
    {
        return robotik::Return<std::vector<Node::Ptr>>::error(
            "Node '" + getNodeName(p_content) + "': '" + p_field_name +
            "' field must be a sequence");
    }

    if (p_content[p_field_name].size() == 0)
    {
        return robotik::Return<std::vector<Node::Ptr>>::error(
            "Node '" + getNodeName(p_content) +
            "' must have at least one child");
    }

    std::vector<Node::Ptr> children;
    for (auto const& child : p_content[p_field_name])
    {
        auto result = parseYAMLNodeInternal(p_context, child);
        if (!result)
        {
            return robotik::Return<std::vector<Node::Ptr>>::error(
                result.getError());
        }
        children.push_back(result.moveValue());
    }
    return robotik::Return<std::vector<Node::Ptr>>::success(
        std::move(children));
}

// ----------------------------------------------------------------------------
//! \brief Static creator functions for each node type
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createSequence(ParsingContext const& context,
                                                 YAML::Node const& content)
{
    auto node = Node::create<Sequence>();
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "children");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    for (auto& child : children.getValue())
    {
        node->addChild(std::move(child));
    }
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr> createSelector(ParsingContext const& context,
                                                 YAML::Node const& content)
{
    auto node = Node::create<Selector>();
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "children");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    for (auto& child : children.getValue())
    {
        node->addChild(std::move(child));
    }
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr> createParallel(ParsingContext const& context,
                                                 YAML::Node const& content)
{
    bool has_policies = content["success_on_all"] || content["fail_on_all"];
    bool has_thresholds =
        content["success_threshold"] || content["failure_threshold"];

    if (has_policies && has_thresholds)
    {
        return robotik::Return<Node::Ptr>::error(
            "Cannot specify both policies and thresholds");
    }
    if (!has_policies && !has_thresholds)
    {
        return robotik::Return<Node::Ptr>::error(
            "Missing policies or thresholds");
    }

    Node::Ptr par;
    if (has_policies)
    {
        bool success_on_all = content["success_on_all"]
                                  ? content["success_on_all"].as<bool>()
                                  : true;
        bool fail_on_all =
            content["fail_on_all"] ? content["fail_on_all"].as<bool>() : true;
        par = Node::create<ParallelAll>(success_on_all, fail_on_all);
    }
    else
    {
        size_t success_threshold =
            content["success_threshold"]
                ? content["success_threshold"].as<size_t>()
                : 1;
        size_t failure_threshold =
            content["failure_threshold"]
                ? content["failure_threshold"].as<size_t>()
                : 1;
        par = Node::create<Parallel>(success_threshold, failure_threshold);
    }

    par->name = getNodeName(content);
    auto children = parseChildren(context, content, "children");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());

    auto* composite = reinterpret_cast<Composite*>(par.get());
    for (auto& child : children.getValue())
    {
        composite->addChild(std::move(child));
    }

    return robotik::Return<Node::Ptr>::success(std::move(par));
}

static robotik::Return<Node::Ptr> createInverter(ParsingContext const& context,
                                                 YAML::Node const& content)
{
    auto node = Node::create<Inverter>();
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Decorator must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr> createRetry(ParsingContext const& context,
                                              YAML::Node const& content)
{
    size_t attempts =
        content["attempts"] ? content["attempts"].as<size_t>() : 3;
    auto node = Node::create<Retry>(attempts);
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Decorator must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr> createRepeat(ParsingContext const& context,
                                               YAML::Node const& content)
{
    size_t times = content["times"] ? content["times"].as<size_t>() : 0;
    auto node = Node::create<Repeat>(times);
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Decorator must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr>
createRepeatUntilSuccess(ParsingContext const& context,
                         YAML::Node const& content)
{
    auto node = Node::create<UntilSuccess>();
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Decorator must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr>
createRepeatUntilFailure(ParsingContext const& context,
                         YAML::Node const& content)
{
    auto node = Node::create<UntilFailure>();
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Decorator must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr>
createForceSuccess(ParsingContext const& context, YAML::Node const& content)
{
    auto node = Node::create<ForceSuccess>();
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Decorator must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr>
createForceFailure(ParsingContext const& context, YAML::Node const& content)
{
    auto node = Node::create<ForceFailure>();
    node->name = getNodeName(content);
    auto children = parseChildren(context, content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Decorator must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr> createAction(ParsingContext const& context,
                                               YAML::Node const& content)
{
    if (!content["name"])
    {
        return robotik::Return<Node::Ptr>::error(
            content.begin()->first.as<std::string>() +
            " node missing 'name' field");
    }

    std::string name = content["name"].as<std::string>();

    // Handle local parameters if present
    if (context.blackboard && content["parameters"])
    {
        populateBlackboard(context.blackboard, content["parameters"]);
    }

    auto node = context.factory.createNode(name);
    if (!node)
    {
        return robotik::Return<Node::Ptr>::error(
            "Failed to create " + content.begin()->first.as<std::string>() +
            " node: " + name);
    }

    node->name = name;
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr> createCondition(ParsingContext const& context,
                                                  YAML::Node const& content)
{
    return createAction(context, content);
}

static robotik::Return<Node::Ptr> createSuccess(ParsingContext const&,
                                                YAML::Node const& content)
{
    auto node = Node::create<Success>();
    node->name = getNodeName(content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

static robotik::Return<Node::Ptr> createFailure(ParsingContext const&,
                                                YAML::Node const& content)
{
    auto node = Node::create<Failure>();
    node->name = getNodeName(content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Get the node creators registry
// ----------------------------------------------------------------------------
NodeCreatorMap& getNodeCreators()
{
    static NodeCreatorMap creators = {
        { Sequence::toString(), createSequence },
        { Selector::toString(), createSelector },
        { Parallel::toString(), createParallel },
        { Inverter::toString(), createInverter },
        { Retry::toString(), createRetry },
        { Repeat::toString(), createRepeat },
        { UntilSuccess::toString(), createRepeatUntilSuccess },
        { UntilFailure::toString(), createRepeatUntilFailure },
        { ForceSuccess::toString(), createForceSuccess },
        { ForceFailure::toString(), createForceFailure },
        { Action::toString(), createAction },
        { Condition::toString(), createCondition },
        { Success::toString(), createSuccess },
        { Failure::toString(), createFailure },
    };
    return creators;
}

//-----------------------------------------------------------------------------
robotik::Return<Tree::Ptr> Builder::fromFile(NodeFactory const& p_factory,
                                             std::string const& p_file_path,
                                             Blackboard::Ptr p_blackboard)
{
    try
    {
        YAML::Node root = YAML::LoadFile(p_file_path);
        if (!root["BehaviorTree"])
        {
            return robotik::Return<Tree::Ptr>::error(
                "Missing 'BehaviorTree' node in YAML file");
        }

        // Handle global Blackboard section
        if (p_blackboard && root["Blackboard"])
        {
            populateBlackboard(p_blackboard, root["Blackboard"]);
        }

        auto nodeResult =
            parseYAMLNode(p_factory, root["BehaviorTree"], p_blackboard);
        if (!nodeResult)
        {
            return robotik::Return<Tree::Ptr>::error(nodeResult.getError());
        }

        Tree::Ptr tree = Tree::create();
        tree->setRoot(nodeResult.moveValue());
        return robotik::Return<Tree::Ptr>::success(std::move(tree));
    }
    catch (const YAML::Exception& e)
    {
        return robotik::Return<Tree::Ptr>::error("YAML parsing error: " +
                                                 std::string(e.what()));
    }
    catch (const std::exception& e)
    {
        return robotik::Return<Tree::Ptr>::error("Error: " +
                                                 std::string(e.what()));
    }
}

//-----------------------------------------------------------------------------
robotik::Return<Tree::Ptr> Builder::fromText(NodeFactory const& p_factory,
                                             std::string const& p_yaml_text,
                                             Blackboard::Ptr p_blackboard)
{
    try
    {
        YAML::Node root = YAML::Load(p_yaml_text);
        if (!root["BehaviorTree"])
        {
            return robotik::Return<Tree::Ptr>::error(
                "Missing 'BehaviorTree' node in YAML text");
        }

        // Handle global Blackboard section
        if (p_blackboard && root["Blackboard"])
        {
            populateBlackboard(p_blackboard, root["Blackboard"]);
        }

        auto nodeResult =
            parseYAMLNode(p_factory, root["BehaviorTree"], p_blackboard);
        if (!nodeResult)
        {
            return robotik::Return<Tree::Ptr>::error(nodeResult.getError());
        }

        auto tree = std::make_unique<Tree>();
        tree->setRoot(nodeResult.moveValue());
        return robotik::Return<Tree::Ptr>::success(std::move(tree));
    }
    catch (const YAML::Exception& e)
    {
        return robotik::Return<Tree::Ptr>::error("YAML parsing error: " +
                                                 std::string(e.what()));
    }
    catch (const std::exception& e)
    {
        return robotik::Return<Tree::Ptr>::error("Error: " +
                                                 std::string(e.what()));
    }
}

// ----------------------------------------------------------------------------
//! \brief Internal helper to parse YAML node with context
// ----------------------------------------------------------------------------
robotik::Return<Node::Ptr>
parseYAMLNodeInternal(ParsingContext const& p_context, YAML::Node const& p_node)
{
    if (!p_node.IsMap())
    {
        return robotik::Return<Node::Ptr>::error(
            "Invalid node format: must be a map");
    }

    auto it = p_node.begin();
    if (it == p_node.end())
    {
        return robotik::Return<Node::Ptr>::error(
            "Empty YAML node: a node must contain at least one key defining "
            "its type (e.g. Sequence, Selector, Action)");
    }

    std::string type = it->first.as<std::string>();
    auto const& creators = getNodeCreators();
    auto fn_it = creators.find(type);
    if (fn_it == creators.end())
    {
        return robotik::Return<Node::Ptr>::error("Unknown node type: " + type);
    }

    return fn_it->second(p_context, it->second);
}

//-----------------------------------------------------------------------------
// Private version with blackboard support
robotik::Return<Node::Ptr> Builder::parseYAMLNode(NodeFactory const& p_factory,
                                                  YAML::Node const& p_node,
                                                  Blackboard::Ptr p_blackboard)
{
    ParsingContext context{ p_factory, p_blackboard };
    return parseYAMLNodeInternal(context, p_node);
}

//-----------------------------------------------------------------------------
// Public version for backward compatibility
robotik::Return<Node::Ptr> Builder::parseYAMLNode(NodeFactory const& p_factory,
                                                  YAML::Node const& p_node)
{
    return parseYAMLNode(p_factory, p_node, nullptr);
}

} // namespace bt
