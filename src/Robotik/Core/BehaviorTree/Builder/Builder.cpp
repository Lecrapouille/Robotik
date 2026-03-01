/**
 * @file Builder.cpp
 * @brief Builder class for creating behavior trees from YAML.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/BehaviorTree/Builder/Builder.hpp"
#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"

#include <functional>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace robotik::bt
{

// ****************************************************************************
//! \brief Context structure for parsing, containing factory and blackboard
// ****************************************************************************
struct SubTreeRegistry
{
    std::unordered_map<std::string, YAML::Node> definitions;
};

struct ParsingContext
{
    NodeFactory const& factory;
    Blackboard::Ptr blackboard;
    SubTreeRegistry const* subtrees = nullptr;
    mutable uint32_t next_id = 1; // Auto-increment ID counter
};

// ----------------------------------------------------------------------------
//! \brief Assign ID to a node from YAML _id field or auto-generate
// ----------------------------------------------------------------------------
static void assignNodeId(Node& p_node,
                         ParsingContext const& p_context,
                         YAML::Node const& p_content)
{
    if (p_content["_id"])
    {
        p_node.setId(p_content["_id"].as<uint32_t>());
        // Update counter to avoid collisions with auto-generated IDs
        if (p_content["_id"].as<uint32_t>() >= p_context.next_id)
        {
            p_context.next_id = p_content["_id"].as<uint32_t>() + 1;
        }
    }
    else
    {
        p_node.setId(p_context.next_id++);
    }
}

using NodeCreatorMap = std::unordered_map<
    std::string,
    std::function<robotik::Return<Node::Ptr>(ParsingContext const&,
                                             YAML::Node const&)>>;

static robotik::Return<Node::Ptr>
parseYAMLNodeInternal(ParsingContext const& p_context,
                      YAML::Node const& p_node);

// ----------------------------------------------------------------------------
//! \brief Get the name of a node from YAML content
// ----------------------------------------------------------------------------
static std::string getNodeName(YAML::Node const& p_content)
{
    return p_content["name"] ? p_content["name"].as<std::string>()
                             : p_content.begin()->first.as<std::string>();
}

// ----------------------------------------------------------------------------
//! \brief Extract port remapping from YAML parameters section.
//! Converts YAML parameters to a map of port name -> blackboard key.
// ----------------------------------------------------------------------------
static std::unordered_map<std::string, std::string>
extractPortRemapping(YAML::Node const& p_parameters)
{
    std::unordered_map<std::string, std::string> remapping;
    if (p_parameters && p_parameters.IsMap())
    {
        for (auto const& param : p_parameters)
        {
            remapping[param.first.as<std::string>()] =
                param.second.as<std::string>();
        }
    }
    return remapping;
}

// ----------------------------------------------------------------------------
//! \brief Load only literal parameters into blackboard.
//! Skips ${...} references which are only used for port remapping.
// ----------------------------------------------------------------------------
static void loadLiteralParameters(Blackboard& p_bb,
                                  YAML::Node const& p_parameters)
{
    if (!p_parameters || !p_parameters.IsMap())
    {
        return;
    }

    std::regex refPattern(R"(\$\{([^}]+)\})");

    for (auto const& param : p_parameters)
    {
        auto const& valueNode = param.second;

        // Skip ${...} references - they're for port remapping only
        if (valueNode.IsScalar())
        {
            std::string value = valueNode.as<std::string>();
            if (std::regex_match(value, refPattern))
            {
                continue; // Skip reference
            }
        }

        // Load literal value into blackboard
        std::string key = param.first.as<std::string>();
        YAML::Node single;
        single[key] = valueNode;
        BlackboardSerializer::load(p_bb, single, &p_bb);
    }
}

// ----------------------------------------------------------------------------
//! \brief Parse children nodes from YAML content
// ----------------------------------------------------------------------------
static robotik::Return<std::vector<Node::Ptr>>
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
//! \brief Build the registry of reusable subtrees if provided in YAML input.
// ----------------------------------------------------------------------------
static robotik::Return<SubTreeRegistry>
buildSubTreeRegistry(YAML::Node const& p_root)
{
    SubTreeRegistry registry;

    if (!p_root["SubTrees"])
    {
        return robotik::Return<SubTreeRegistry>::success(std::move(registry));
    }

    if (!p_root["SubTrees"].IsMap())
    {
        return robotik::Return<SubTreeRegistry>::error(
            "'SubTrees' section must be a map of name -> node definitions");
    }

    for (auto const& entry : p_root["SubTrees"])
    {
        registry.definitions.emplace(entry.first.as<std::string>(),
                                     entry.second);
    }

    return robotik::Return<SubTreeRegistry>::success(std::move(registry));
}

// ----------------------------------------------------------------------------
//! \brief Static creator functions for each node type
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createSequence(ParsingContext const& p_context, YAML::Node const& p_content)
{
    auto node = Node::create<Sequence>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "children");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    for (auto& child : children.getValue())
    {
        node->addChild(std::move(child));
    }
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a selector node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createSelector(ParsingContext const& p_context, YAML::Node const& p_content)
{
    auto node = Node::create<Selector>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "children");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    for (auto& child : children.getValue())
    {
        node->addChild(std::move(child));
    }
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a parallel node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createParallel(ParsingContext const& p_context, YAML::Node const& p_content)
{
    bool has_policies = p_content["success_on_all"] || p_content["fail_on_all"];
    bool has_thresholds =
        p_content["success_threshold"] || p_content["failure_threshold"];

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
        bool success_on_all = p_content["success_on_all"]
                                  ? p_content["success_on_all"].as<bool>()
                                  : true;
        bool fail_on_all = p_content["fail_on_all"]
                               ? p_content["fail_on_all"].as<bool>()
                               : true;
        par = Node::create<ParallelAll>(success_on_all, fail_on_all);
    }
    else
    {
        size_t success_threshold =
            p_content["success_threshold"]
                ? p_content["success_threshold"].as<size_t>()
                : 1;
        size_t failure_threshold =
            p_content["failure_threshold"]
                ? p_content["failure_threshold"].as<size_t>()
                : 1;
        par = Node::create<Parallel>(success_threshold, failure_threshold);
    }

    par->name = getNodeName(p_content);
    assignNodeId(*par, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "children");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());

    auto* composite = reinterpret_cast<Composite*>(par.get());
    for (auto& child : children.getValue())
    {
        composite->addChild(std::move(child));
    }

    return robotik::Return<Node::Ptr>::success(std::move(par));
}

// ----------------------------------------------------------------------------
//! \brief Create an inverter node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createInverter(ParsingContext const& p_context, YAML::Node const& p_content)
{
    auto node = Node::create<Inverter>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
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

// ----------------------------------------------------------------------------
//! \brief Create a repeater node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createRepeater(ParsingContext const& p_context, YAML::Node const& p_content)
{
    size_t times = p_content["times"] ? p_content["times"].as<size_t>() : 0;
    auto node = Node::create<Repeater>(times);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);

    // Set blackboard for port access
    node->setBlackboard(p_context.blackboard);

    // Configure port remapping if parameters are present
    if (p_content["parameters"])
    {
        node->setPortRemapping(extractPortRemapping(p_content["parameters"]));
    }

    auto children = parseChildren(p_context, p_content, "child");
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

// ----------------------------------------------------------------------------
//! \brief Create a repeat until success node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createRepeatUntilSuccess(ParsingContext const& p_context,
                         YAML::Node const& p_content)
{
    size_t attempts =
        p_content["attempts"] ? p_content["attempts"].as<size_t>() : 0;
    auto node = Node::create<UntilSuccess>(attempts);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
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

// ----------------------------------------------------------------------------
//! \brief Create a repeat until failure node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createRepeatUntilFailure(ParsingContext const& p_context,
                         YAML::Node const& p_content)
{
    size_t attempts =
        p_content["attempts"] ? p_content["attempts"].as<size_t>() : 0;
    auto node = Node::create<UntilFailure>(attempts);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
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

// ----------------------------------------------------------------------------
//! \brief Create a force success node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createForceSuccess(ParsingContext const& p_context, YAML::Node const& p_content)
{
    auto node = Node::create<ForceSuccess>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
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

// ----------------------------------------------------------------------------
//! \brief Create a force failure node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createForceFailure(ParsingContext const& p_context, YAML::Node const& p_content)
{
    auto node = Node::create<ForceFailure>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
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

// ----------------------------------------------------------------------------
//! \brief Create an action node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createAction(ParsingContext const& p_context,
                                               YAML::Node const& p_content)
{
    if (!p_content["name"])
    {
        return robotik::Return<Node::Ptr>::error(
            p_content.begin()->first.as<std::string>() +
            " node missing 'name' field");
    }

    std::string name = p_content["name"].as<std::string>();

    auto node = p_context.factory.createNode(name);
    if (!node)
    {
        return robotik::Return<Node::Ptr>::error(
            "Failed to create " + p_content.begin()->first.as<std::string>() +
            " node: " + name);
    }

    // Set the blackboard for the node (now on Node base class)
    node->setBlackboard(p_context.blackboard);

    // Handle local parameters if present
    if (p_context.blackboard && p_content["parameters"])
    {
        // Load only literal parameters into blackboard (not ${...} references)
        // References are only used for port remapping, not stored in BB
        loadLiteralParameters(*p_context.blackboard, p_content["parameters"]);

        // Configure port remapping for all parameters
        node->setPortRemapping(extractPortRemapping(p_content["parameters"]));
    }

    node->name = name;
    assignNodeId(*node, p_context, p_content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a condition node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createCondition(ParsingContext const& p_context, YAML::Node const& p_content)
{
    return createAction(p_context, p_content);
}

// ----------------------------------------------------------------------------
//! \brief Create a success node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createSuccess(ParsingContext const& p_context,
                                                YAML::Node const& p_content)
{
    auto node = Node::create<Success>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a failure node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createFailure(ParsingContext const& p_context,
                                                YAML::Node const& p_content)
{
    auto node = Node::create<Failure>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Apply port remapping from parent to child blackboard.
//! For inputs: copies value from parent BB to child BB under remapped key.
//! For outputs: stores remapping info for later propagation.
// ----------------------------------------------------------------------------
static void applySubTreePortRemapping(
    YAML::Node const& p_parameters,
    Blackboard::Ptr const& p_parentBB,
    Blackboard::Ptr const& p_childBB,
    std::unordered_map<std::string, std::string>& p_outputRemapping)
{
    if (!p_parameters || !p_parameters.IsMap())
    {
        return;
    }

    std::regex pattern(R"(\$\{([^}]+)\})");

    for (auto const& param : p_parameters)
    {
        std::string childKey = param.first.as<std::string>();
        std::string value = param.second.as<std::string>();

        std::smatch match;
        if (std::regex_match(value, match, pattern))
        {
            // It's a reference ${parent_key}
            std::string parentKey = match[1].str();

            // Try to get value from parent blackboard and copy to child
            if (auto raw = p_parentBB->raw(parentKey); raw)
            {
                // Input: copy value from parent to child under childKey
                p_childBB->setRaw(childKey, *raw);
            }
            else
            {
                // Output: parent key doesn't exist yet, store remapping
                // The child will write to childKey, we need to propagate to
                // parentKey
                p_outputRemapping[childKey] = parentKey;
            }
        }
        else
        {
            // Literal value, set directly in child blackboard
            p_childBB->set(childKey, value);
        }
    }
}

// ----------------------------------------------------------------------------
//! \brief Create a subtree node referencing another behavior tree
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createSubTree(ParsingContext const& p_context,
                                                YAML::Node const& p_content)
{
    if (!p_context.subtrees)
    {
        return robotik::Return<Node::Ptr>::error(
            "SubTree node encountered but no 'SubTrees' section was provided");
    }

    if (!p_content["reference"])
    {
        return robotik::Return<Node::Ptr>::error(
            "SubTree node missing 'reference' field");
    }

    auto reference = p_content["reference"].as<std::string>();
    auto it = p_context.subtrees->definitions.find(reference);
    if (it == p_context.subtrees->definitions.end())
    {
        return robotik::Return<Node::Ptr>::error("Unknown subtree reference: " +
                                                 reference);
    }

    ParsingContext nested = p_context;
    if (p_context.blackboard)
    {
        nested.blackboard = p_context.blackboard->createChild();
    }
    else
    {
        nested.blackboard = std::make_shared<Blackboard>();
    }

    // Apply port remapping from parameters
    std::unordered_map<std::string, std::string> outputRemapping;
    std::unordered_map<std::string, std::string> allRemapping;
    if (p_content["parameters"] && p_context.blackboard)
    {
        applySubTreePortRemapping(p_content["parameters"],
                                  p_context.blackboard,
                                  nested.blackboard,
                                  outputRemapping);

        // Extract all remappings for display
        allRemapping = extractPortRemapping(p_content["parameters"]);
    }

    // Store port remapping info in the child blackboard for dump()
    if (!allRemapping.empty())
    {
        nested.blackboard->setPortRemapping(allRemapping);
    }

    auto subtreeRoot = parseYAMLNodeInternal(nested, it->second);
    if (!subtreeRoot)
    {
        return robotik::Return<Node::Ptr>::error(
            "Failed to instantiate subtree '" + reference +
            "': " + subtreeRoot.getError());
    }

    auto subtree = Tree::create();
    subtree->setBlackboard(nested.blackboard);
    subtree->setRoot(subtreeRoot.moveValue());

    // Store output remapping and parent blackboard for later propagation
    if (!outputRemapping.empty())
    {
        subtree->setOutputRemapping(outputRemapping);
        subtree->setParentBlackboard(p_context.blackboard);
    }

    auto handle =
        std::make_shared<SubTreeHandle>(reference, std::move(subtree));

    auto node = Node::create<SubTreeNode>(handle);
    node->name =
        p_content["name"] ? p_content["name"].as<std::string>() : reference;
    assignNodeId(*node, p_context, p_content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a timeout decorator node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createTimeout(ParsingContext const& p_context,
                                                YAML::Node const& p_content)
{
    size_t ms = p_content["milliseconds"]
                    ? p_content["milliseconds"].as<size_t>()
                    : 1000;
    auto node = Node::create<Timeout>(ms);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);

    // Set blackboard for port access
    node->setBlackboard(p_context.blackboard);

    // Configure port remapping if parameters are present
    if (p_content["parameters"])
    {
        node->setPortRemapping(extractPortRemapping(p_content["parameters"]));
    }

    auto children = parseChildren(p_context, p_content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Timeout must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a delay decorator node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createDelay(ParsingContext const& p_context,
                                              YAML::Node const& p_content)
{
    size_t ms = p_content["milliseconds"]
                    ? p_content["milliseconds"].as<size_t>()
                    : 1000;
    auto node = Node::create<Delay>(ms);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Delay must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a cooldown decorator node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createCooldown(ParsingContext const& p_context, YAML::Node const& p_content)
{
    size_t ms = p_content["milliseconds"]
                    ? p_content["milliseconds"].as<size_t>()
                    : 1000;
    auto node = Node::create<Cooldown>(ms);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "Cooldown must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a run once decorator node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createRunOnce(ParsingContext const& p_context,
                                                YAML::Node const& p_content)
{
    auto node = Node::create<RunOnce>();
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    auto children = parseChildren(p_context, p_content, "child");
    if (!children)
        return robotik::Return<Node::Ptr>::error(children.getError());
    if (children.getValue().size() != 1)
    {
        return robotik::Return<Node::Ptr>::error(
            "RunOnce must have exactly one child");
    }
    node->setChild(std::move(children.getValue()[0]));
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a wait leaf node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr> createWait(ParsingContext const& p_context,
                                             YAML::Node const& p_content)
{
    size_t ms = p_content["milliseconds"]
                    ? p_content["milliseconds"].as<size_t>()
                    : 1000;
    auto node = Node::create<Wait>(ms);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Create a set blackboard leaf node
// ----------------------------------------------------------------------------
static robotik::Return<Node::Ptr>
createSetBlackboard(ParsingContext const& p_context,
                    YAML::Node const& p_content)
{
    if (!p_content["key"])
    {
        return robotik::Return<Node::Ptr>::error(
            "SetBlackboard node missing 'key' field");
    }
    if (!p_content["value"])
    {
        return robotik::Return<Node::Ptr>::error(
            "SetBlackboard node missing 'value' field");
    }

    std::string key = p_content["key"].as<std::string>();
    std::string value = p_content["value"].as<std::string>();
    auto node = Node::create<SetBlackboard>(key, value, p_context.blackboard);
    node->name = getNodeName(p_content);
    assignNodeId(*node, p_context, p_content);
    return robotik::Return<Node::Ptr>::success(std::move(node));
}

// ----------------------------------------------------------------------------
//! \brief Get the node creators registry
// ----------------------------------------------------------------------------
static NodeCreatorMap& getNodeCreators()
{
    static NodeCreatorMap creators = {
        { Sequence::toString(), createSequence },
        { Selector::toString(), createSelector },
        { Parallel::toString(), createParallel },
        { Inverter::toString(), createInverter },
        { Repeater::toString(), createRepeater },
        { "Repeat", createRepeater }, // Backward compatibility alias
        { UntilSuccess::toString(), createRepeatUntilSuccess },
        { UntilFailure::toString(), createRepeatUntilFailure },
        { ForceSuccess::toString(), createForceSuccess },
        { ForceFailure::toString(), createForceFailure },
        { Timeout::toString(), createTimeout },
        { Delay::toString(), createDelay },
        { Cooldown::toString(), createCooldown },
        { RunOnce::toString(), createRunOnce },
        { Action::toString(), createAction },
        { Condition::toString(), createCondition },
        { Success::toString(), createSuccess },
        { Failure::toString(), createFailure },
        { Wait::toString(), createWait },
        { SetBlackboard::toString(), createSetBlackboard },
        { SubTreeNode::toString(), createSubTree },
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

        Blackboard::Ptr blackboard =
            p_blackboard ? p_blackboard : std::make_shared<Blackboard>();

        if (root["Blackboard"])
        {
            BlackboardSerializer::load(
                *blackboard, root["Blackboard"], blackboard.get());
        }

        auto registryResult = buildSubTreeRegistry(root);
        if (!registryResult)
        {
            return robotik::Return<Tree::Ptr>::error(registryResult.getError());
        }
        auto registry = registryResult.moveValue();
        SubTreeRegistry const* registryPtr =
            registry.definitions.empty() ? nullptr : &registry;

        auto nodeResult = parseYAMLNode(
            p_factory, root["BehaviorTree"], blackboard, registryPtr);
        if (!nodeResult)
        {
            return robotik::Return<Tree::Ptr>::error(nodeResult.getError());
        }

        Tree::Ptr tree = Tree::create();
        tree->setBlackboard(blackboard);
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

        Blackboard::Ptr blackboard =
            p_blackboard ? p_blackboard : std::make_shared<Blackboard>();

        if (root["Blackboard"])
        {
            BlackboardSerializer::load(
                *blackboard, root["Blackboard"], blackboard.get());
        }

        auto registryResult = buildSubTreeRegistry(root);
        if (!registryResult)
        {
            return robotik::Return<Tree::Ptr>::error(registryResult.getError());
        }
        auto registry = registryResult.moveValue();
        SubTreeRegistry const* registryPtr =
            registry.definitions.empty() ? nullptr : &registry;

        auto nodeResult = parseYAMLNode(
            p_factory, root["BehaviorTree"], blackboard, registryPtr);
        if (!nodeResult)
        {
            return robotik::Return<Tree::Ptr>::error(nodeResult.getError());
        }

        auto tree = Tree::create();
        tree->setBlackboard(blackboard);
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
static robotik::Return<Node::Ptr>
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
// Private version with blackboard/subtree support
robotik::Return<Node::Ptr>
Builder::parseYAMLNode(NodeFactory const& p_factory,
                       YAML::Node const& p_node,
                       Blackboard::Ptr p_blackboard,
                       SubTreeRegistry const* p_subtrees)
{
    ParsingContext context{ p_factory, p_blackboard, p_subtrees };
    return parseYAMLNodeInternal(context, p_node);
}

//-----------------------------------------------------------------------------
// Public version for backward compatibility
robotik::Return<Node::Ptr> Builder::parseYAMLNode(NodeFactory const& p_factory,
                                                  YAML::Node const& p_node)
{
    return parseYAMLNode(p_factory, p_node, nullptr, nullptr);
}

} // namespace robotik::bt
