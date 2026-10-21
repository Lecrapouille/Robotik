// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/Workflow.hpp"

#include "Robotik/Common/Config.hpp"
#include "Robotik/Common/Must.hpp"

#include <stdexcept>
#include <string>

namespace robotik
{

/// A positive count (capacity, products at once).
static std::uint32_t count(ConfigNode const& p_node, char const* p_key, std::string const& p_where)
{
    double const value = p_node.number(p_key, 1.0);
    if (value < 1.0)
    {
        throw std::runtime_error(p_where + "." + p_key + " must be 1 or more");
    }
    return static_cast<std::uint32_t>(value);
}

/// Reads the operators and products, and checks the routes.
static WorkflowDescription readWorkflow(ConfigNode const& p_node)
{
    WorkflowDescription workflow;
    workflow.id = p_node.text("id");
    if (workflow.id.empty())
    {
        throw std::runtime_error("workflow.id is required");
    }
    for (ConfigNode const& item : p_node.at("operators").items())
    {
        WorkflowOperator op;
        op.id = item.text("id");
        if (op.id.empty())
        {
            throw std::runtime_error("every operator needs an id");
        }
        for (WorkflowOperator const& known : workflow.operators)
        {
            if (known.id == op.id)
            {
                throw std::runtime_error("operator '" + op.id + "' is declared twice");
            }
        }
        op.entity = item.text("entity");
        op.capacity = count(item, "capacity", "operator " + op.id);
        workflow.operators.push_back(std::move(op));
    }
    if (workflow.operators.empty())
    {
        throw std::runtime_error("workflow.operators: at least one operator");
    }

    for (ConfigNode const& item : p_node.at("products").items())
    {
        WorkflowProduct product;
        product.id = item.text("id");
        if (product.id.empty())
        {
            throw std::runtime_error("every product needs an id");
        }
        product.count = count(item, "count", "product " + product.id);
        std::size_t previous = 0U;
        for (ConfigNode const& step : item.at("route").items())
        {
            WorkflowOperation operation;
            operation.name = step.text("name");
            operation.on = step.text("on");
            operation.skill = step.text("skill");
            operation.duration = Seconds(step.number("duration", -1.0));
            std::string const where = "product " + product.id + ", operation '" +
                                      operation.name + "'";
            if (operation.name.empty() || operation.on.empty())
            {
                throw std::runtime_error("product " + product.id +
                                         ": every operation needs a name and 'on'");
            }
            if (operation.duration.value() < 0.0)
            {
                throw std::runtime_error(where + ": duration (s) is required");
            }
            std::size_t index = workflow.operators.size();
            for (std::size_t i = 0U; i < workflow.operators.size(); ++i)
            {
                if (workflow.operators[i].id == operation.on)
                {
                    index = i;
                }
            }
            if (index == workflow.operators.size())
            {
                throw std::runtime_error(where + ": unknown operator '" + operation.on + "'");
            }
            // A flowshop: the operators are visited in the order of their
            // list, each one once. Other routes (jobshop) are not handled yet.
            if (!product.route.empty() && index <= previous)
            {
                throw std::runtime_error(
                    where + ": operators must be visited in the order of workflow.operators, "
                            "each at most once (a flowshop)");
            }
            previous = index;
            product.route.push_back(std::move(operation));
        }
        if (product.route.empty())
        {
            throw std::runtime_error("product " + product.id + ": the route is empty");
        }
        workflow.products.push_back(std::move(product));
    }
    if (workflow.products.empty())
    {
        throw std::runtime_error("workflow.products: at least one product");
    }
    return workflow;
}

Result<WorkflowDescription> loadWorkflow(std::filesystem::path const& p_file)
{
    // The border of this reader: whatever stops it inside becomes a message.
    try
    {
        ConfigNode const root = must(ConfigNode::load(p_file));
        ConfigNode const& node = root.at("workflow");
        if (!node.isMap())
        {
            return failure(p_file.string() + ": no 'workflow:' section");
        }
        WorkflowDescription workflow = readWorkflow(node);
        workflow.file = p_file;
        return workflow;
    }
    catch (std::exception const& error)
    {
        return failure(p_file.string() + ": " + error.what());
    }
}

} // namespace robotik
