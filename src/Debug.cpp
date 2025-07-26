#include "Robotik/Debug.hpp"
#include "Robotik/private/Conversions.hpp"
#include "Robotik/private/Joint.hpp"
#include "Robotik/private/Link.hpp"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace robotik::debug
{

static void printNodes(const scene::Node& p_node,
                       size_t p_depth,
                       std::vector<bool>& p_is_last_at_depth,
                       bool p_detailed);

// ----------------------------------------------------------------------------
void printRobot(const Robot& p_robot, bool p_detailed)
{
    std::cout << "Robot: " << p_robot.name() << std::endl;
    if (!p_robot.hasRoot())
    {
        std::cout << "  No root node found!" << std::endl;
        return;
    }

    // Print the tree structure
    std::vector<bool> is_last_at_depth;
    printNodes(p_robot.root(), 0, is_last_at_depth, p_detailed);
}

// ----------------------------------------------------------------------------
static void printNodes(const scene::Node& p_node,
                       size_t p_depth,
                       std::vector<bool>& p_is_last_at_depth,
                       bool p_detailed)
{
    // Build indentation string
    std::string indent;
    for (size_t i = 0; i < p_depth; ++i)
    {
        if (i == p_depth - 1)
        {
            // Last level - use connector
            indent += p_is_last_at_depth[i] ? "└── " : "├── ";
        }
        else
        {
            // Intermediate levels
            indent += p_is_last_at_depth[i] ? "    " : "│   ";
        }
    }

    // Print the node
    // std::cout << indent << p_node.printName(p_detailed) << std::endl;

    // Print detailed information if requested
    if (p_detailed)
    {
        std::string detail_base;
        for (size_t i = 0; i < p_depth; ++i)
        {
            detail_base += p_is_last_at_depth[i] ? "    " : "│   ";
        }

        bool end_connector = p_node.children().empty();
        // std::cout << p_node.printDetails(end_connector) << std::endl;
    }

    // Process children
    const auto& children = p_node.children();
    for (size_t i = 0; i < children.size(); ++i)
    {
        bool is_last_child = (i == children.size() - 1);

        // Ensure vector is large enough
        if (p_depth >= p_is_last_at_depth.size())
        {
            p_is_last_at_depth.resize(p_depth + 1);
        }
        p_is_last_at_depth[p_depth] = is_last_child;

        printNodes(*children[i], p_depth + 1, p_is_last_at_depth, p_detailed);
    }
}

} // namespace robotik::debug