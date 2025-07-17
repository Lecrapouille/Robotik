#include "Robotik/Debug.hpp"

#include <iostream>

namespace robotik::debug
{

// ----------------------------------------------------------------------------
void printRobot(const Robot& p_robot)
{
    size_t indentation = 0;

    std::cout << "Robot: " << p_robot.getName() << std::endl;
    p_robot.getRootNode()->traverse(
        [&indentation](Node const& p_node)
        {
            if (auto joint = dynamic_cast<Joint const*>(&p_node))
            {
                indentation++;
                std::cout << std::string(indentation, ' ');
                printJoint(*joint);
            }
        });
}

// ----------------------------------------------------------------------------
void printJoint(const Joint& p_joint)
{
    std::cout << "Joint: " << p_joint.getName() << std::endl;
}

// ----------------------------------------------------------------------------
void printLink(const Link& p_link)
{
    // std::cout << "Link: " << p_link.getName() << std::endl;
}

} // namespace robotik::debug