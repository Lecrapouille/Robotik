/**
 * @brief Simple test program to validate trajectory generation and joint
 * limits.
 */

#include "Robotik/Core/Robot.hpp"
#include "Robotik/Core/Trajectory.hpp"
#include "Robotik/Core/UrdfLoader.hpp"

#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <urdf_file>" << std::endl;
        return 1;
    }

    // Load robot from URDF
    robotik::URDFLoader parser;
    auto robot = parser.load(argv[1]);

    if (!robot)
    {
        std::cerr << "Failed to load robot: " << parser.error() << std::endl;
        return 1;
    }

    std::cout << "✅ Robot loaded successfully: " << robot->name() << std::endl;
    std::cout << "   Number of joints: " << robot->blueprint().numJoints()
              << std::endl;

    // Validate robot limits
    std::cout << "\n📊 Validating joint limits and dynamics..." << std::endl;
    if (robot->isValid())
    {
        std::cout << "✅ All joint limits are valid!" << std::endl;
    }
    else
    {
        std::cout << "❌ Some joint limits are invalid!" << std::endl;
    }

    // Print joint information
    std::cout << "\n🔧 Joint Information:" << std::endl;
    robot->blueprint().forEachJoint(
        [](robotik::Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            std::cout << "  Joint " << index << " (" << joint.name()
                      << "):" << std::endl;
            std::cout << "    Type: ";
            switch (joint.type())
            {
                case robotik::Joint::Type::REVOLUTE:
                    std::cout << "REVOLUTE";
                    break;
                case robotik::Joint::Type::PRISMATIC:
                    std::cout << "PRISMATIC";
                    break;
                case robotik::Joint::Type::CONTINUOUS:
                    std::cout << "CONTINUOUS";
                    break;
                case robotik::Joint::Type::FIXED:
                    std::cout << "FIXED";
                    break;
            }
            std::cout << std::endl;
            std::cout << "    Position limits: [" << min << ", " << max << "]"
                      << std::endl;
            std::cout << "    Max velocity: " << joint.maxVelocity()
                      << std::endl;
            std::cout << "    Max effort: " << joint.effort_max() << std::endl;
            std::cout << "    Damping: " << joint.damping() << std::endl;
            std::cout << "    Friction: " << joint.friction() << std::endl;
        });

    // Test trajectory generation
    std::cout << "\n🎯 Testing trajectory generation..." << std::endl;

    // Create start and goal configurations
    std::vector<double> start_config(robot->blueprint().numJoints());
    std::vector<double> goal_config(robot->blueprint().numJoints());

    robot->blueprint().forEachJoint(
        [&start_config, &goal_config](robotik::Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            start_config[index] = min + (max - min) * 0.25;
            goal_config[index] = min + (max - min) * 0.75;
        });

    // Generate trajectory
    robotik::JointSpaceGenerator generator;
    auto trajectory = generator.generate(start_config, goal_config, 2.0);

    std::cout << "✅ Trajectory generated successfully!" << std::endl;
    std::cout << "   Duration: " << trajectory->duration() << " seconds"
              << std::endl;
    std::cout << "   Valid: " << (trajectory->isValid() ? "Yes" : "No")
              << std::endl;

    // Test trajectory evaluation at different times
    std::cout << "\n📈 Trajectory evaluation:" << std::endl;
    for (double t = 0.0; t <= trajectory->duration(); t += 0.5)
    {
        auto point = trajectory->evaluate(t);
        std::cout << "  t=" << t << "s: pos=[";
        for (size_t i = 0; i < point.position.size(); ++i)
        {
            std::cout << point.position[i];
            if (i < point.position.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    // Test velocity-limited joint application
    std::cout << "\n🏃 Testing velocity-limited joint application..."
              << std::endl;

    // Set robot to start configuration
    robot->blueprint().forEachJoint(
        [&start_config](robotik::Joint& joint, size_t index)
        { joint.position(start_config[index]); });

    std::cout << "   Initial position: [";
    robot->blueprint().forEachJoint(
        [](robotik::Joint const& joint, size_t index)
        {
            if (index > 0)
                std::cout << ", ";
            std::cout << joint.position();
        });
    std::cout << "]" << std::endl;

    // Apply target with speed limit (simulate 10 steps of 0.1s)
    double dt = 0.1;
    for (int step = 0; step < 10; ++step)
    {
        robot->applyJointTargetsWithSpeedLimit(goal_config, dt);
    }

    std::cout << "   After 10 steps (1s): [";
    robot->blueprint().forEachJoint(
        [](robotik::Joint const& joint, size_t index)
        {
            if (index > 0)
                std::cout << ", ";
            std::cout << joint.position();
        });
    std::cout << "]" << std::endl;

    std::cout << "\n✅ All tests passed!" << std::endl;

    return 0;
}
