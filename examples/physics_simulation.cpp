/**
 * @file physics_simulation.cpp
 * @brief Example of using PhysicsSimulator to simulate robot dynamics
 *
 * This example demonstrates how to:
 * 1. Load a robot from a URDF file
 * 2. Create a physics simulator
 * 3. Configure joint properties (damping, friction, effort limits)
 * 4. Run a physics simulation loop
 * 5. Observe the effects of gravity on the robot
 */

#include <Robotik/Robotik.hpp>
#include <iomanip>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <path_to_urdf_file>"
                  << std::endl;
        return 1;
    }

    // ========================================================================
    // 1. Load the robot from URDF
    // ========================================================================
    std::string error;
    auto robot = robotik::loadRobot(argv[1], &error);

    if (!robot)
    {
        std::cerr << "Error loading robot: " << error << std::endl;
        return 1;
    }

    std::cout << "Robot loaded: " << robot->name() << std::endl;
    std::cout << "Number of joints: " << robot->jointNames().size()
              << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 2. Create a physics simulator
    // ========================================================================
    double dt = 0.01;                         // 10ms time step
    Eigen::Vector3d gravity(0.0, 0.0, -9.81); // Standard Earth gravity

    robotik::PhysicsSimulator simulator(dt, gravity);

    std::cout << "Physics simulator created:" << std::endl;
    std::cout << "  Time step: " << dt << " s" << std::endl;
    std::cout << "  Gravity: [" << gravity.transpose() << "] m/s²" << std::endl;
    std::cout << std::endl;

    // ========================================================================
    // 3. Configure joint properties
    // ========================================================================
    // Note: In a real URDF file, these would be loaded from the file.
    // Here we set them manually for demonstration.

    auto joint_names = robot->jointNames();
    for (const auto& name : joint_names)
    {
        auto& joint = const_cast<robotik::Joint&>(robot->joint(name));

        // Set physical properties
        joint.damping(0.1);     // Damping coefficient
        joint.friction(0.05);   // Friction coefficient
        joint.effort_max(10.0); // Maximum torque/force (N·m or N)

        std::cout << "Joint '" << name << "' configured:" << std::endl;
        std::cout << "  Damping: " << joint.damping() << std::endl;
        std::cout << "  Friction: " << joint.friction() << std::endl;
        std::cout << "  Max effort: " << joint.effort_max() << " N·m"
                  << std::endl;
    }
    std::cout << std::endl;

    // ========================================================================
    // 4. Set initial configuration
    // ========================================================================
    robot->setNeutralPosition(); // Start at neutral (all joints at 0)

    std::cout << "Initial joint positions:" << std::endl;
    auto positions = robot->jointPositions();
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        std::cout << "  " << joint_names[i] << ": " << std::fixed
                  << std::setprecision(3) << positions[i] << " rad"
                  << std::endl;
    }
    std::cout << std::endl;

    // ========================================================================
    // 5. Run simulation loop
    // ========================================================================
    std::cout << "Running simulation for 2 seconds..." << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    double simulation_time = 2.0; // 2 seconds
    int num_steps = static_cast<int>(simulation_time / dt);
    int print_interval = 50; // Print every 0.5 seconds

    for (int step = 0; step <= num_steps; ++step)
    {
        // Step the physics simulation
        simulator.step(*robot);

        // Print state periodically
        if (step % print_interval == 0)
        {
            double t = step * dt;
            std::cout << "Time: " << std::fixed << std::setprecision(2) << t
                      << " s" << std::endl;

            positions = robot->jointPositions();
            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                const auto& joint = robot->joint(joint_names[i]);
                std::cout << "  " << std::setw(15) << std::left
                          << joint_names[i] << " pos: " << std::setw(8)
                          << std::right << std::fixed << std::setprecision(3)
                          << positions[i] << " rad, vel: " << std::setw(8)
                          << std::right << std::fixed << std::setprecision(3)
                          << joint.velocity() << " rad/s, acc: " << std::setw(8)
                          << std::right << std::fixed << std::setprecision(3)
                          << joint.acceleration() << " rad/s²" << std::endl;
            }
            std::cout << std::endl;
        }
    }

    std::cout << std::string(60, '-') << std::endl;
    std::cout << "Simulation complete!" << std::endl;

    // ========================================================================
    // 6. Optional: Apply external forces/torques
    // ========================================================================
    std::cout << "\nExample: Applying external torque to first joint..."
              << std::endl;

    if (!joint_names.empty())
    {
        auto& joint = const_cast<robotik::Joint&>(robot->joint(joint_names[0]));
        joint.effort(5.0); // Apply 5 N·m torque

        std::cout << "Applied " << joint.effort() << " N·m to joint '"
                  << joint_names[0] << "'" << std::endl;

        // Simulate for 1 more second
        num_steps = static_cast<int>(1.0 / dt);
        for (int step = 0; step < num_steps; ++step)
        {
            simulator.step(*robot);
        }

        positions = robot->jointPositions();
        std::cout << "After 1 second with applied torque:" << std::endl;
        std::cout << "  Position: " << positions[0] << " rad" << std::endl;
        std::cout << "  Velocity: " << joint.velocity() << " rad/s"
                  << std::endl;
    }

    return 0;
}
