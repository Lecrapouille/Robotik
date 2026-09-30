#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <iostream>
#include <string>

namespace
{

char const* label(robotik::Status p_status)
{
    switch (p_status)
    {
        case robotik::Status::Success:
            return "success";
        case robotik::Status::failure:
            return "failure";
        default:
            return "running";
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "usage: Robotik-Headless scenario.yml\n";
        return 1;
    }

    try
    {
        compages::world::World world;
        robotik::Simulation simulation(world, nullptr, robotik::Scenario::load(argv[1]));

        constexpr double kDt = 0.01;
        constexpr double kTimeout = 120.0;
        while (!simulation.finished() && simulation.runtime().time() < kTimeout)
        {
            simulation.step(kDt);
        }

        for (auto const& entry : simulation.trace().entries)
        {
            std::cout << "  " << entry.start << "s  " << entry.name << "  "
                      << label(entry.status) << "  (" << entry.end - entry.start << "s)\n";
        }
        bool passed = true;
        for (auto const& check : simulation.checks())
        {
            std::cout << (check.passed ? "[PASS] " : "[FAIL] ") << check.text << '\n';
            passed = passed && check.passed;
        }
        std::cout << "time=" << simulation.runtime().time() << "s\n";
        return passed ? 0 : 2;
    }
    catch (std::exception const& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
