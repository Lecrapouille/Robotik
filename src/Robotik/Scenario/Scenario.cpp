// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/Scenario.hpp"

#include "BlackThorn/Builder/Yaml.hpp"

#include <filesystem>
#include <numbers>
#include <stdexcept>

namespace robotik
{

namespace
{

double number(bt::YamlNode const& p_node, std::string_view p_key, double p_default)
{
    return p_node.child(p_key).asDouble().value_or(p_default);
}

double item(bt::YamlNode const& p_node, std::size_t p_index, double p_default)
{
    return p_node.child(p_index).asDouble().value_or(p_default);
}

std::string text(bt::YamlNode const& p_node, std::string_view p_key)
{
    bt::YamlNode const child = p_node.child(p_key);
    return child.valid() ? child.scalar() : std::string{};
}

Vector3 vector(bt::YamlNode const& p_node, Vector3 p_default)
{
    if (!p_node.isSeq() || p_node.size() != 3u)
    {
        return p_default;
    }
    return { item(p_node, 0u, p_default.x),
             item(p_node, 1u, p_default.y),
             item(p_node, 2u, p_default.z) };
}

std::array<double, 2> range(bt::YamlNode const& p_node)
{
    if (!p_node.isSeq() || p_node.size() != 2u)
    {
        return { 0.0, 0.0 };
    }
    return { item(p_node, 0u, 0.0), item(p_node, 1u, 0.0) };
}

std::vector<std::string> strings(bt::YamlNode const& p_node)
{
    std::vector<std::string> result;
    if (p_node.isSeq())
    {
        p_node.forEachSeq([&result](bt::YamlNode p_item)
                          { result.push_back(p_item.scalar()); });
    }
    else if (p_node.valid() && !p_node.scalar().empty())
    {
        result.push_back(p_node.scalar());
    }
    return result;
}

std::filesystem::path resolve(std::filesystem::path const& p_directory,
                              std::string const& p_file)
{
    return p_file.empty() ? std::filesystem::path{}
                          : (p_directory / p_file).lexically_normal();
}

Scenario::Camera camera(std::string_view p_name, bt::YamlNode const& p_node)
{
    std::string const type = text(p_node, "type");
    if (type != "camera" && type != "rgbd")
    {
        throw std::runtime_error("Sensor '" + std::string(p_name) +
                                 "': unknown type '" + type + "'");
    }
    Scenario::Camera camera{ std::string(p_name), {} };
    CameraConfig& config = camera.config;
    config.parent = text(p_node, "parent");
    config.mount.position = vector(p_node.child("position"), {});
    Vector3 const euler = vector(p_node.child("rpy"), {});
    bt::YamlNode const quaternion = p_node.child("quaternion");
    if (quaternion.isSeq() && quaternion.size() == 4u)
    {
        config.mount.rotation = Quaternion(item(quaternion, 0u, 1.0),
                                           item(quaternion, 1u, 0.0),
                                           item(quaternion, 2u, 0.0),
                                           item(quaternion, 3u, 0.0))
                                    .normalized();
    }
    else
    {
        config.mount.rotation = rpy(euler.x, euler.y, euler.z);
    }
    bt::YamlNode const resolution = p_node.child("resolution");
    double const fov = number(p_node, "fov", 70.0) * std::numbers::pi / 180.0;
    config.intrinsics = CameraIntrinsics::fromFov(
        static_cast<std::uint32_t>(item(resolution, 0u, 320.0)),
        static_cast<std::uint32_t>(item(resolution, 1u, 240.0)),
        Radians(fov));
    config.frequency = number(p_node, "frequency", 30.0);
    config.depth = type == "rgbd";
    config.noise = number(p_node, "noise", 0.0);
    return camera;
}

Scenario::Actuator actuator(std::string_view p_name, bt::YamlNode const& p_node)
{
    Scenario::Actuator actuator;
    actuator.name = p_name;
    std::string const type = text(p_node, "type");
    if (type == "joint_group")
    {
        actuator.type = Scenario::Actuator::Type::JointGroup;
        actuator.joints = strings(p_node.child("joints"));
    }
    else if (type == "motor")
    {
        actuator.type = Scenario::Actuator::Type::Motor;
        actuator.joints = strings(p_node.child("joint"));
        if (actuator.joints.size() != 1u)
        {
            throw std::runtime_error("Motor '" + actuator.name +
                                     "' needs one joint");
        }
    }
    else if (type == "vacuum")
    {
        actuator.type = Scenario::Actuator::Type::Vacuum;
        actuator.link = text(p_node, "parent");
        actuator.length = Length(number(p_node, "length", 0.06));
    }
    else
    {
        throw std::runtime_error("Actuator '" + actuator.name +
                                 "': unknown type '" + type + "'");
    }
    return actuator;
}

Scenario::Object object(std::string_view p_name, bt::YamlNode const& p_node)
{
    Scenario::Object object;
    object.shape.name = p_name;
    std::string const type = text(p_node, "type");
    if (type == "box")
    {
        object.shape.type = ecs::SceneObject::Type::BOX;
    }
    else if (type != "cube")
    {
        throw std::runtime_error("Object '" + object.shape.name +
                                 "': unknown type '" + type + "'");
    }
    Vector3 const size = vector(p_node.child("size"), { 0.04, 0.04, 0.04 });
    object.shape.size = { Length(size.x), Length(size.y), Length(size.z) };
    Vector3 const color = vector(p_node.child("color"), { 0.8, 0.8, 0.8 });
    object.shape.color = { static_cast<float>(color.x),
                           static_cast<float>(color.y),
                           static_cast<float>(color.z) };
    object.position = vector(p_node.child("position"), {});
    bt::YamlNode const randomize = p_node.child("randomize");
    object.randomize = { range(randomize.child("x")),
                         range(randomize.child("y")),
                         range(randomize.child("z")) };
    return object;
}

void readFaults(Scenario& p_scenario, bt::YamlNode const& p_node)
{
    p_node.forEachSeq(
        [&p_scenario](bt::YamlNode p_fault)
        {
            std::string resource = text(p_fault, "resource");
            if (resource.empty())
            {
                throw std::runtime_error("A fault has no resource");
            }
            if (p_fault.hasKey("rate"))
            {
                p_scenario.random_faults.push_back(
                    { std::move(resource), number(p_fault, "rate", 0.0) });
                return;
            }
            std::string const action = text(p_fault, "action");
            if (action != "disable" && action != "restore")
            {
                throw std::runtime_error("Fault on '" + resource +
                                         "': action must be disable or restore");
            }
            p_scenario.faults.push_back({ Seconds(number(p_fault, "at", 0.0)),
                                          std::move(resource),
                                          action == "disable" });
        });
}

} // namespace

Scenario Scenario::load(std::filesystem::path const& p_path)
{
    auto parsed = bt::YamlDocument::parseFile(p_path.string());
    if (!parsed)
    {
        throw std::runtime_error("Cannot read scenario '" + p_path.string() +
                                 "': " + parsed.getError());
    }
    bt::YamlNode const root = parsed.getValue().root();
    std::filesystem::path const directory =
        std::filesystem::absolute(p_path).parent_path();

    Scenario scenario;
    scenario.name = text(root, "scenario");
    scenario.seed = static_cast<std::uint64_t>(number(root, "seed", 0.0));

    bt::YamlNode const robot = root.child("robot");
    scenario.robot_model = resolve(directory, text(robot, "model"));
    if (scenario.robot_model.empty())
    {
        throw std::runtime_error("Scenario '" + p_path.string() +
                                 "' has no robot.model");
    }
    if (robot.hasKey("home"))
    {
        robot.child("home").forEachMap(
            [&](std::string_view p_joint, bt::YamlNode p_value) {
                scenario.home[std::string(p_joint)] =
                    p_value.asDouble().value_or(0.0);
            });
    }
    if (robot.hasKey("sensors"))
    {
        robot.child("sensors").forEachMap(
            [&](std::string_view p_name, bt::YamlNode p_node)
            { scenario.cameras.push_back(camera(p_name, p_node)); });
    }
    if (robot.hasKey("actuators"))
    {
        robot.child("actuators").forEachMap(
            [&](std::string_view p_name, bt::YamlNode p_node)
            { scenario.actuators.push_back(actuator(p_name, p_node)); });
    }

    bt::YamlNode const world = root.child("world");
    if (world.hasKey("objects"))
    {
        world.child("objects").forEachMap(
            [&](std::string_view p_name, bt::YamlNode p_node)
            { scenario.objects.push_back(object(p_name, p_node)); });
    }

    if (root.hasKey("faults"))
    {
        readFaults(scenario, root.child("faults"));
    }

    bt::YamlNode const execute = root.child("execute");
    scenario.task = text(execute, "task");
    scenario.behavior_tree = resolve(directory, text(execute, "behavior_tree"));

    if (root.hasKey("assert"))
    {
        scenario.asserts = strings(root.child("assert"));
    }
    return scenario;
}

} // namespace robotik
