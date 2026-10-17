// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/World.hpp"

#include "Robotik/Planning/Pddl.hpp"
#include "Robotik/Scenario/Config.hpp"
#include "Robotik/Scenario/PackagePath.hpp"

#include <algorithm>
#include <numbers>
#include <stdexcept>

namespace robotik
{

namespace
{

//! The robot flange and the tool link that lands on it (see Tools.md).
constexpr char const* FLANGE = "flange";
constexpr char const* TOOL_MOUNT = "tool_mount";

[[noreturn]] void fail(std::string const& p_where, std::string const& p_what)
{
    throw std::runtime_error(p_where + ": " + p_what);
}

//! A noise channel: a standard deviation, or { sigma, bias, resolution,
//! saturation: [min, max] }.
NoiseModel noise(ConfigNode const& p_node, std::string_view p_key)
{
    ConfigNode const& node = p_node.at(p_key);
    NoiseModel model;
    if (node.isNull())
    {
        return model;
    }
    if (node.isScalar())
    {
        model.sigma = p_node.number(p_key, 0.0);
        return model;
    }
    model.sigma = node.number("sigma", 0.0);
    model.bias = node.number("bias", 0.0);
    model.resolution = node.number("resolution", 0.0);
    if (node.has("saturation"))
    {
        std::array<double, 2> const bounds = node.range("saturation");
        model.min = bounds[0];
        model.max = bounds[1];
    }
    if (model.sigma < 0.0 || model.resolution < 0.0)
    {
        throw std::runtime_error("'" + std::string(p_key) +
                                 "': sigma and resolution must not be negative");
    }
    return model;
}

CameraConfig cameraConfig(std::string const& p_where, ConfigNode const& p_node, bool p_depth)
{
    CameraConfig config;
    config.parent = p_node.text("parent");
    // Optical frame (z forward, x right, y down) in the parent link.
    config.mount = p_node.asPose();
    ConfigNode const& resolution = p_node.at("resolution");
    double width = 320.0;
    double height = 240.0;
    if (!resolution.isNull())
    {
        if (!resolution.isSequence() || resolution.items().size() != 2u ||
            !resolution.items()[0].asNumber() || !resolution.items()[1].asNumber())
        {
            fail(p_where, "resolution must be [width, height]");
        }
        width = *resolution.items()[0].asNumber();
        height = *resolution.items()[1].asNumber();
    }
    double const fov = p_node.number("fov", 70.0) * std::numbers::pi / 180.0;
    config.intrinsics = CameraIntrinsics::fromFov(static_cast<std::uint32_t>(width),
                                                  static_cast<std::uint32_t>(height),
                                                  Radians(fov));
    config.frequency = p_node.number("frequency", 30.0);
    config.depth = p_depth;
    config.noise = p_node.number("noise", 0.0);
    return config;
}

SensorSpec sensor(std::string const& p_where,
                  std::string const& p_name,
                  ConfigNode const& p_node)
{
    std::string const where = p_where + ", sensor '" + p_name + "'";
    std::string const type = p_node.text("type");
    try
    {
        SensorSpec spec;
        spec.name = p_name;
        Seconds const latency(p_node.number("latency", 0.0));
        if (latency.value() < 0.0)
        {
            fail(where, "latency must not be negative");
        }
        if (type == "camera" || type == "rgbd")
        {
            if (p_node.has("latency"))
            {
                fail(where, "cameras have no latency model");
            }
            spec.type = SensorSpec::Type::Camera;
            spec.camera = cameraConfig(where, p_node, type == "rgbd");
        }
        else if (type == "imu")
        {
            spec.type = SensorSpec::Type::Imu;
            spec.imu.parent = p_node.text("parent");
            spec.imu.mount = p_node.asPose();
            spec.imu.frequency = p_node.number("frequency", spec.imu.frequency);
            spec.imu.latency = latency;
            spec.imu.accelerometer = noise(p_node, "accelerometer");
            spec.imu.gyroscope = noise(p_node, "gyroscope");
        }
        else if (type == "range")
        {
            spec.type = SensorSpec::Type::Range;
            RangeScannerConfig& range = spec.range;
            range.parent = p_node.text("parent");
            range.mount = p_node.asPose();
            range.beams = static_cast<std::uint32_t>(p_node.number("beams", range.beams));
            range.field_of_view =
                Radians(p_node.number("fov", 360.0) * std::numbers::pi / 180.0);
            range.max_range = Length(p_node.number("max_range", range.max_range.value()));
            range.frequency = p_node.number("frequency", range.frequency);
            range.latency = latency;
            range.noise = noise(p_node, "noise");
        }
        else if (type == "force_torque")
        {
            spec.type = SensorSpec::Type::ForceTorque;
            ForceTorqueConfig& force = spec.force_torque;
            force.link = p_node.text("link");
            if (force.link.empty())
            {
                fail(where, "a force/torque sensor needs the link it measures");
            }
            force.frequency = p_node.number("frequency", force.frequency);
            force.latency = latency;
            force.force = noise(p_node, "force");
            force.torque = noise(p_node, "torque");
        }
        else
        {
            fail(where, "unknown type '" + type + "' (camera, rgbd, imu, range, force_torque)");
        }
        return spec;
    }
    catch (std::runtime_error const& error)
    {
        if (std::string(error.what()).starts_with(where))
        {
            throw;
        }
        fail(where, error.what());
    }
}

ActuatorSpec actuator(std::string const& p_where,
                      std::string const& p_name,
                      ConfigNode const& p_node)
{
    std::string const where = p_where + ", actuator '" + p_name + "'";
    ActuatorSpec spec;
    spec.name = p_name;
    std::string const type = p_node.text("type");
    if (type == "joint_group")
    {
        spec.type = ActuatorSpec::Type::JointGroup;
        spec.joints = p_node.strings("joints");
    }
    else if (type == "motor")
    {
        spec.type = ActuatorSpec::Type::Motor;
        spec.joints = p_node.strings("joint");
        if (spec.joints.size() != 1u)
        {
            fail(where, "a motor needs one joint");
        }
    }
    else if (type == "vacuum")
    {
        spec.type = ActuatorSpec::Type::Vacuum;
        spec.link = p_node.text("parent");
        spec.length = Length(p_node.number("length", 0.0));
    }
    else
    {
        fail(where, "unknown type '" + type + "' (joint_group, motor, vacuum)");
    }
    return spec;
}

EntitySpec::Type entityType(std::string const& p_where, std::string const& p_text)
{
    if (p_text == "robot")
        return EntitySpec::Type::Robot;
    if (p_text == "tool")
        return EntitySpec::Type::Tool;
    if (p_text == "body")
        return EntitySpec::Type::Body;
    if (p_text == "prop")
        return EntitySpec::Type::Prop;
    fail(p_where, "type must be robot, tool, body or prop, not '" + p_text + "'");
}

ShapeSpec shape(std::string const& p_where, ConfigNode const& p_entity)
{
    ConfigNode const& node = p_entity.at("shape");
    ShapeSpec spec;
    std::string const type = node.text("type");
    if (type == "box")
    {
        spec.type = ShapeSpec::Type::Box;
    }
    else if (type == "container")
    {
        spec.type = ShapeSpec::Type::Container;
    }
    else
    {
        fail(p_where, "shape.type must be box or container, not '" + type + "'");
    }
    spec.size = node.vector3("size", spec.size);
    Vector3 const color = p_entity.vector3("color", { 0.8, 0.8, 0.8 });
    spec.color = { static_cast<float>(color.x),
                   static_cast<float>(color.y),
                   static_cast<float>(color.z) };
    return spec;
}

//! "easel.canvas_mount" -> { "easel", "canvas_mount" }; "easel" -> { "easel", "" }.
std::pair<std::string, std::string> splitFrame(std::string const& p_text)
{
    std::size_t const dot = p_text.find('.');
    if (dot == std::string::npos)
    {
        return { p_text, {} };
    }
    return { p_text.substr(0, dot), p_text.substr(dot + 1u) };
}

EntitySpec entity(std::string const& p_world,
                  ConfigNode const& p_node,
                  std::filesystem::path const& p_directory,
                  AssetResolver const& p_assets)
{
    EntitySpec spec;
    spec.id = p_node.text("id");
    if (spec.id.empty() || spec.id.find('.') != std::string::npos)
    {
        fail(p_world, "every entity needs an id without '.'");
    }
    std::string const where = p_world + ", entity '" + spec.id + "'";
    try
    {
        spec.type = entityType(where, p_node.text("type"));
        spec.kind = lowerCase(p_node.text("kind"));
        spec.model_ref = p_node.text("model");
        if (!spec.model_ref.empty())
        {
            // A file name (not a model id) stays in the package.
            bool const file = spec.model_ref.find('/') != std::string::npos ||
                              spec.model_ref.ends_with(".urdf");
            spec.model = file ? packageFile(p_directory, spec.model_ref)
                              : p_assets.model(spec.model_ref);
        }
        if (p_node.has("shape"))
        {
            spec.shape = shape(where, p_node);
        }
        if (spec.model.empty() == !spec.shape.has_value())
        {
            fail(where, "give either a model or a shape");
        }
        spec.pose = p_node.pose("pose");
        if (p_node.has("attach"))
        {
            ConfigNode const& node = p_node.at("attach");
            Attachment attach;
            std::tie(attach.parent, attach.parent_frame) = splitFrame(node.text("parent"));
            attach.frame = node.text("frame");
            attach.transform = node.pose("transform");
            if (attach.parent.empty())
            {
                fail(where, "attach.parent must name an entity (entity.frame)");
            }
            spec.attach = std::move(attach);
        }
        std::string const physics = p_node.text("physics");
        if (physics == "free")
        {
            spec.physics = EntitySpec::Physics::Free;
        }
        else if (physics == "fixed")
        {
            spec.physics = EntitySpec::Physics::Fixed;
        }
        else if (!physics.empty() && physics != "none")
        {
            fail(where, "physics must be none, fixed or free");
        }
        spec.mass = p_node.number("mass", spec.mass);
        spec.friction = p_node.number("friction", spec.friction);
        ConfigNode const& randomize = p_node.at("randomize");
        spec.randomize = { randomize.range("x"), randomize.range("y"), randomize.range("z") };

        ConfigNode const& home = p_node.at("home");
        for (auto const& [joint, value] : home.entries())
        {
            std::optional<double> const position = value.asNumber();
            if (!position)
            {
                fail(where, "home." + joint + " is not a number");
            }
            spec.home[joint] = *position;
        }
        for (auto const& [name, node] : p_node.at("sensors").entries())
        {
            spec.sensors.push_back(sensor(where, name, node));
        }
        for (auto const& [name, node] : p_node.at("actuators").entries())
        {
            spec.actuators.push_back(actuator(where, name, node));
        }
    }
    catch (std::runtime_error const& error)
    {
        if (std::string(error.what()).starts_with(where))
        {
            throw;
        }
        fail(where, error.what());
    }

    // --- Rules per type -----------------------------------------------------
    switch (spec.type)
    {
        case EntitySpec::Type::Robot:
        case EntitySpec::Type::Tool:
            if (spec.model.empty())
            {
                fail(where, "a robot or a tool needs a URDF model");
            }
            break;
        case EntitySpec::Type::Body:
            if (spec.physics == EntitySpec::Physics::None)
            {
                spec.physics = EntitySpec::Physics::Free;
            }
            if (spec.shape && spec.shape->type != ShapeSpec::Type::Box)
            {
                fail(where, "a body shape must be a box");
            }
            if (spec.shape && spec.physics == EntitySpec::Physics::Fixed)
            {
                fail(where, "a primitive body must be free");
            }
            break;
        case EntitySpec::Type::Prop:
            if (!spec.shape || spec.physics != EntitySpec::Physics::None)
            {
                fail(where, "a prop is a shape without physics");
            }
            break;
    }
    if (spec.type != EntitySpec::Type::Robot && spec.type != EntitySpec::Type::Tool &&
        (!spec.sensors.empty() || !spec.home.empty()))
    {
        fail(where, "only robots and tools carry sensors or a home posture");
    }
    return spec;
}

} // namespace

WorldDescription WorldDescription::read(ConfigNode const& p_world,
                                        std::filesystem::path const& p_directory,
                                        AssetResolver const& p_assets)
{
    WorldDescription world;
    world.id = p_world.text("id");
    std::string const where = "world '" + world.id + "'";
    if (world.id.empty())
    {
        fail("world", "an id is required");
    }
    ConfigNode const& floor = p_world.at("floor");
    if (floor.isMap())
    {
        world.floor = true;
        world.floor_friction = floor.number("friction", 1.0);
    }
    else if (!floor.isNull())
    {
        world.floor = p_world.flag("floor", false);
    }
    if (!p_world.at("entities").isSequence())
    {
        fail(where, "entities must be a list");
    }
    for (ConfigNode const& node : p_world.at("entities").items())
    {
        EntitySpec spec = entity(where, node, p_directory, p_assets);
        if (world.find(spec.id) != nullptr)
        {
            fail(where, "entity '" + spec.id + "' is declared twice");
        }
        world.entities.push_back(std::move(spec));
    }

    // --- Relations ------------------------------------------------------------
    long const robots = std::count_if(world.entities.begin(),
                                      world.entities.end(),
                                      [](EntitySpec const& p_entity)
                                      { return p_entity.type == EntitySpec::Type::Robot; });
    if (robots != 1)
    {
        fail(where, "exactly one entity of type robot is required");
    }
    std::string const& robot = world.robot().id;
    int mounted = 0;
    for (EntitySpec const& spec : world.entities)
    {
        if (!spec.attach)
        {
            continue;
        }
        std::string const entity = where + ", entity '" + spec.id + "'";
        EntitySpec const* parent = world.find(spec.attach->parent);
        if (parent == nullptr || parent == &spec)
        {
            fail(entity, "attach.parent '" + spec.attach->parent + "' is not another entity");
        }
        if (spec.type == EntitySpec::Type::Tool)
        {
            if (spec.attach->parent != robot || spec.attach->parent_frame != FLANGE ||
                spec.attach->frame != TOOL_MOUNT)
            {
                fail(entity, "a tool attaches with { parent: " + robot + ".flange, frame: tool_mount }");
            }
            ++mounted;
        }
        else if (spec.type == EntitySpec::Type::Robot)
        {
            fail(entity, "the robot cannot be attached; give it a pose");
        }
        else if (!spec.attach->frame.empty())
        {
            fail(entity, "only tools land a frame of their own on the parent");
        }
        else if (parent->type == EntitySpec::Type::Tool)
        {
            fail(entity, "cannot attach to a tool");
        }
        else if (parent->attach && parent->type != EntitySpec::Type::Robot)
        {
            if (EntitySpec const* grand = world.find(parent->attach->parent);
                grand != nullptr && grand->attach)
            {
                fail(entity, "attachments nest two levels at most");
            }
        }
    }
    if (mounted > 1)
    {
        fail(where, "only one tool can be mounted on the robot");
    }
    return world;
}

WorldDescription WorldDescription::load(std::filesystem::path const& p_path,
                                        AssetResolver const& p_assets)
{
    ConfigNode const file = ConfigNode::load(p_path);
    if (!file.has("world"))
    {
        throw std::runtime_error(p_path.string() + ": no 'world:' section");
    }
    return read(file.at("world"), packageRoot(p_path), p_assets);
}

EntitySpec const* WorldDescription::find(std::string_view p_id) const
{
    auto const found = std::find_if(entities.begin(),
                                    entities.end(),
                                    [p_id](EntitySpec const& p_entity)
                                    { return p_entity.id == p_id; });
    return found == entities.end() ? nullptr : &*found;
}

EntitySpec const& WorldDescription::robot() const
{
    for (EntitySpec const& spec : entities)
    {
        if (spec.type == EntitySpec::Type::Robot)
        {
            return spec;
        }
    }
    throw std::logic_error("World '" + id + "' has no robot");
}

EntitySpec const* WorldDescription::mountedTool() const
{
    for (EntitySpec const& spec : entities)
    {
        if (spec.type == EntitySpec::Type::Tool && spec.attach)
        {
            return &spec;
        }
    }
    return nullptr;
}

void WorldDescription::mount(std::string_view p_id)
{
    EntitySpec* chosen = nullptr;
    if (!p_id.empty())
    {
        for (EntitySpec& spec : entities)
        {
            if (spec.id == p_id && spec.type == EntitySpec::Type::Tool)
            {
                chosen = &spec;
            }
        }
        if (chosen == nullptr)
        {
            throw std::invalid_argument("World '" + id + "' has no tool '" +
                                        std::string(p_id) + "'");
        }
    }
    for (EntitySpec& spec : entities)
    {
        if (spec.type == EntitySpec::Type::Tool)
        {
            spec.attach.reset();
        }
    }
    if (chosen != nullptr)
    {
        chosen->attach = Attachment{ robot().id, FLANGE, TOOL_MOUNT, Pose{ zero3(), Quaternion{} } };
    }
}

} // namespace robotik
