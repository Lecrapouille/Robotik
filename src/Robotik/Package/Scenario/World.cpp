// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Scenario/World.hpp"

#include "Robotik/Task/Planning/Pddl.hpp"
#include "Robotik/Common/Config.hpp"
#include "Robotik/Package/Scenario/PackagePath.hpp"

#include <algorithm>
#include <numbers>

namespace robotik
{

/// A world error prefixed with the place it was found.
static Failure fail(std::string const& p_where, std::string const& p_what)
{
    return failure(p_where + ": " + p_what);
}

/// @p_result, its error prefixed with @p_where unless it already names it.
template <typename T>
static Result<T> within(std::string const& p_where, Result<T> p_result)
{
    if (!p_result && !p_result.error().starts_with(p_where))
    {
        return fail(p_where, p_result.error());
    }
    return p_result;
}

/// A noise channel: a standard deviation, or { sigma, bias, resolution,
/// saturation: [min, max] }.
static Result<NoiseModel> noise(ConfigNode const& p_node,
                                std::string_view p_key)
{
    ConfigNode const& node = p_node.at(p_key);
    NoiseModel model;
    if (node.isNull())
    {
        return model;
    }
    if (node.isScalar())
    {
        ROBOTIK_TRY_ASSIGN(model.sigma, p_node.number(p_key, 0.0));
        return model;
    }
    ROBOTIK_TRY(
        node.checkKeys({ "sigma", "bias", "resolution", "saturation" }, p_key));
    ROBOTIK_TRY_ASSIGN(model.sigma, node.number("sigma", 0.0));
    ROBOTIK_TRY_ASSIGN(model.bias, node.number("bias", 0.0));
    ROBOTIK_TRY_ASSIGN(model.resolution, node.number("resolution", 0.0));
    if (node.has("saturation"))
    {
        std::array<double, 2> bounds{};
        ROBOTIK_TRY_ASSIGN(bounds, node.range("saturation"));
        model.min = bounds[0];
        model.max = bounds[1];
    }
    if (model.sigma < 0.0 || model.resolution < 0.0)
    {
        return failure("'" + std::string(p_key) +
                       "': sigma and resolution must not be negative");
    }
    return model;
}

/// Reads a camera mount, its resolution and its clip planes.
static Result<CameraConfig>
cameraConfig(std::string const& p_where, ConfigNode const& p_node, bool p_depth)
{
    CameraConfig config;
    config.parent = p_node.text("parent");
    // Optical frame (z forward, x right, y down) in the parent link.
    ROBOTIK_TRY_ASSIGN(config.mount, p_node.asPose());
    ConfigNode const& resolution = p_node.at("resolution");
    double width = 320.0;
    double height = 240.0;
    if (!resolution.isNull())
    {
        if (!resolution.isSequence() || resolution.items().size() != 2U ||
            !resolution.items()[0].asNumber() ||
            !resolution.items()[1].asNumber())
        {
            return fail(p_where, "resolution must be [width, height]");
        }
        width = *resolution.items()[0].asNumber();
        height = *resolution.items()[1].asNumber();
    }
    double fov = 70.0;
    ROBOTIK_TRY_ASSIGN(fov, p_node.number("fov", fov));
    fov *= std::numbers::pi / 180.0;
    config.intrinsics =
        CameraIntrinsics::fromFov(static_cast<std::uint32_t>(width),
                                  static_cast<std::uint32_t>(height),
                                  Radians(fov));
    ROBOTIK_TRY_ASSIGN(config.frequency, p_node.number("frequency", 30.0));
    config.depth = p_depth;
    ROBOTIK_TRY_ASSIGN(config.noise, p_node.number("noise", 0.0));
    return config;
}

/// Reads one sensor entry of an entity; errors name the sensor.
static Result<SensorSpec> readSensor(std::string const& p_where,
                                     std::string const& p_name,
                                     ConfigNode const& p_node)
{
    std::string const type = p_node.text("type");
    SensorSpec spec;
    spec.name = p_name;
    double latency_s = 0.0;
    ROBOTIK_TRY_ASSIGN(latency_s, p_node.number("latency", 0.0));
    Seconds const latency(latency_s);
    if (latency.value() < 0.0)
    {
        return fail(p_where, "latency must not be negative");
    }
    if (type == "camera" || type == "rgbd")
    {
        if (p_node.has("latency"))
        {
            return fail(p_where, "cameras have no latency model");
        }
        ROBOTIK_TRY(p_node.checkKeys({ "type",
                                       "parent",
                                       "position",
                                       "rpy",
                                       "quaternion",
                                       "resolution",
                                       "fov",
                                       "frequency",
                                       "noise",
                                       "latency" },
                                     p_where));
        spec.type = SensorSpec::Type::Camera;
        ROBOTIK_TRY_ASSIGN(spec.camera,
                           cameraConfig(p_where, p_node, type == "rgbd"));
    }
    else if (type == "imu")
    {
        ROBOTIK_TRY(p_node.checkKeys({ "type",
                                       "parent",
                                       "position",
                                       "rpy",
                                       "quaternion",
                                       "frequency",
                                       "latency",
                                       "accelerometer",
                                       "gyroscope" },
                                     p_where));
        spec.type = SensorSpec::Type::Imu;
        spec.imu.parent = p_node.text("parent");
        ROBOTIK_TRY_ASSIGN(spec.imu.mount, p_node.asPose());
        ROBOTIK_TRY_ASSIGN(spec.imu.frequency,
                           p_node.number("frequency", spec.imu.frequency));
        spec.imu.latency = latency;
        ROBOTIK_TRY_ASSIGN(spec.imu.accelerometer,
                           noise(p_node, "accelerometer"));
        ROBOTIK_TRY_ASSIGN(spec.imu.gyroscope, noise(p_node, "gyroscope"));
    }
    else if (type == "range")
    {
        ROBOTIK_TRY(p_node.checkKeys({ "type",
                                       "parent",
                                       "position",
                                       "rpy",
                                       "quaternion",
                                       "beams",
                                       "fov",
                                       "max_range",
                                       "frequency",
                                       "latency",
                                       "noise" },
                                     p_where));
        spec.type = SensorSpec::Type::Range;
        RangeScannerConfig& range = spec.range;
        range.parent = p_node.text("parent");
        ROBOTIK_TRY_ASSIGN(range.mount, p_node.asPose());
        double beams = 0.0;
        ROBOTIK_TRY_ASSIGN(beams, p_node.number("beams", range.beams));
        range.beams = static_cast<std::uint32_t>(beams);
        double fov = 360.0;
        ROBOTIK_TRY_ASSIGN(fov, p_node.number("fov", fov));
        range.field_of_view = Radians(fov * std::numbers::pi / 180.0);
        double max_range = range.max_range.value();
        ROBOTIK_TRY_ASSIGN(max_range, p_node.number("max_range", max_range));
        range.max_range = Length(max_range);
        ROBOTIK_TRY_ASSIGN(range.frequency,
                           p_node.number("frequency", range.frequency));
        range.latency = latency;
        ROBOTIK_TRY_ASSIGN(range.noise, noise(p_node, "noise"));
    }
    else if (type == "force_torque")
    {
        ROBOTIK_TRY(p_node.checkKeys(
            { "type", "link", "frequency", "latency", "force", "torque" },
            p_where));
        spec.type = SensorSpec::Type::ForceTorque;
        ForceTorqueConfig& force = spec.force_torque;
        force.link = p_node.text("link");
        if (force.link.empty())
        {
            return fail(p_where,
                        "a force/torque sensor needs the link it measures");
        }
        ROBOTIK_TRY_ASSIGN(force.frequency,
                           p_node.number("frequency", force.frequency));
        force.latency = latency;
        ROBOTIK_TRY_ASSIGN(force.force, noise(p_node, "force"));
        ROBOTIK_TRY_ASSIGN(force.torque, noise(p_node, "torque"));
    }
    else
    {
        return fail(p_where,
                    "unknown type '" + type +
                        "' (camera, rgbd, imu, range, force_torque)");
    }
    return spec;
}

/// Reads one sensor entry of an entity.
static Result<SensorSpec> sensor(std::string const& p_where,
                                 std::string const& p_name,
                                 ConfigNode const& p_node)
{
    std::string const where = p_where + ", sensor '" + p_name + "'";
    return within(where, readSensor(where, p_name, p_node));
}

/// Reads one actuator entry of an entity.
static Result<ActuatorSpec> actuator(std::string const& p_where,
                                     std::string const& p_name,
                                     ConfigNode const& p_node)
{
    std::string const where = p_where + ", actuator '" + p_name + "'";
    ActuatorSpec spec;
    spec.name = p_name;
    std::string const type = p_node.text("type");
    if (type == "joint_group")
    {
        ROBOTIK_TRY(p_node.checkKeys({ "type", "joints" }, where));
        spec.type = ActuatorSpec::Type::JointGroup;
        spec.joints = p_node.strings("joints");
    }
    else if (type == "motor")
    {
        ROBOTIK_TRY(p_node.checkKeys({ "type", "joint" }, where));
        spec.type = ActuatorSpec::Type::Motor;
        spec.joints = p_node.strings("joint");
        if (spec.joints.size() != 1U)
        {
            return fail(where, "a motor needs one joint");
        }
    }
    else if (type == "vacuum")
    {
        ROBOTIK_TRY(p_node.checkKeys({ "type", "parent", "length" }, where));
        spec.type = ActuatorSpec::Type::Vacuum;
        spec.link = p_node.text("parent");
        double length = 0.0;
        ROBOTIK_TRY_ASSIGN(length, within(where, p_node.number("length", 0.0)));
        spec.length = Length(length);
    }
    else
    {
        return fail(where,
                    "unknown type '" + type + "' (joint_group, motor, vacuum)");
    }
    return spec;
}

/// Maps the type text of an entity onto its enum.
static Result<EntitySpec::Type> entityType(std::string const& p_where,
                                           std::string const& p_text)
{
    if (p_text == "robot")
        return EntitySpec::Type::Robot;
    if (p_text == "tool")
        return EntitySpec::Type::Tool;
    if (p_text == "body")
        return EntitySpec::Type::Body;
    if (p_text == "prop")
        return EntitySpec::Type::Prop;
    return fail(p_where,
                "type must be robot, tool, body or prop, not '" + p_text + "'");
}

/// Reads the shape block of an entity.
static Result<ShapeSpec> shape(std::string const& p_where,
                               ConfigNode const& p_entity)
{
    ConfigNode const& node = p_entity.at("shape");
    ROBOTIK_TRY(node.checkKeys({ "type", "size" }, p_where + ", shape"));
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
        return fail(p_where,
                    "shape.type must be box or container, not '" + type + "'");
    }
    ROBOTIK_TRY_ASSIGN(spec.size, node.vector3("size", spec.size));
    Vector3 color{ 0.8, 0.8, 0.8 };
    ROBOTIK_TRY_ASSIGN(color, p_entity.vector3("color", color));
    spec.color = { static_cast<float>(color.x),
                   static_cast<float>(color.y),
                   static_cast<float>(color.z) };
    return spec;
}

/// "easel.canvas_mount" -> { "easel", "canvas_mount" }; "easel" -> { "easel",
/// "" }.
static std::pair<std::string, std::string> splitFrame(std::string const& p_text)
{
    std::size_t const dot = p_text.find('.');
    if (dot == std::string::npos)
    {
        return { p_text, {} };
    }
    return { p_text.substr(0, dot), p_text.substr(dot + 1U) };
}

/// Reads the fields of one entity; errors name the entity.
static Result<void> readEntity(EntitySpec& p_spec,
                               std::string const& p_where,
                               ConfigNode const& p_node,
                               std::filesystem::path const& p_directory,
                               AssetResolver const& p_assets)
{
    EntitySpec& spec = p_spec;
    std::string const& where = p_where;
    ROBOTIK_TRY(p_node.checkKeys({ "id",
                                   "type",
                                   "kind",
                                   "model",
                                   "shape",
                                   "color",
                                   "pose",
                                   "attach",
                                   "physics",
                                   "mass",
                                   "friction",
                                   "randomize",
                                   "home",
                                   "sensors",
                                   "actuators",
                                   "touches" },
                                 where));
    ROBOTIK_TRY_ASSIGN(spec.type, entityType(where, p_node.text("type")));
    spec.kind = lowerCase(p_node.text("kind"));
    spec.model_ref = p_node.text("model");
    if (!spec.model_ref.empty())
    {
        // A file name (not a model id) stays in the package.
        bool const file = spec.model_ref.find('/') != std::string::npos ||
                          spec.model_ref.ends_with(".urdf");
        if (file)
        {
            ROBOTIK_TRY_ASSIGN(spec.model,
                               packageFile(p_directory, spec.model_ref));
        }
        else
        {
            ROBOTIK_TRY_ASSIGN(spec.model, p_assets.model(spec.model_ref));
        }
    }
    if (p_node.has("shape"))
    {
        ROBOTIK_TRY_ASSIGN(spec.shape, shape(where, p_node));
    }
    if (spec.model.empty() == !spec.shape.has_value())
    {
        return fail(where, "give either a model or a shape");
    }
    ROBOTIK_TRY_ASSIGN(spec.pose, p_node.pose("pose"));
    if (p_node.has("attach"))
    {
        ConfigNode const& node = p_node.at("attach");
        ROBOTIK_TRY(
            node.checkKeys({ "parent", "frame", "transform" }, "attach"));
        Attachment attach;
        std::tie(attach.parent, attach.parent_frame) =
            splitFrame(node.text("parent"));
        attach.frame = node.text("frame");
        ROBOTIK_TRY_ASSIGN(attach.transform, node.pose("transform"));
        if (attach.parent.empty())
        {
            return fail(where,
                        "attach.parent must name an entity (entity.frame)");
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
        return fail(where, "physics must be none, fixed or free");
    }
    ROBOTIK_TRY_ASSIGN(spec.mass, p_node.number("mass", spec.mass));
    ROBOTIK_TRY_ASSIGN(spec.friction, p_node.number("friction", spec.friction));
    ConfigNode const& randomize = p_node.at("randomize");
    ROBOTIK_TRY(randomize.checkKeys({ "x", "y", "z" }, "randomize"));
    for (std::size_t axis = 0; axis < 3U; ++axis)
    {
        static constexpr std::array<char const*, 3> AXES = { "x", "y", "z" };
        ROBOTIK_TRY_ASSIGN(spec.randomize[axis], randomize.range(AXES[axis]));
    }

    ConfigNode const& home = p_node.at("home");
    for (auto const& [joint, value] : home.entries())
    {
        std::optional<double> const position = value.asNumber();
        if (!position)
        {
            return fail(where, "home." + joint + " is not a number");
        }
        spec.home[joint] = *position;
    }
    for (auto const& [name, node] : p_node.at("sensors").entries())
    {
        SensorSpec sensor_spec;
        ROBOTIK_TRY_ASSIGN(sensor_spec, sensor(where, name, node));
        spec.sensors.push_back(std::move(sensor_spec));
    }
    for (auto const& [name, node] : p_node.at("actuators").entries())
    {
        ActuatorSpec actuator_spec;
        ROBOTIK_TRY_ASSIGN(actuator_spec, actuator(where, name, node));
        spec.actuators.push_back(std::move(actuator_spec));
    }
    spec.touches = p_node.strings("touches");
    return success();
}

/// Reads one entity, including its model, shape and devices.
static Result<EntitySpec> entity(std::string const& p_world,
                                 ConfigNode const& p_node,
                                 std::filesystem::path const& p_directory,
                                 AssetResolver const& p_assets)
{
    EntitySpec spec;
    spec.id = p_node.text("id");
    if (spec.id.empty() || spec.id.find('.') != std::string::npos)
    {
        return fail(p_world, "every entity needs an id without '.'");
    }
    std::string const where = p_world + ", entity '" + spec.id + "'";
    ROBOTIK_TRY(
        within(where, readEntity(spec, where, p_node, p_directory, p_assets)));

    // A primitive body without a physics keyword falls freely.
    if (spec.type == EntitySpec::Type::Body &&
        spec.physics == EntitySpec::Physics::None)
    {
        spec.physics = EntitySpec::Physics::Free;
    }
    return spec;
}

Result<WorldDescription> readWorld(ConfigNode const& p_world,
                                   std::filesystem::path const& p_directory,
                                   AssetResolver const& p_assets)
{
    WorldDescription world;
    world.id = p_world.text("id");
    std::string const where = "world '" + world.id + "'";
    if (world.id.empty())
    {
        return fail("world", "an id is required");
    }
    ROBOTIK_TRY(p_world.checkKeys({ "id", "floor", "entities" }, where));
    ConfigNode const& floor = p_world.at("floor");
    if (floor.isMap())
    {
        ROBOTIK_TRY(floor.checkKeys({ "friction" }, where + ", floor"));
        world.floor = true;
        ROBOTIK_TRY_ASSIGN(world.floor_friction, floor.number("friction", 1.0));
    }
    else if (!floor.isNull())
    {
        ROBOTIK_TRY_ASSIGN(world.floor,
                           within(where, p_world.flag("floor", false)));
    }
    if (!p_world.at("entities").isSequence())
    {
        return fail(where, "entities must be a list");
    }
    for (ConfigNode const& node : p_world.at("entities").items())
    {
        EntitySpec spec;
        ROBOTIK_TRY_ASSIGN(spec, entity(where, node, p_directory, p_assets));
        world.entities.push_back(std::move(spec));
    }
    // The rules shared with a description written in C++.
    ROBOTIK_TRY(validate(world));
    return world;
}

Result<WorldDescription> loadWorld(std::filesystem::path const& p_path,
                                   AssetResolver const& p_assets)
{
    ConfigNode file;
    ROBOTIK_TRY_ASSIGN(file, ConfigNode::load(p_path));
    if (!file.has("world"))
    {
        return failure(p_path.string() + ": no 'world:' section");
    }
    std::filesystem::path root;
    ROBOTIK_TRY_ASSIGN(root, packageRoot(p_path));
    return readWorld(file.at("world"), root, p_assets);
}

} // namespace robotik
