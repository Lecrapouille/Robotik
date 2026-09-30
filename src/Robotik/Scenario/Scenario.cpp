#include "Robotik/Scenario/Scenario.hpp"

#include "BlackThorn/Builder/Yaml.hpp"

#include <filesystem>
#include <stdexcept>

namespace robotik
{

namespace
{

std::array<float, 3> triple(bt::YamlNode const& p_node, std::array<float, 3> p_default)
{
    if (!p_node.valid() || !p_node.isSeq() || p_node.size() != 3u)
    {
        return p_default;
    }
    for (std::size_t i = 0; i < 3u; ++i)
    {
        p_default[i] = static_cast<float>(p_node.child(i).asDouble().value_or(p_default[i]));
    }
    return p_default;
}

std::string text(bt::YamlNode const& p_node, std::string const& p_key)
{
    return p_node.valid() && p_node.hasKey(p_key) ? p_node.child(p_key).scalar()
                                                   : std::string{};
}

std::string resolve(std::filesystem::path const& p_directory, std::string const& p_file)
{
    if (p_file.empty())
    {
        return {};
    }
    return (p_directory / p_file).lexically_normal().string();
}

} // namespace

Scenario Scenario::load(std::string const& p_path)
{
    auto parsed = bt::YamlDocument::parseFile(p_path);
    if (!parsed)
    {
        throw std::runtime_error("Cannot read scenario '" + p_path + "': " +
                                 parsed.getError());
    }
    bt::YamlNode const root = parsed.getValue().root();
    std::filesystem::path const directory =
        std::filesystem::absolute(p_path).parent_path();

    Scenario scenario;
    scenario.name = text(root, "scenario");

    bt::YamlNode const world = root.child("world");
    bt::YamlNode const robot = world.child("robot");
    scenario.robot_model = resolve(directory, text(robot, "model"));
    if (scenario.robot_model.empty())
    {
        throw std::runtime_error("Scenario '" + p_path + "' has no world.robot.model");
    }
    if (robot.hasKey("home"))
    {
        robot.child("home").forEachMap(
            [&](std::string_view p_joint, bt::YamlNode p_value)
            { scenario.home[std::string(p_joint)] = p_value.asDouble().value_or(0.0); });
    }
    if (robot.hasKey("tool_length"))
    {
        scenario.tool_length = robot.child("tool_length").asDouble().value_or(0.06);
    }
    if (robot.hasKey("camera"))
    {
        bt::YamlNode const node = robot.child("camera");
        Camera camera;
        camera.link = text(node, "link");
        camera.position = triple(node.child("position"), camera.position);
        if (node.hasKey("fov"))
        {
            camera.sensor.fov_degrees = static_cast<float>(
                node.child("fov").asDouble().value_or(camera.sensor.fov_degrees));
        }
        if (auto const size = triple(node.child("resolution"), { 320.0f, 240.0f, 0.0f });
            size[0] > 0.0f && size[1] > 0.0f)
        {
            camera.sensor.width = static_cast<std::uint32_t>(size[0]);
            camera.sensor.height = static_cast<std::uint32_t>(size[1]);
        }
        scenario.camera = camera;
    }

    if (world.hasKey("objects"))
    {
        world.child("objects").forEachSeq(
            [&](bt::YamlNode p_node)
            {
                Object object;
                object.shape.name = text(p_node, "name");
                std::string const type = text(p_node, "type");
                if (type == "box")
                {
                    object.shape.type = ecs::SceneObject::Type::Box;
                }
                else if (type != "cube")
                {
                    throw std::runtime_error("Object '" + object.shape.name +
                                             "': unknown type '" + type + "'");
                }
                object.shape.size = triple(p_node.child("size"), object.shape.size);
                object.shape.color = triple(p_node.child("color"), object.shape.color);
                object.position = triple(p_node.child("position"), object.position);
                scenario.objects.push_back(std::move(object));
            });
    }

    bt::YamlNode const execute = root.child("execute");
    scenario.task = text(execute, "task");
    scenario.behavior_tree = resolve(directory, text(execute, "behavior_tree"));

    if (root.hasKey("assert"))
    {
        root.child("assert").forEachSeq([&](bt::YamlNode p_node)
                                        { scenario.asserts.push_back(p_node.scalar()); });
    }
    return scenario;
}

} // namespace robotik
