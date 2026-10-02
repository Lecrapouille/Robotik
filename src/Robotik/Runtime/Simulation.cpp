#include "Robotik/Runtime/Simulation.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Skills/HomeSkill.hpp"
#include "Robotik/Skills/PickPlaceSkills.hpp"
#include "Robotik/Systems/GraspSystem.hpp"

#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/Components/Camera.hpp"
#include "Compages/World/World.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <regex>
#include <stdexcept>

#define SCENARIO_WALL_THICKNESS 0.005f
#define APPROACH_CLEARANCE_M 0.10
#define OBJECT_INSIDE_ASSERT_REGEX \
    R"re(object\("([^"]+)"\)\.inside\("([^"]+)"\))re"

namespace robotik
{

//------------------------------------------------------------------------------
//! @brief A container: floor and four walls, all children of p_parent.
//! @param p_scene Renderer scene.
//! @param p_parent Parent entity.
//! @param p_shape Shape of the container.
//------------------------------------------------------------------------------
static void walls(compages::renderer::Scene& p_scene,
                  compages::world::Entity p_parent,
                  ecs::SceneObject const& p_shape)
{
    auto const look = compages::renderer::color(
        p_shape.color[0], p_shape.color[1], p_shape.color[2]);
    auto const x = static_cast<float>(p_shape.size[0].value());
    auto const y = static_cast<float>(p_shape.size[1].value());
    auto const z = static_cast<float>(p_shape.size[2].value());
    auto part = [&](char const* p_name,
                    float p_px,
                    float p_py,
                    float p_pz,
                    float p_sx,
                    float p_sy,
                    float p_sz)
    {
        p_scene.box(p_shape.name + p_name, look)
            .parent(p_parent)
            .position(p_px, p_py, p_pz)
            .scale(p_sx, p_sy, p_sz);
    };
    part("_floor",
         0.0f,
         0.0f,
         (SCENARIO_WALL_THICKNESS - z) * 0.5f,
         x,
         y,
         SCENARIO_WALL_THICKNESS);
    part("_north",
         0.0f,
         (y - SCENARIO_WALL_THICKNESS) * 0.5f,
         0.0f,
         x,
         SCENARIO_WALL_THICKNESS,
         z);
    part("_south",
         0.0f,
         (SCENARIO_WALL_THICKNESS - y) * 0.5f,
         0.0f,
         x,
         SCENARIO_WALL_THICKNESS,
         z);
    part("_east",
         (x - SCENARIO_WALL_THICKNESS) * 0.5f,
         0.0f,
         0.0f,
         SCENARIO_WALL_THICKNESS,
         y,
         z);
    part("_west",
         (SCENARIO_WALL_THICKNESS - x) * 0.5f,
         0.0f,
         0.0f,
         SCENARIO_WALL_THICKNESS,
         y,
         z);
}

//------------------------------------------------------------------------------
//! @brief Checks if p_object is inside p_container.
//! @param p_object Object entity.
//! @param p_container Container entity.
//! @return True if p_object is inside p_container.
//------------------------------------------------------------------------------
static bool inside(compages::world::Entity p_object,
                   compages::world::Entity p_container)
{
    auto const at = p_object.position();
    auto const center = p_container.position();
    auto const& size = p_container.get<ecs::SceneObject>().size;
    return std::abs(at.x - center.x) <
               static_cast<float>(size[0].value() * 0.5) &&
           std::abs(at.y - center.y) <
               static_cast<float>(size[1].value() * 0.5) &&
           std::abs(at.z - center.z) <
               static_cast<float>(size[2].value() * 0.5);
}

//------------------------------------------------------------------------------
Simulation::Simulation(compages::world::World& p_world,
                       compages::renderer::Scene* p_scene,
                       Scenario p_scenario)
    : m_world(p_world), m_scenario(std::move(p_scenario))
{
    m_runtime =
        (p_scene != nullptr)
            ? std::make_unique<RobotRuntime>(
                  m_world, m_scenario.robot_model, *p_scene)
            : std::make_unique<RobotRuntime>(m_world, m_scenario.robot_model);
    m_runtime->hold(m_scenario.home);
    m_context = std::make_unique<RobotContext>(m_runtime->context());
    spawn(p_scene);
    buildTree();
}

//------------------------------------------------------------------------------
Simulation::~Simulation() = default;

//------------------------------------------------------------------------------
void Simulation::spawn(compages::renderer::Scene* p_scene)
{
    m_world.each<ecs::RobotTag>([this](compages::world::Entity p_entity,
                                       ecs::RobotTag&) { m_robot = p_entity; });

    compages::world::Entity tool = findTool(m_world);
    if (!tool)
    {
        throw std::runtime_error(
            "The robot has no end effector to mount a gripper on");
    }
    tool.set(ecs::VacuumGripper{ {}, m_scenario.tool_length });

    for (Scenario::Object const& object : m_scenario.objects)
    {
        ecs::SceneObject const& shape = object.shape;
        compages::world::Entity body;
        if (p_scene != nullptr && shape.type == ecs::SceneObject::Type::CUBE)
        {
            body = p_scene->box(shape.name,
                                compages::renderer::color(shape.color[0],
                                                          shape.color[1],
                                                          shape.color[2]));
            body.scale(static_cast<float>(shape.size[0].value()),
                       static_cast<float>(shape.size[1].value()),
                       static_cast<float>(shape.size[2].value()));
        }
        else
        {
            body = m_world.entity(shape.name);
            if (p_scene != nullptr)
            {
                walls(*p_scene, body, shape);
            }
        }
        body.parent(m_robot)
            .position(
                object.position[0], object.position[1], object.position[2])
            .set(shape);
    }

    if (m_scenario.camera && p_scene != nullptr)
    {
        Scenario::Camera const& mount = *m_scenario.camera;
        compages::world::Entity link;
        m_world.each<ecs::Link>(
            [&link, link_name = mount.link](compages::world::Entity p_entity,
                                            ecs::Link& p_link)
            { link = (p_link.name == link_name) ? p_entity : link; });
        if (!link)
        {
            throw std::runtime_error("Camera link '" + mount.link +
                                     "' not found");
        }
        compages::world::Entity active = p_scene->activeCamera();
        // Compages cameras look down their -Z: flip it onto the link +Z.
        m_camera = p_scene->camera("RobotCamera")
                       .parent(link)
                       .position(mount.position[0],
                                 mount.position[1],
                                 mount.position[2])
                       .rotation(Radians(std::numbers::pi_v<float>),
                                 compages::core::Vector3f(1.0f, 0.0f, 0.0f));
        auto& lens = m_camera.get<compages::world::Camera>();
        lens.fov = units::angle::degree_t(mount.sensor.fov_degrees);
        lens.near_plane = 0.01f;
        lens.far_plane = 20.0f;
        m_camera.set(mount.sensor).set(ecs::DetectedObjects{});
        if (active)
        {
            p_scene->activeCamera(active);
        }
    }
}

//------------------------------------------------------------------------------
void Simulation::buildTree()
{
    auto add = [&](std::string const& p_name, std::shared_ptr<Skill> p_skill)
    {
        registerSkill(
            m_factory, p_name, std::move(p_skill), *m_context, m_trace);
    };

    add("Home", std::make_shared<HomeSkill>());
    add("Release", std::make_shared<ReleaseSkill>());
    for (Scenario::Object const& object : m_scenario.objects)
    {
        std::string const& name = object.shape.name;
        add("Detect(" + name + ")", std::make_shared<DetectSkill>(name));
        add("Approach(" + name + ")",
            std::make_shared<ApproachSkill>(name,
                                            Length(APPROACH_CLEARANCE_M)));
        add("Reach(" + name + ")",
            std::make_shared<ApproachSkill>(name, Length{}));
        add("Grasp(" + name + ")", std::make_shared<GraspSkill>(name));
    }

    if (m_scenario.behavior_tree.empty())
    {
        return;
    }
    m_blackboard = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromFile(
        m_factory, m_scenario.behavior_tree.string(), m_blackboard);
    if (!built)
    {
        throw std::runtime_error("Behavior tree '" +
                                 m_scenario.behavior_tree.string() +
                                 "': " + built.getError());
    }
    m_tree = std::move(built.getValue());
}

void Simulation::tick(Seconds p_dt)
{
    m_context->time = m_runtime->time();
    m_context->dt = p_dt;
    if (m_tree && !finished())
    {
        m_status = m_tree->tick();
    }
    GraspSystem{}.update(m_world, m_runtime->kinematics());
    if (MujocoBackend const* mujoco = m_runtime->simulation())
    {
        m_max_contacts = std::max(m_max_contacts, mujoco->contacts());
    }
}

void Simulation::step(Seconds p_dt)
{
    tick(p_dt);
    m_runtime->step(p_dt);
}

void Simulation::step(compages::world::ViewFrame const& p_frame)
{
    tick(Seconds(p_frame.elapsed));
    m_runtime->step(p_frame);
}

std::vector<Simulation::Check> Simulation::checks() const
{
    static std::regex const inside_assert_regex(OBJECT_INSIDE_ASSERT_REGEX);

    std::vector<Check> result;
    for (std::string const& text : m_scenario.asserts)
    {
        Check check{ text, false };
        std::smatch match;
        if (text == "robot.success")
        {
            check.passed = m_status == bt::Status::SUCCESS;
        }
        else if (text == "gripper.empty")
        {
            compages::world::Entity tool = findTool(m_world);
            check.passed =
                tool && !m_world.alive(tool.get<ecs::VacuumGripper>().held);
        }
        else if (text == "collisions == 0")
        {
            check.passed = m_max_contacts == 0;
        }
        else if (std::regex_match(text, match, inside_assert_regex))
        {
            compages::world::Entity object =
                findObject(m_world, match[1].str());
            compages::world::Entity container =
                findObject(m_world, match[2].str());
            check.passed = object && container && inside(object, container);
        }
        else
        {
            check.text += "  (unknown assertion)";
        }
        result.push_back(std::move(check));
    }
    return result;
}

} // namespace robotik
