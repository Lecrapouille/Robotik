// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyView.hpp"

#include "FlyDraw.hpp"

#include "Robotik/Robot/Backends/SceneView.hpp"
#include "Robotik/Robot/Robot.hpp"

#include "Compages/GPU/GPU.hpp"
#include "Compages/GPU/RenderPass.hpp"
#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/Components/Camera.hpp"
#include "Compages/World/World.hpp"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>

static constexpr std::size_t TRAIL = 240;
static constexpr float PI = 3.14159265358979323846F;

/// Loads the URDF into the window scene. No extra cameras or ground:
/// the fly demo builds those itself.
class FlySceneView final: public robotik::SceneView
{
public:

    explicit FlySceneView(compages::renderer::Scene& p_scene) : m_scene(p_scene)
    {
    }

    // ------------------------------------------------------------------------
    /// Loads the URDF into the window scene.
    // ------------------------------------------------------------------------
    compages::world::Entity robot(compages::world::World& /*p_world*/,
                                  std::filesystem::path const& p_urdf) override
    {
        auto loaded = m_scene.load(p_urdf.string());
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        return loaded.value();
    }

private:

    compages::renderer::Scene& m_scene;
};

/// Everything that must die before GLFW. The robot first, because its
/// entities live in the world.
struct FlyView::Gpu
{
    GLFWwindow* window = nullptr;
    bool glfw = false;
    std::unique_ptr<compages::world::World> world;
    std::unique_ptr<compages::renderer::Scene> scene;
    std::unique_ptr<FlySceneView> hooks;
    std::unique_ptr<robotik::RobotSession> robot;
    compages::world::Entity camera;
    compages::world::Entity eyes[2];
    compages::world::Entity beams[3];
    compages::world::Entity beam_from[3];
    compages::world::Entity beam_to[3];
    compages::world::Entity trail[TRAIL];

    ~Gpu()
    {
        robot.reset();
        hooks.reset();
        scene.reset();
        world.reset();
        if (glfw)
        {
            glfwTerminate();
        }
    }
};

/// Stores the scenario. The window stays closed until @ref open.
FlyView::FlyView(FlyScenario const& p_scenario) : m_scenario(p_scenario) {}

/// Defined here so the unique_ptr to @ref Gpu sees a complete type.
FlyView::~FlyView() = default;

/// Window, chase camera, URDF, eye insets, arena, beams and trail.
bool FlyView::open()
{
    m_gpu = std::make_unique<Gpu>();
    if (glfwInit() == GLFW_FALSE)
    {
        std::cerr << "GLFW: no window\n";
        return false;
    }
    m_gpu->glfw = true;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    m_gpu->window =
        glfwCreateWindow(1600, 900, "Robotik — fly", nullptr, nullptr);
    if (m_gpu->window == nullptr)
    {
        std::cerr << "Cannot open an OpenGL 4.5 window\n";
        return false;
    }
    glfwMakeContextCurrent(m_gpu->window);
    glfwSwapInterval(1);
    auto ready = compages::gpu::init(
        reinterpret_cast<compages::gpu::LoadProc>(glfwGetProcAddress));
    if (!ready)
    {
        std::cerr << ready.error() << '\n';
        return false;
    }

    m_gpu->world = std::make_unique<compages::world::World>();
    m_gpu->scene = std::make_unique<compages::renderer::Scene>(*m_gpu->world);
    m_gpu->scene->background(0.55F, 0.74F, 0.90F).ambient(0.42F, 0.42F, 0.44F);
    m_gpu->scene->sun("Sun").rotation(
        robotik::Radians(-0.9F), compages::core::Vector3f(1.0F, 0.3F, 0.0F));
    m_gpu->camera = m_gpu->scene->camera("View");
    m_gpu->scene->activeCamera(m_gpu->camera);

    m_gpu->hooks = std::make_unique<FlySceneView>(*m_gpu->scene);
    m_gpu->robot = std::make_unique<robotik::RobotSession>(
        *m_gpu->world, m_scenario.robot_model, m_gpu->hooks.get());
    // Insets sit on the bottom corners. y is the bottom of the window.
    m_gpu->eyes[0] =
        mountFlyEye(*m_gpu->scene, *m_gpu->robot, "eye_L", "EyeLeft");
    m_gpu->eyes[1] =
        mountFlyEye(*m_gpu->scene, *m_gpu->robot, "eye_R", "EyeRight");
    if (m_gpu->eyes[0])
    {
        m_gpu->eyes[0].get<compages::world::Camera>().viewport = {
            0.02F, 0.04F, 0.24F, 0.30F
        };
    }
    if (m_gpu->eyes[1])
    {
        m_gpu->eyes[1].get<compages::world::Camera>().viewport = {
            0.74F, 0.04F, 0.24F, 0.30F
        };
    }

    // Boxes are Z-up in the scenario. Compages Y is up, so height and depth
    // swap.
    auto brown = compages::renderer::color(0.45F, 0.32F, 0.18F);
    int index = 0;
    for (FlyBox const& box : m_scenario.obstacles)
    {
        compages::core::Vector3f const at = flyToView(box.position);
        m_gpu->scene->box("obstacle_" + std::to_string(index), brown)
            .position(at.x, at.y, at.z)
            .scale(static_cast<float>(box.size.x),
                   static_cast<float>(box.size.z),
                   static_cast<float>(box.size.y));
        ++index;
    }
    compages::core::Vector3f const food = flyToView(m_scenario.target);
    m_gpu->scene->sphere("food", compages::renderer::color(0.95F, 0.75F, 0.15F))
        .position(food.x, food.y, food.z)
        .scale(0.18F);
    compages::core::Vector3f const middle =
        flyToView(robotik::Vector3(m_scenario.arena.x * 0.5, 0.0, 0.0));
    m_gpu->scene->plane("Floor", compages::renderer::color(0.55F, 0.62F, 0.38F))
        .position(middle.x, 0.0F, middle.z)
        .rotation(robotik::Radians(-0.5F * PI),
                  compages::core::Vector3f(1.0F, 0.0F, 0.0F))
        .scale(static_cast<float>(m_scenario.arena.x + 4.0),
               static_cast<float>(m_scenario.arena.y + 4.0),
               1.0F);

    // Yellow left, red centre, blue right. A sphere marks each end of the cone.
    auto const left = compages::renderer::color(0.95F, 0.75F, 0.2F);
    auto const center = compages::renderer::color(0.95F, 0.25F, 0.2F);
    auto const right = compages::renderer::color(0.25F, 0.45F, 1.0F);
    compages::renderer::Look const looks[3] = { left, center, right };
    char const* names[3] = { "ray_left", "ray_center", "ray_right" };
    for (int ray = 0; ray < 3; ++ray)
    {
        m_gpu->beams[ray] =
            m_gpu->scene->cone(std::string(names[ray]), looks[ray]);
        m_gpu->beam_from[ray] =
            m_gpu->scene->sphere(std::string(names[ray]) + "_from", looks[ray]);
        m_gpu->beam_to[ray] =
            m_gpu->scene->sphere(std::string(names[ray]) + "_to", looks[ray]);
    }
    auto crumb = compages::renderer::color(0.95F, 0.95F, 0.9F);
    for (std::size_t i = 0; i < TRAIL; ++i)
    {
        m_gpu->trail[i] =
            m_gpu->scene->sphere("trail_" + std::to_string(i), crumb)
                .position(0.0F, -8.0F, 0.0F)
                .scale(0.04F);
    }

    if (auto prepared = m_gpu->scene->prepare(); !prepared)
    {
        std::cerr << prepared.error() << '\n';
        return false;
    }
    return true;
}

/// One posed frame. Escape and the window close both return false.
bool FlyView::frame(FlyEnvironment const& p_environment)
{
    if (m_gpu == nullptr || m_gpu->window == nullptr)
    {
        return false;
    }
    if (glfwWindowShouldClose(m_gpu->window) == GLFW_TRUE)
    {
        return false;
    }
    glfwPollEvents();
    if (glfwGetKey(m_gpu->window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    {
        return false;
    }

    FlySnapshot const& snapshot = p_environment.snapshot();
    poseFlyBody(*m_gpu->robot, snapshot);
    m_gpu->robot->step(robotik::Seconds(p_environment.dt()));

    for (int ray = 0; ray < 3; ++ray)
    {
        float const value = ray == 0 ? snapshot.vision.left
                                     : (ray == 1 ? snapshot.vision.center
                                                 : snapshot.vision.right);
        aimFlyBeam(m_gpu->beams[ray],
                   m_gpu->beam_from[ray],
                   m_gpu->beam_to[ray],
                   snapshot.vision.origins[ray],
                   snapshot.vision.ends[ray],
                   value);
    }

    std::span<robotik::Vector3 const> const trail = p_environment.trail();
    std::size_t const shown = std::min(trail.size(), TRAIL);
    for (std::size_t i = 0; i < shown; ++i)
    {
        compages::core::Vector3f const at = flyToView(trail[i]);
        m_gpu->trail[i].position(at.x, at.y, at.z);
    }

    compages::core::Vector3f const thorax = flyToView(
        robotik::Vector3(snapshot.plant.x, snapshot.plant.y, snapshot.plant.z));
    m_gpu->camera.position(thorax.x - 2.4F, thorax.y + 1.8F, thorax.z + 2.6F)
        .lookAt(thorax);
    m_gpu->world->update();

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(m_gpu->window, &width, &height);
    if (width > 0 && height > 0)
    {
        compages::gpu::RenderPass pass(compages::gpu::PassDesc{
            .width = static_cast<std::uint32_t>(width),
            .height = static_cast<std::uint32_t>(height),
            .color = { 0.55F, 0.74F, 0.90F, 1.0F } });
        // The chase view first. Each eye camera then clears only its inset.
        m_gpu->scene->render(m_gpu->camera);
        if (m_gpu->eyes[0])
        {
            m_gpu->scene->render(m_gpu->eyes[0]);
        }
        if (m_gpu->eyes[1])
        {
            m_gpu->scene->render(m_gpu->eyes[1]);
        }
    }
    glfwSwapBuffers(m_gpu->window);
    return true;
}
