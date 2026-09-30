#include "App.hpp"

#include "Robotik/ECS/PerceptionComponents.hpp"

#include "Compages/GPU/GPU.hpp"
#include "Compages/GPU/RenderPass.hpp"
#include "Compages/World/Controllers/Controls.hpp"
#include "Compages/World/Controllers/ViewFrame.hpp"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <numbers>
#include <span>

namespace
{

compages::core::Vector4f const kSky{ 0.12f, 0.14f, 0.18f, 1.0f };

struct Window
{
    GLFWwindow* handle = nullptr;
    bool glfw_ready = false;
    bool device_ready = false;
    bool imgui_ready = false;

    bool open()
    {
        if (glfwInit() == GLFW_FALSE)
        {
            return false;
        }
        glfw_ready = true;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_DEPTH_BITS, 24);
        handle = glfwCreateWindow(1600, 900, "Robotik Simulator", nullptr, nullptr);
        if (handle == nullptr)
        {
            return false;
        }
        glfwMakeContextCurrent(handle);
        glfwSwapInterval(1);
        auto ready = compages::gpu::init(
            reinterpret_cast<compages::gpu::LoadProc>(glfwGetProcAddress));
        if (!ready)
        {
            std::cerr << ready.error() << '\n';
            return false;
        }
        device_ready = true;

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(handle, true);
        ImGui_ImplOpenGL3_Init("#version 450");
        imgui_ready = true;
        return true;
    }

    ~Window()
    {
        if (imgui_ready)
        {
            ImGui_ImplOpenGL3_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }
        if (device_ready)
        {
            compages::gpu::shutdown();
        }
        if (handle != nullptr)
        {
            glfwDestroyWindow(handle);
        }
        if (glfw_ready)
        {
            glfwTerminate();
        }
    }
};

// Mouse input of the world panel, in its own pixels with Y up.
compages::world::ViewFrame viewFrame(App const& p_app, float p_elapsed, float p_total)
{
    ImGuiIO const& io = ImGui::GetIO();
    compages::world::ViewFrame frame;
    frame.width = p_app.view.width;
    frame.height = p_app.view.height;
    frame.elapsed = p_elapsed;
    frame.total = p_total;
    frame.input.mouse_over = p_app.view_hovered;
    if (p_app.view_hovered)
    {
        frame.input.mouse_delta = compages::core::Vector2f(io.MouseDelta.x, -io.MouseDelta.y);
        frame.input.scroll = io.MouseWheel;
        frame.input.mouse_right = io.MouseDown[ImGuiMouseButton_Right];
    }
    return frame;
}

// Renders the wrist camera, reads the picture back and runs the detector.
void perceive(App& p_app)
{
    compages::world::Entity camera = p_app.simulation->camera();
    if (!camera)
    {
        return;
    }
    auto const& sensor = camera.get<robotik::ecs::CameraSensor>();
    if (!p_app.robot_view.resize(sensor.width, sensor.height))
    {
        return;
    }
    {
        compages::gpu::RenderPass pass(p_app.robot_view.framebuffer,
                                       compages::gpu::PassDesc{ .color = kSky });
        p_app.scene->render(camera);
    }
    auto pixels = p_app.robot_view.color.read();
    if (!pixels)
    {
        return;
    }
    std::span<const std::byte> const bytes = pixels.value();
    auto& detected = camera.get<robotik::ecs::DetectedObjects>();
    detected.items = p_app.detector.detect(
        { reinterpret_cast<std::uint8_t const*>(bytes.data()), bytes.size() },
        static_cast<int>(sensor.width), static_cast<int>(sensor.height));
    ++detected.frame;
}

} // namespace

bool RenderTarget::resize(std::uint32_t p_width, std::uint32_t p_height)
{
    if (p_width == width && p_height == height)
    {
        return width != 0;
    }
    width = 0;
    height = 0;
    compages::Status made = color.allocate(
        { .format = compages::gpu::PixelFormat::RGB8, .width = p_width, .height = p_height });
    if (made)
    {
        made = depth.allocate({ .format = compages::gpu::PixelFormat::Depth32F,
                                .width = p_width,
                                .height = p_height,
                                .magnify = compages::gpu::Filter::Nearest,
                                .minify = compages::gpu::Filter::Nearest });
    }
    if (made)
    {
        made = framebuffer.attach(color, depth);
    }
    if (!made)
    {
        std::cerr << "Render target: " << made.error() << '\n';
        return false;
    }
    width = p_width;
    height = p_height;
    return true;
}

void App::load()
{
    error.clear();
    simulation.reset();
    scene.reset();
    world = std::make_unique<compages::world::World>();
    scene = std::make_unique<compages::renderer::Scene>(*world);
    scene->background(kSky.x, kSky.y, kSky.z).ambient(0.35f, 0.35f, 0.38f);
    scene->sun("Sun").rotation(-0.9f, compages::core::Vector3f(1.0f, 0.3f, 0.0f));
    view_camera = scene->camera("ViewCamera");
    scene->activeCamera(view_camera);
    detector = {};
    try
    {
        simulation = std::make_unique<robotik::Simulation>(
            *world, scene.get(), robotik::Scenario::load(scenario_path));
    }
    catch (std::exception const& failure)
    {
        error = failure.what();
        simulation.reset();
        return;
    }

    scene->plane("Floor", compages::renderer::color(0.32f, 0.33f, 0.35f))
        .parent(simulation->robot())
        .position(0.0f, 0.0f, -0.001f)
        .scale(3.0f);
    for (auto const& object : simulation->scenario().objects)
    {
        detector.add(object.shape.name, object.shape.color);
    }
    view_camera.position(1.4f, 1.1f, 1.4f)
        .add<compages::world::Orbit>(compages::core::Vector3f(0.2f, 0.3f, 0.0f));
    if (auto prepared = scene->prepare(); !prepared)
    {
        error = prepared.error();
    }
    playing = true;
}

int main(int argc, char** argv)
{
    Window window;
    if (!window.open())
    {
        std::cerr << "Cannot open an OpenGL 4.5 window\n";
        return 1;
    }

    App app;
    app.scenario_path = (argc > 1) ? argv[1] : "data/scenarios/pick_and_place.yml";
    app.load();

    double last = glfwGetTime();
    float total = 0.0f;
    while (glfwWindowShouldClose(window.handle) == GLFW_FALSE)
    {
        glfwPollEvents();
        double const now = glfwGetTime();
        float const elapsed = static_cast<float>(now - last);
        last = now;
        total += elapsed;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawPanels(app);
        ImGui::Render();

        if (app.simulation && app.view.width > 0)
        {
            bool const advance = app.playing || app.step_once;
            app.step_once = false;
            compages::world::ViewFrame frame =
                viewFrame(app, advance ? elapsed * app.speed : 0.0f, total);
            if (advance)
            {
                app.simulation->step(frame);
            }
            else
            {
                app.world->update(frame);
            }
            perceive(app);
            compages::gpu::RenderPass pass(app.view.framebuffer,
                                           compages::gpu::PassDesc{ .color = kSky });
            app.scene->render(app.view_camera);
        }

        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window.handle, &width, &height);
        {
            compages::gpu::RenderPass pass(compages::gpu::PassDesc{
                .width = static_cast<std::uint32_t>(width),
                .height = static_cast<std::uint32_t>(height),
                .color = { 0.05f, 0.05f, 0.06f, 1.0f } });
        }
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window.handle);
    }
    return 0;
}
