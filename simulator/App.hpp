// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "SimulatorView.hpp"

#include "FlyBrain.hpp"
#include "FlyEnvironment.hpp"

#include "Robotik/Robot/TeachPendant.hpp"
#include "Robotik/Plugin/PluginSession.hpp"
#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/World.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

inline constexpr char const* TEACH_PANEL = "Teach";
inline constexpr char const* TIMELINE_PANEL = "Timeline";
inline constexpr char const* SKILL_LIST_PANEL = "Skill list";
inline constexpr char const* SKILLS_PANEL = "Skills";

struct TeachWatch
{
    bool manual = false;
    bool loop = false;
    float joint_step = 0.05f;
    float linear_step = 0.01f;
    float angular_step = 0.05f;
    float duration = 2.0f;
    std::string label;
    robotik::TeachPendant pendant;
    std::vector<compages::world::Entity> markers;
    std::uint32_t marker_serial = 0;
};

struct FlyWatch
{
    std::unique_ptr<FlyEnvironment> environment;
    std::unique_ptr<FlyBrain> brain;
    std::unique_ptr<robotik::RobotSession> robot;
    std::array<float, FlyObservation::SIZE> observation{};
    std::array<float, FlyAction::SIZE> action{};
    float episode_return = 0.0f;
    bool done = false;
    bool success = false;
    double pause = 0.0;
    std::vector<compages::world::Entity> obstacles;
    compages::world::Entity beams[3];
    compages::world::Entity beam_from[3];
    compages::world::Entity beam_to[3];
    std::vector<compages::world::Entity> trail;
    //!< Left eye, then right eye. Empty when the URDF has no such link.
    compages::world::Entity eyes[2];
    RenderTarget eye_picture[2];
    //!< True when @ref brain was loaded from the FlyWire edge list.
    bool connectome = false;
};

struct App
{
    std::filesystem::path scenario_path;
    std::string scenario_title;
    std::string scenario_text;
    std::string error;
    //! @brief The loaded package advances time inside its own update.
    bool owns_clock = false;

    robotik::PluginSession plugins;

    std::unique_ptr<compages::world::World> world;
    std::unique_ptr<compages::renderer::Scene> scene;
    std::unique_ptr<SimulatorView> scene_view;
    std::unique_ptr<robotik::Mission> mission;
    std::unique_ptr<robotik::Simulation> simulation;
    TeachWatch teach;
    FlyWatch fly;
    compages::world::Entity view_camera;

    RenderTarget view;
    bool view_hovered = false;

    bool playing = true;
    bool step_once = false;
    float speed = 1.0f;
    std::uint64_t seed = 0;

    void load();
    void reset(std::uint64_t p_seed);
    void advance(double p_elapsed);

private:

    void advanceFly(double p_elapsed);

    double m_lag = 0.0;
    bool m_reported = false;
};

void drawPanels(App& p_app);
void teachPanel(App& p_app);
void skillsBoard(App& p_app);
