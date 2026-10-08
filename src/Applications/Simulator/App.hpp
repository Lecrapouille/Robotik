// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "ColorDetector.hpp"
#include "LineFollowerMission.hpp"
#include "PickPlaceControl.hpp"
#include "PickPlaceMission.hpp"
#include "SimulatorView.hpp"

#include "FlyBrain.hpp"
#include "FlyEnvironment.hpp"

#include "Robotik/Math/Random.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Robot/TeachPendant.hpp"

#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/World.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

inline constexpr char const* TEACH_PANEL = "Teach";

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

enum class HostedMission
{
    PickPlace,
    PickPlaceFaults,
    LineFollower,
    PickPlaceRl,
    Fly,
};

struct RlWatch
{
    bool converged = true;
    float mix = 1.0f;
    robotik::Random noise{ robotik::Seed{ 1 } };
    bool auto_repeat = true;
    float spread = 0.06f;
    int max_steps = 120;
    std::array<float, PICK_PLACE_OBSERVATIONS> observation{};
    std::array<float, PICK_PLACE_ACTIONS> action{};
    std::uint32_t steps = 0;
    float episode_return = 0.0f;
    std::uint32_t delivered = 0;
    std::uint32_t failed = 0;
    bool grasped = false;
    bool done = false;
    bool success = false;
    robotik::JointGroup* arm = nullptr;
    robotik::VacuumGripper* gripper = nullptr;
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
    HostedMission kind = HostedMission::PickPlace;
    std::filesystem::path scenario_path = "data/scenarios/pick_and_place.yml";
    std::string scenario_text;
    std::string error;

    std::unique_ptr<compages::world::World> world;
    std::unique_ptr<compages::renderer::Scene> scene;
    std::unique_ptr<SimulatorView> scene_view;
    std::unique_ptr<robotik::Mission> mission;
    std::unique_ptr<robotik::Simulation> simulation;
    ColorDetector* detector = nullptr;
    LineFollowerMission* line_follower = nullptr;
    RlWatch rl;
    TeachWatch teach;
    FlyWatch fly;
    compages::world::Entity view_camera;

    RenderTarget view;
    bool view_hovered = false;

    bool playing = true;
    bool step_once = false;
    float speed = 1.0f;
    std::uint64_t seed = 0;

    void select(HostedMission p_kind);
    void load();
    void reset(std::uint64_t p_seed);
    void advance(double p_elapsed);

private:

    void resetRl();
    void stepRl();
    void advanceFly(double p_elapsed);

    double m_lag = 0.0;
    double m_rl_pause = 0.0;
    bool m_reported = false;
};

void drawPanels(App& p_app);
void teachPanel(App& p_app);
