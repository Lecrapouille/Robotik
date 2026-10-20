// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "WalkPolicy.hpp"

#include "Robotik/Common/Config.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <stdexcept>

/// Reads a YAML sequence of numbers of the expected length.
static std::vector<double> numbers(robotik::ConfigNode const& p_node,
                                   std::string_view p_key,
                                   std::size_t p_size)
{
    std::vector<double> values;
    for (robotik::ConfigNode const& item : p_node.at(p_key).items())
    {
        std::optional<double> const value = item.asNumber();
        if (!value)
        {
            throw std::runtime_error("walk." + std::string(p_key) +
                                     ": not a number");
        }
        values.push_back(*value);
    }
    if (values.size() != p_size)
    {
        throw std::runtime_error("walk." + std::string(p_key) + ": " +
                                 std::to_string(values.size()) +
                                 " values for " + std::to_string(p_size) +
                                 " joints");
    }
    return values;
}

/// Clamps a value to a symmetric bound and returns it as float.
static float clip(double p_value, double p_bound)
{
    return static_cast<float>(std::clamp(p_value, -p_bound, p_bound));
}

// ----------------------------------------------------------------------------
WalkContract WalkContract::load(std::filesystem::path const& p_path)
{
    robotik::Result<robotik::ConfigNode> loaded = robotik::ConfigNode::load(p_path);
    if (!loaded)
    {
        throw std::runtime_error(loaded.error());
    }
    robotik::ConfigNode const root = loaded.take();
    robotik::ConfigNode const& walk = root.at("walk");

    WalkContract contract;
    contract.joints = walk.strings("joints");
    std::size_t const count = contract.joints.size();
    contract.kp = numbers(walk, "kp", count);
    contract.kd = numbers(walk, "kd", count);
    contract.default_angle = numbers(walk, "default_angle", count);
    contract.action_scale = numbers(walk, "action_scale", count);
    contract.torque_limit = numbers(walk, "torque_limit", count);
    contract.armature = numbers(walk, "armature", count);
    contract.decimation =
        static_cast<int>(walk.number("decimation", contract.decimation));
    contract.history =
        static_cast<int>(walk.number("history", contract.history));
    contract.observations =
        static_cast<int>(walk.number("observations", contract.observations));
    contract.clip_observations =
        walk.number("clip_observations", contract.clip_observations);
    contract.clip_actions = walk.number("clip_actions", contract.clip_actions);
    contract.scale_ang_vel =
        walk.number("scale_ang_vel", contract.scale_ang_vel);
    contract.scale_dof_pos =
        walk.number("scale_dof_pos", contract.scale_dof_pos);
    contract.scale_dof_vel =
        walk.number("scale_dof_vel", contract.scale_dof_vel);
    std::vector<double> const command = numbers(walk, "max_command", 3U);
    std::ranges::copy(command, contract.max_command.begin());

    if (contract.observations != static_cast<int>(9U + 3U * count))
    {
        throw std::runtime_error("walk.observations must be 9 + 3 x joints");
    }
    return contract;
}

// ****************************************************************************
struct WalkPolicy::Onnx
{
    explicit Onnx(std::filesystem::path const& p_model)
        : env(ORT_LOGGING_LEVEL_WARNING, "robotik_humanoid_walk"),
          session(env, p_model.c_str(), options()),
          memory(
              Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    {
        Ort::AllocatorWithDefaultOptions allocator;
        input = session.GetInputNameAllocated(0, allocator).get();
        output = session.GetOutputNameAllocated(0, allocator).get();
    }

    static Ort::SessionOptions options()
    {
        // One thread: the step is a few microseconds and runs in the
        // simulation loop, next to parallel environments.
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        return options;
    }

    Ort::Env env;
    Ort::Session session;
    Ort::MemoryInfo memory;
    std::string input;
    std::string output;
};

// ----------------------------------------------------------------------------
WalkPolicy::WalkPolicy(std::filesystem::path const& p_model,
                       WalkContract p_contract)
    : m_contract(std::move(p_contract))
{
    try
    {
        m_onnx = std::make_unique<Onnx>(p_model);
    }
    catch (Ort::Exception const& e)
    {
        throw std::runtime_error("Walk policy '" + p_model.string() +
                                 "': " + e.what());
    }
    std::size_t const count = m_contract.joints.size();
    m_history.assign(
        static_cast<std::size_t>(m_contract.history * m_contract.observations),
        0.0F);
    m_actions.assign(count, 0.0F);
    m_targets.assign(m_contract.default_angle.begin(),
                     m_contract.default_angle.end());
}

WalkPolicy::~WalkPolicy() = default;

// ----------------------------------------------------------------------------
void WalkPolicy::reset()
{
    std::ranges::fill(m_history, 0.0F);
    std::ranges::fill(m_actions, 0.0F);
    m_targets.assign(m_contract.default_angle.begin(),
                     m_contract.default_angle.end());
    m_fresh = true;
}

// ----------------------------------------------------------------------------
void WalkPolicy::observe(WalkInput const& p_input)
{
    WalkContract const& c = m_contract;
    std::size_t const count = c.joints.size();
    std::size_t const size = static_cast<std::size_t>(c.observations);
    std::vector<float> obs;
    obs.reserve(size);

    // Gravity seen from the IMU: the policy reads the tilt from it.
    robotik::Vector3 const gravity =
        p_input.orientation.conjugate() * robotik::Vector3(0.0, 0.0, -1.0);
    robotik::Vector3 const& w = p_input.angular_velocity;
    for (double v : { w.x, w.y, w.z })
    {
        obs.push_back(clip(v * c.scale_ang_vel, c.clip_observations));
    }
    for (double v : { gravity.x, gravity.y, gravity.z })
    {
        obs.push_back(clip(v, c.clip_observations));
    }
    for (double v : p_input.command)
    {
        obs.push_back(clip(v, c.clip_observations));
    }
    for (std::size_t i = 0; i < count; ++i)
    {
        obs.push_back(
            clip((p_input.positions[i] - c.default_angle[i]) * c.scale_dof_pos,
                 c.clip_observations));
    }
    for (std::size_t i = 0; i < count; ++i)
    {
        obs.push_back(
            clip(p_input.velocities[i] * c.scale_dof_vel, c.clip_observations));
    }
    obs.insert(obs.end(), m_actions.begin(), m_actions.end());

    // Newest first. The first observation fills the whole history.
    if (m_fresh)
    {
        for (std::size_t k = 0; k < static_cast<std::size_t>(c.history); ++k)
        {
            std::ranges::copy(obs,
                              m_history.begin() + static_cast<long>(k * size));
        }
        m_fresh = false;
        return;
    }
    std::copy_backward(m_history.begin(),
                       m_history.end() - static_cast<long>(size),
                       m_history.end());
    std::ranges::copy(obs, m_history.begin());
}

// ----------------------------------------------------------------------------
std::span<double const> WalkPolicy::act(WalkInput const& p_input)
{
    WalkContract const& c = m_contract;
    observe(p_input);

    std::array<int64_t, 2> const shape{
        1, static_cast<int64_t>(m_history.size())
    };
    Ort::Value input = Ort::Value::CreateTensor<float>(m_onnx->memory,
                                                       m_history.data(),
                                                       m_history.size(),
                                                       shape.data(),
                                                       shape.size());
    char const* const inputs[] = { m_onnx->input.c_str() };
    char const* const outputs[] = { m_onnx->output.c_str() };
    std::vector<Ort::Value> result = m_onnx->session.Run(
        Ort::RunOptions{ nullptr }, inputs, &input, 1U, outputs, 1U);

    float const* const actions = result.front().GetTensorData<float>();
    for (std::size_t i = 0; i < m_actions.size(); ++i)
    {
        m_actions[i] = clip(actions[i], c.clip_actions);
        m_targets[i] = static_cast<double>(m_actions[i]) * c.action_scale[i] +
                       c.default_angle[i];
    }
    return m_targets;
}
