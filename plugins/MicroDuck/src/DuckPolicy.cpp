// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "DuckPolicy.hpp"

#include "Robotik/Package/Scenario/Config.hpp"

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cstdint>
#include <stdexcept>

/// Width of the observation the official policies declare.
#define OBS_LEN 61
/// Servos the policy commands. The mouth is not one of them.
#define ACTION_LEN 14

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

// ----------------------------------------------------------------------------
DuckContract DuckContract::load(std::filesystem::path const& p_path)
{
    robotik::ConfigNode const root = robotik::ConfigNode::load(p_path);
    robotik::ConfigNode const& walk = root.at("walk");

    DuckContract contract;
    contract.joints = walk.strings("joints");
    std::size_t const count = contract.joints.size();
    if (count != ACTION_LEN)
    {
        throw std::runtime_error("walk.joints: the policy commands 14 servos");
    }
    contract.kp = numbers(walk, "kp", count);
    contract.kd = numbers(walk, "kd", count);
    contract.damping = numbers(walk, "damping", count);
    contract.default_angle = numbers(walk, "default_angle", count);
    contract.action_scale = numbers(walk, "action_scale", count);
    contract.torque_limit = numbers(walk, "torque_limit", count);
    contract.armature = numbers(walk, "armature", count);
    contract.decimation =
        static_cast<int>(walk.number("decimation", contract.decimation));
    contract.physics_step = walk.number("physics_step", contract.physics_step);
    int const observations =
        static_cast<int>(walk.number("observations", OBS_LEN));
    if (observations != OBS_LEN)
    {
        throw std::runtime_error("walk.observations must be 61");
    }
    std::vector<double> const command = numbers(walk, "max_command", 3u);
    std::copy(command.begin(), command.end(), contract.max_command.begin());
    return contract;
}

// ****************************************************************************
struct DuckPolicy::Onnx
{
    explicit Onnx(std::filesystem::path const& p_model)
        : env(ORT_LOGGING_LEVEL_WARNING, "robotik_microduck"),
          session(env, p_model.c_str(), options()),
          memory(
              Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    {
        Ort::AllocatorWithDefaultOptions allocator;
        input = session.GetInputNameAllocated(0, allocator).get();
        output = session.GetOutputNameAllocated(0, allocator).get();
        std::vector<std::int64_t> const in =
            session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        std::vector<std::int64_t> const out =
            session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (in.size() != 2u || in.back() != OBS_LEN || out.size() != 2u ||
            out.back() != ACTION_LEN)
        {
            throw std::runtime_error("policy graph is not [1,61] -> [1,14]");
        }
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
DuckPolicy::DuckPolicy(std::filesystem::path const& p_model,
                       DuckContract p_contract)
    : m_contract(std::move(p_contract))
{
    try
    {
        m_onnx = std::make_unique<Onnx>(p_model);
    }
    catch (Ort::Exception const& e)
    {
        throw std::runtime_error("Duck policy '" + p_model.string() +
                                 "': " + e.what());
    }
    catch (std::runtime_error const& e)
    {
        throw std::runtime_error("Duck policy '" + p_model.string() +
                                 "': " + e.what());
    }
    std::size_t const count = m_contract.joints.size();
    m_observation.assign(OBS_LEN, 0.0f);
    m_actions.assign(count, 0.0f);
    m_targets.assign(m_contract.default_angle.begin(),
                     m_contract.default_angle.end());
}

DuckPolicy::~DuckPolicy() = default;

// ----------------------------------------------------------------------------
void DuckPolicy::reset()
{
    std::fill(m_actions.begin(), m_actions.end(), 0.0f);
    m_targets.assign(m_contract.default_angle.begin(),
                     m_contract.default_angle.end());
}

// ----------------------------------------------------------------------------
std::span<double const> DuckPolicy::act(DuckInput const& p_input)
{
    DuckContract const& c = m_contract;
    std::size_t const count = c.joints.size();
    // Gravity seen from the trunk: the policy reads the tilt from it.
    robotik::Vector3 const gravity =
        p_input.orientation.conjugate() * robotik::Vector3(0.0, 0.0, -1.0);
    robotik::Vector3 const& w = p_input.angular_velocity;

    std::size_t at = 0;
    auto put = [&](double p_value)
    { m_observation[at++] = static_cast<float>(p_value); };
    put(w.x);
    put(w.y);
    put(w.z);
    put(gravity.x);
    put(gravity.y);
    put(gravity.z);
    for (std::size_t i = 0; i < count; ++i)
    {
        put(p_input.positions[i] - c.default_angle[i]);
    }
    for (std::size_t i = 0; i < count; ++i)
    {
        put(p_input.velocities[i]);
    }
    for (float action : m_actions)
    {
        put(action);
    }
    for (double value : p_input.command)
    {
        put(value);
    }
    // Head (4) and body (6) stay zero: x, y and yaw are unbound, and this
    // scenario does not steer the head or the trunk pose.
    while (at < OBS_LEN)
    {
        put(0.0);
    }

    std::array<std::int64_t, 2> const shape{ 1, OBS_LEN };
    Ort::Value input = Ort::Value::CreateTensor<float>(m_onnx->memory,
                                                       m_observation.data(),
                                                       m_observation.size(),
                                                       shape.data(),
                                                       shape.size());
    char const* const inputs[] = { m_onnx->input.c_str() };
    char const* const outputs[] = { m_onnx->output.c_str() };
    std::vector<Ort::Value> result = m_onnx->session.Run(
        Ort::RunOptions{ nullptr }, inputs, &input, 1u, outputs, 1u);

    float const* const actions = result.front().GetTensorData<float>();
    for (std::size_t i = 0; i < count; ++i)
    {
        m_actions[i] = actions[i];
        m_targets[i] = static_cast<double>(m_actions[i]) * c.action_scale[i] +
                       c.default_angle[i];
    }
    return m_targets;
}
