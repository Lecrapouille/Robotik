// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Backends/MujocoBackend.hpp"

#include <mujoco/mujoco.h>

#include <pugixml.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <unistd.h>

namespace robotik
{

struct MujocoBackend::Impl
{
    mjModel* model = nullptr;
    mjData* data = nullptr;
    std::filesystem::path generated;

    ~Impl()
    {
        mj_deleteData(data);
        mj_deleteModel(model);
        if (!generated.empty())
        {
            std::error_code ignored;
            std::filesystem::remove(generated, ignored);
        }
    }
};

// MuJoCo rejects a moving body whose mass or inertia is missing or ~0.
// URDF visuals often omit <inertial>. A small default keeps the same file
// loadable without changing what Pinocchio or Compages see.
static std::filesystem::path
withInertia(std::filesystem::path const& p_filename)
{
    pugi::xml_document document;
    if (!document.load_file(p_filename.c_str()))
    {
        return p_filename;
    }

    bool changed = false;
    std::filesystem::path const directory =
        std::filesystem::absolute(p_filename).parent_path();
    for (pugi::xml_node link : document.child("robot").children("link"))
    {
        pugi::xml_node inertial = link.child("inertial");
        double const mass =
            inertial ? inertial.child("mass").attribute("value").as_double(0.0)
                     : 0.0;
        if (!inertial || mass < 1e-8)
        {
            if (inertial)
            {
                link.remove_child(inertial);
            }
            inertial = link.append_child("inertial");
            inertial.append_child("origin").append_attribute("xyz") = "0 0 0";
            inertial.append_child("mass").append_attribute("value") = "1";
            pugi::xml_node inertia = inertial.append_child("inertia");
            inertia.append_attribute("ixx") = "0.01";
            inertia.append_attribute("ixy") = "0";
            inertia.append_attribute("ixz") = "0";
            inertia.append_attribute("iyy") = "0.01";
            inertia.append_attribute("iyz") = "0";
            inertia.append_attribute("izz") = "0.01";
            changed = true;
        }
    }

    // Mesh paths are relative to the URDF. The copy lives in a temp directory,
    // so they become absolute.
    for (pugi::xpath_node found : document.select_nodes("//mesh"))
    {
        pugi::xml_attribute filename = found.node().attribute("filename");
        std::string const value = filename.as_string();
        if (value.empty() || value.find("://") != std::string::npos ||
            value.front() == '/')
        {
            continue;
        }
        filename.set_value(
            (directory / value).lexically_normal().string().c_str());
        changed = true;
    }

    if (!changed)
    {
        return p_filename;
    }

    // One file per instance: parallel environments load the same model.
    static std::atomic<unsigned> s_counter{ 0u };
    std::filesystem::path const output =
        std::filesystem::temp_directory_path() /
        ("robotik-" + std::to_string(::getpid()) + "-" +
         std::to_string(s_counter++) + "-" + p_filename.filename().string());
    if (!document.save_file(output.c_str()))
    {
        throw std::runtime_error("Cannot write a MuJoCo URDF copy to " +
                                 output.string());
    }
    return output;
}

MujocoBackend::MujocoBackend(std::filesystem::path const& p_urdf)
    : m_impl(std::make_unique<Impl>())
{
    std::filesystem::path const source = withInertia(p_urdf);
    if (source != p_urdf)
    {
        m_impl->generated = source;
    }
    char error[1024] = {};
    m_impl->model = mj_loadXML(source.c_str(), nullptr, error, sizeof(error));
    if (m_impl->model == nullptr)
    {
        throw std::runtime_error("MuJoCo failed to load '" + p_urdf.string() +
                                 "': " + error);
    }
    // Reflected rotor inertia and viscous friction of real gear motors. Without
    // them a light wrist makes the explicit PD loop unstable at 1 ms.
    for (int dof = 0; dof < m_impl->model->nv; ++dof)
    {
        m_impl->model->dof_armature[dof] =
            std::max(m_impl->model->dof_armature[dof], 0.1);
        m_impl->model->dof_damping[dof] =
            std::max(m_impl->model->dof_damping[dof], 0.5);
    }
    m_impl->data = mj_makeData(m_impl->model);
}

MujocoBackend::~MujocoBackend() = default;

void MujocoBackend::attach(Robot& p_robot)
{
    mjModel const* model = m_impl->model;
    JointSet const& joints = p_robot.joints();
    m_bindings.assign(joints.size(), Binding{});
    for (JointId id = 0; id < joints.size(); ++id)
    {
        char const* name = joints.name(id).c_str();
        int const joint = mj_name2id(model, mjOBJ_JOINT, name);
        if (joint < 0)
        {
            continue;
        }
        m_bindings[id].qpos = model->jnt_qposadr[joint];
        m_bindings[id].dof = model->jnt_dofadr[joint];
        m_bindings[id].actuator = mj_name2id(model, mjOBJ_ACTUATOR, name);
    }
}

void MujocoBackend::reset(Robot& p_robot)
{
    mj_resetData(m_impl->model, m_impl->data);
    JointSet const& joints = p_robot.joints();
    for (JointId id = 0; id < m_bindings.size(); ++id)
    {
        if (m_bindings[id].qpos >= 0)
        {
            m_impl->data->qpos[m_bindings[id].qpos] = joints.position(id);
        }
    }
    mj_forward(m_impl->model, m_impl->data);
}

void MujocoBackend::step(Robot& p_robot, Seconds p_dt)
{
    mjModel* model = m_impl->model;
    mjData* data = m_impl->data;
    JointSet& joints = p_robot.joints();

    // The PD loop runs at the physics rate, not at the caller rate.
    int const steps = std::clamp(
        static_cast<int>(std::lround(p_dt.value() / m_timestep.value())), 1, 50);
    model->opt.timestep = m_timestep.value();

    for (int i = 0; i < steps; ++i)
    {
        joints.control(m_timestep);

        // qfrc_bias holds gravity and Coriolis at the last mj_forward state.
        mju_copy(data->qfrc_applied, data->qfrc_bias, static_cast<int>(model->nv));
        if (model->nu > 0)
        {
            mju_zero(data->ctrl, static_cast<int>(model->nu));
        }
        for (JointId id = 0; id < m_bindings.size(); ++id)
        {
            Binding const& binding = m_bindings[id];
            if (binding.actuator >= 0)
            {
                data->ctrl[binding.actuator] = joints.effort(id);
            }
            else if (binding.dof >= 0)
            {
                data->qfrc_applied[binding.dof] += joints.effort(id);
            }
        }

        mj_step(model, data);

        for (JointId id = 0; id < m_bindings.size(); ++id)
        {
            Binding const& binding = m_bindings[id];
            if (binding.qpos >= 0)
            {
                joints.measure(id, data->qpos[binding.qpos], data->qvel[binding.dof]);
            }
        }
    }
}

int MujocoBackend::contacts() const
{
    return m_impl->data->ncon;
}

} // namespace robotik
