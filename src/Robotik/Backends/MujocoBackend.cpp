#include "Robotik/Backends/MujocoBackend.hpp"

#include <mujoco/mujoco.h>

#include <pugixml.hpp>

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace robotik
{

struct MujocoBackend::Impl
{
    mjModel* model = nullptr;
    mjData* data = nullptr;
    std::filesystem::path generated;
};

namespace
{

// MuJoCo rejects a moving body whose mass or inertia is missing or ~0.
// URDF visuals often omit <inertial>. A small default keeps the same file
// loadable without changing what Pinocchio or Compages see.
std::filesystem::path withInertia(std::string const& p_filename)
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
            inertial ? inertial.child("mass").attribute("value").as_double(0.0) : 0.0;
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
        filename.set_value((directory / value).lexically_normal().string().c_str());
        changed = true;
    }

    if (!changed)
    {
        return p_filename;
    }

    std::filesystem::path const output =
        std::filesystem::temp_directory_path() /
        ("robotik-" + std::filesystem::path(p_filename).filename().string());
    if (!document.save_file(output.c_str()))
    {
        throw std::runtime_error("Cannot write a MuJoCo URDF copy to " +
                                 output.string());
    }
    return output;
}

} // namespace

MujocoBackend::MujocoBackend(std::string const& p_filename)
    : m_impl(new Impl)
{
    std::filesystem::path const source = withInertia(p_filename);
    if (source != std::filesystem::path(p_filename))
    {
        m_impl->generated = source;
    }
    char error[1024] = {};
    m_impl->model = mj_loadXML(source.c_str(), nullptr, error, sizeof(error));
    if (m_impl->model == nullptr)
    {
        std::string message = error;
        delete m_impl;
        m_impl = nullptr;
        throw std::runtime_error("MuJoCo failed to load '" + p_filename + "': " +
                                 message);
    }
    // Reflected rotor inertia and viscous friction of real gear motors. Without
    // them a light wrist makes the explicit PD loop unstable at 1 ms.
    for (int dof = 0; dof < m_impl->model->nv; ++dof)
    {
        m_impl->model->dof_armature[dof] = std::max(m_impl->model->dof_armature[dof], 0.1);
        m_impl->model->dof_damping[dof] = std::max(m_impl->model->dof_damping[dof], 0.5);
    }
    m_impl->data = mj_makeData(m_impl->model);
    reset();
}

MujocoBackend::~MujocoBackend()
{
    if (m_impl == nullptr)
    {
        return;
    }
    mj_deleteData(m_impl->data);
    mj_deleteModel(m_impl->model);
    if (!m_impl->generated.empty())
    {
        std::error_code ignored;
        std::filesystem::remove(m_impl->generated, ignored);
    }
    delete m_impl;
}

void MujocoBackend::reset()
{
    mj_resetData(m_impl->model, m_impl->data);
    mj_forward(m_impl->model, m_impl->data);
    m_time = m_impl->data->time;
}

void MujocoBackend::step(double p_dt)
{
    if (p_dt > 0.0)
    {
        m_impl->model->opt.timestep = p_dt;
    }
    mj_step(m_impl->model, m_impl->data);
    m_time = m_impl->data->time;
}

double MujocoBackend::time() const
{
    return m_time;
}

int MujocoBackend::jointId(std::string const& p_name) const
{
    return mj_name2id(m_impl->model, mjOBJ_JOINT, p_name.c_str());
}

int MujocoBackend::qposIndex(int p_joint_id) const
{
    if (p_joint_id < 0)
    {
        return -1;
    }
    return m_impl->model->jnt_qposadr[p_joint_id];
}

int MujocoBackend::qvelIndex(int p_joint_id) const
{
    if (p_joint_id < 0)
    {
        return -1;
    }
    return m_impl->model->jnt_dofadr[p_joint_id];
}

int MujocoBackend::dofIndex(int p_joint_id) const
{
    return qvelIndex(p_joint_id);
}

int MujocoBackend::actuatorId(std::string const& p_name) const
{
    return mj_name2id(m_impl->model, mjOBJ_ACTUATOR, p_name.c_str());
}

double MujocoBackend::qpos(int p_index) const
{
    return m_impl->data->qpos[p_index];
}

double MujocoBackend::qvel(int p_index) const
{
    return m_impl->data->qvel[p_index];
}

void MujocoBackend::clearAppliedForces()
{
    mju_zero(m_impl->data->qfrc_applied, static_cast<int>(m_impl->model->nv));
    if (m_impl->model->nu > 0)
    {
        mju_zero(m_impl->data->ctrl, static_cast<int>(m_impl->model->nu));
    }
}

void MujocoBackend::setCtrl(int p_actuator_id, double p_effort)
{
    if (p_actuator_id >= 0 && p_actuator_id < m_impl->model->nu)
    {
        m_impl->data->ctrl[p_actuator_id] = p_effort;
    }
}

void MujocoBackend::setQpos(int p_index, double p_value)
{
    if (p_index >= 0 && p_index < m_impl->model->nq)
    {
        m_impl->data->qpos[p_index] = p_value;
        mj_forward(m_impl->model, m_impl->data);
    }
}

void MujocoBackend::compensateGravity()
{
    // qfrc_bias holds gravity and Coriolis at the state of the last mj_forward.
    mju_addTo(m_impl->data->qfrc_applied, m_impl->data->qfrc_bias,
              static_cast<int>(m_impl->model->nv));
}

int MujocoBackend::contacts() const
{
    return m_impl->data->ncon;
}

void MujocoBackend::addQfrc(int p_dof_index, double p_effort)
{
    if (p_dof_index >= 0 && p_dof_index < m_impl->model->nv)
    {
        m_impl->data->qfrc_applied[p_dof_index] += p_effort;
    }
}

} // namespace robotik
