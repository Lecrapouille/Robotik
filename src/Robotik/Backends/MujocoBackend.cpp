// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Backends/MujocoBackend.hpp"

#include "Robotik/Robot/Robot.hpp"

#include <mujoco/mujoco.h>

#include <pugixml.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

#include <unistd.h>

#define ROTOR_ARMATURE 0.1
#define JOINT_DAMPING 0.5
#define MAX_SUBSTEPS 50
#define MAX_RAY_HOPS 16

namespace robotik
{

struct MujocoBackend::Impl
{
    mjModel* model = nullptr;
    mjData* data = nullptr;
    std::filesystem::path generated;
    //!< Root body of the robot (child of the world body).
    int root = -1;
    //!< First qpos / DOF of the free joint, or -1 for a fixed base.
    int free_qpos = -1;
    int free_dof = -1;
    //!< Floor geom, or -1.
    int floor = -1;
    //!< True when cfrc_int is older than the last step.
    bool stale_wrenches = true;

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

//! Edits the parsed URDF: free joint, floor, friction, no body fusion.
static void edit(mjSpec* p_spec, MujocoOptions const& p_options)
{
    // Fusing static bodies would merge links that skills and sensors name.
    p_spec->compiler.fusestatic = 0;

    mjsBody* world = mjs_findBody(p_spec, "world");
    mjsElement* first = mjs_firstChild(world, mjOBJ_BODY, 0);
    if (first == nullptr)
    {
        throw std::runtime_error("MuJoCo: the URDF has no link");
    }
    if (p_options.floating_base)
    {
        mjsJoint* joint = mjs_addFreeJoint(mjs_asBody(first));
        mjs_setName(joint->element, "robotik_base");
    }
    if (p_options.floor)
    {
        mjsGeom* floor = mjs_addGeom(world, nullptr);
        floor->type = mjGEOM_PLANE;
        floor->size[0] = floor->size[1] = 0.0;
        floor->size[2] = 1.0;
        mjs_setName(floor->element, "robotik_floor");
    }
    for (auto const& [link, friction] : p_options.friction)
    {
        mjsBody* body = mjs_findBody(p_spec, link.c_str());
        if (body == nullptr)
        {
            throw std::runtime_error("MuJoCo: no link '" + link +
                                     "' for the friction option");
        }
        for (mjsElement* geom = mjs_firstChild(body, mjOBJ_GEOM, 0);
             geom != nullptr;
             geom = mjs_nextChild(body, geom, 0))
        {
            mjs_asGeom(geom)->friction[0] = friction;
        }
    }
}

MujocoBackend::MujocoBackend(std::filesystem::path const& p_urdf,
                             MujocoOptions p_options)
    : m_impl(std::make_unique<Impl>()), m_options(std::move(p_options))
{
    std::filesystem::path const source = withInertia(p_urdf);
    if (source != p_urdf)
    {
        m_impl->generated = source;
    }
    char error[1024] = {};
    mjSpec* spec = mj_parseXML(source.c_str(), nullptr, error, sizeof(error));
    if (spec == nullptr)
    {
        throw std::runtime_error("MuJoCo failed to load '" + p_urdf.string() +
                                 "': " + error);
    }
    try
    {
        edit(spec, m_options);
    }
    catch (...)
    {
        mj_deleteSpec(spec);
        throw;
    }
    m_impl->model = mj_compile(spec, nullptr);
    std::string const compile_error = mjs_getError(spec);
    mj_deleteSpec(spec);
    if (m_impl->model == nullptr)
    {
        throw std::runtime_error("MuJoCo failed to compile '" +
                                 p_urdf.string() + "': " + compile_error);
    }

    mjModel* model = m_impl->model;
    // Reflected rotor inertia and viscous friction of real gear motors. Without
    // them a light wrist makes the explicit PD loop unstable at 1 ms. The free
    // joint of a floating base is not a motor.
    for (int joint = 0; joint < model->njnt; ++joint)
    {
        if (model->jnt_type[joint] != mjJNT_HINGE &&
            model->jnt_type[joint] != mjJNT_SLIDE)
        {
            continue;
        }
        int const dof = model->jnt_dofadr[joint];
        model->dof_armature[dof] =
            std::max(model->dof_armature[dof], ROTOR_ARMATURE);
        model->dof_damping[dof] = std::max(model->dof_damping[dof], JOINT_DAMPING);
    }
    for (int body = 1; body < model->nbody; ++body)
    {
        if (model->body_parentid[body] == 0)
        {
            m_impl->root = body;
            break;
        }
    }
    if (int const joint = mj_name2id(model, mjOBJ_JOINT, "robotik_base");
        joint >= 0)
    {
        m_impl->free_qpos = model->jnt_qposadr[joint];
        m_impl->free_dof = model->jnt_dofadr[joint];
    }
    m_impl->floor = mj_name2id(model, mjOBJ_GEOM, "robotik_floor");
    m_impl->data = mj_makeData(model);
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
    mjModel* model = m_impl->model;
    mjData* data = m_impl->data;
    mj_resetData(model, data);

    Pose const& base = p_robot.base().pose;
    double const position[3] = { base.position.x, base.position.y,
                                 base.position.z };
    double const quaternion[4] = { base.rotation.w, base.rotation.x,
                                   base.rotation.y, base.rotation.z };
    if (m_impl->free_qpos >= 0)
    {
        mju_copy(data->qpos + m_impl->free_qpos, position, 3);
        mju_copy(data->qpos + m_impl->free_qpos + 3, quaternion, 4);
    }
    else if (m_impl->root >= 0)
    {
        mju_copy(model->body_pos + 3 * m_impl->root, position, 3);
        mju_copy(model->body_quat + 4 * m_impl->root, quaternion, 4);
    }

    JointSet const& joints = p_robot.joints();
    for (JointId id = 0; id < m_bindings.size(); ++id)
    {
        if (m_bindings[id].qpos >= 0)
        {
            data->qpos[m_bindings[id].qpos] = joints.position(id);
        }
    }
    mj_forward(model, data);
    m_impl->stale_wrenches = true;
}

void MujocoBackend::step(Robot& p_robot, Seconds p_dt)
{
    mjModel* model = m_impl->model;
    mjData* data = m_impl->data;
    JointSet& joints = p_robot.joints();

    // The joint loops run at the physics rate, not at the caller rate.
    double const timestep = m_options.timestep.value();
    int const steps = std::clamp(
        static_cast<int>(std::lround(p_dt.value() / timestep)), 1, MAX_SUBSTEPS);
    model->opt.timestep = timestep;

    for (int i = 0; i < steps; ++i)
    {
        joints.control(m_options.timestep);

        // qfrc_bias holds gravity and Coriolis at the last mj_forward state.
        // Only the actuated joints are compensated: a floating base falls.
        mju_zero(data->qfrc_applied, static_cast<int>(model->nv));
        if (model->nu > 0)
        {
            mju_zero(data->ctrl, static_cast<int>(model->nu));
        }
        for (JointId id = 0; id < m_bindings.size(); ++id)
        {
            Binding const& binding = m_bindings[id];
            if (binding.dof < 0)
            {
                continue;
            }
            data->qfrc_applied[binding.dof] = data->qfrc_bias[binding.dof];
            if (binding.actuator >= 0)
            {
                data->ctrl[binding.actuator] = joints.effort(id);
            }
            else
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

    if (m_impl->free_qpos >= 0)
    {
        double const* q = data->qpos + m_impl->free_qpos;
        double const* v = data->qvel + m_impl->free_dof;
        BaseState base;
        base.pose.position = Vector3(q[0], q[1], q[2]);
        base.pose.rotation = Quaternion(q[3], q[4], q[5], q[6]).normalized();
        base.twist.linear = Vector3(v[0], v[1], v[2]);
        base.twist.angular = base.pose.rotation * Vector3(v[3], v[4], v[5]);
        p_robot.measureBase(base);
    }
    m_impl->stale_wrenches = true;
}

int MujocoBackend::contacts() const
{
    mjData const* data = m_impl->data;
    int count = 0;
    for (int i = 0; i < data->ncon; ++i)
    {
        mjContact const& contact = data->contact[i];
        if (contact.geom[0] != m_impl->floor && contact.geom[1] != m_impl->floor)
        {
            ++count;
        }
    }
    return count;
}

std::optional<double> MujocoBackend::raycast(Vector3 const& p_origin,
                                             Vector3 const& p_direction,
                                             double p_max) const
{
    mjModel const* model = m_impl->model;
    mjData const* data = m_impl->data;
    double origin[3] = { p_origin.x, p_origin.y, p_origin.z };
    double const direction[3] = { p_direction.x, p_direction.y, p_direction.z };
    double travelled = 0.0;
    for (int hop = 0; hop < MAX_RAY_HOPS; ++hop)
    {
        int geom = -1;
        double const distance =
            mj_ray(model, data, origin, direction, nullptr, 1, -1, &geom, nullptr);
        if (distance < 0.0 || travelled + distance > p_max)
        {
            return std::nullopt;
        }
        int const body = model->geom_bodyid[geom];
        if (m_impl->root < 0 || model->body_rootid[body] != m_impl->root)
        {
            return travelled + distance;
        }
        // Robot geom: continue the ray behind it.
        double const skip = distance + 1e-4;
        travelled += skip;
        for (int k = 0; k < 3; ++k)
        {
            origin[k] += skip * direction[k];
        }
    }
    return std::nullopt;
}

std::optional<Wrench> MujocoBackend::wrench(std::string const& p_link) const
{
    mjModel const* model = m_impl->model;
    mjData* data = m_impl->data;
    int const body = mj_name2id(model, mjOBJ_BODY, p_link.c_str());
    if (body < 0)
    {
        return std::nullopt;
    }
    if (m_impl->stale_wrenches)
    {
        mj_rnePostConstraint(model, data);
        m_impl->stale_wrenches = false;
    }
    // Same transform as the MuJoCo force and torque sensors.
    double spatial[6];
    mju_transformSpatial(spatial,
                         data->cfrc_int + 6 * body,
                         1,
                         data->xpos + 3 * body,
                         data->subtree_com + 3 * model->body_rootid[body],
                         data->xmat + 9 * body);
    Wrench result;
    result.torque = Vector3(spatial[0], spatial[1], spatial[2]);
    result.force = Vector3(spatial[3], spatial[4], spatial[5]);
    return result;
}

} // namespace robotik
