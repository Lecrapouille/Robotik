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
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <unistd.h>

#define ROTOR_ARMATURE 0.1
#define JOINT_DAMPING 0.5
#define MAX_SUBSTEPS 50
#define MAX_RAY_HOPS 16

namespace robotik
{

struct MujocoBackend::Impl
{
    struct Chain
    {
        std::string name;
        std::filesystem::path source;
        std::filesystem::path parsed;
    };

    //! @brief @c child_mount of @c child hangs on @c parent_mount of @c parent.
    struct Graft
    {
        std::string parent;
        std::string parent_mount;
        std::string child;
        std::string child_mount;
    };

    Chain* find(std::string const& p_name)
    {
        for (Chain& chain : chains)
        {
            if (chain.name == p_name)
            {
                return &chain;
            }
        }
        return nullptr;
    }

    mjModel* model = nullptr;
    mjData* data = nullptr;
    std::vector<Chain> chains;
    std::vector<Graft> grafts;
    //!< True until @ref MujocoBackend::compile has consumed @c chains.
    bool dirty = true;
    //!< Set by @ref MujocoBackend::attach(Robot&), so a later graft rebinds.
    Robot* robot = nullptr;
    //!< Root body of the first chain (child of the world body).
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
        for (Chain const& chain : chains)
        {
            if (chain.parsed != chain.source)
            {
                std::error_code ignored;
                std::filesystem::remove(chain.parsed, ignored);
            }
        }
    }
};

struct SpecDelete
{
    void operator()(mjSpec* p_spec) const
    {
        mj_deleteSpec(p_spec);
    }
};

using SpecPtr = std::unique_ptr<mjSpec, SpecDelete>;

static SpecPtr parseSpec(std::filesystem::path const& p_path)
{
    char error[1024] = {};
    mjSpec* const spec = mj_parseXML(p_path.c_str(), nullptr, error, sizeof(error));
    if (spec == nullptr)
    {
        throw std::runtime_error("MuJoCo failed to load '" + p_path.string() +
                                 "': " + error);
    }
    spec->compiler.fusestatic = 0;
    return SpecPtr{ spec };
}

//! A link name, or a joint name (the link that joint moves).
static mjsBody* findMount(mjSpec* p_spec, std::string const& p_name)
{
    if (mjsBody* const body = mjs_findBody(p_spec, p_name.c_str()))
    {
        return body;
    }
    mjsElement* const joint = mjs_findElement(p_spec, mjOBJ_JOINT, p_name.c_str());
    return joint != nullptr ? mjs_getParent(joint) : nullptr;
}

static std::vector<mjsBody*> rootBodies(mjSpec* p_spec)
{
    std::vector<mjsBody*> roots;
    mjsBody* const world = mjs_findBody(p_spec, "world");
    for (mjsElement* child = mjs_firstChild(world, mjOBJ_BODY, 0); child != nullptr;
         child = mjs_nextChild(world, child, 0))
    {
        roots.push_back(mjs_asBody(child));
    }
    return roots;
}

//! Hang @p_child on an identity frame of @p_parent. mjs_attach copies a body
//! onto a frame (the other way round is rejected).
static void hang(mjSpec* p_host, mjsBody* p_parent, mjsBody* p_child)
{
    mjsFrame* const frame = mjs_addFrame(p_parent, nullptr);
    if (frame == nullptr ||
        mjs_attach(frame->element, p_child->element, "", "") == nullptr)
    {
        throw std::runtime_error(std::string("MuJoCo failed to attach a chain: ") +
                                 mjs_getError(p_host));
    }
}

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
//! @p_root is the first chain's root link. Sibling chains hang on the world
//! too, so the free joint must not follow whichever body MuJoCo lists first.
static void edit(mjSpec* p_spec,
                 MujocoOptions const& p_options,
                 std::string const& p_root)
{
    // Fusing static bodies would merge links that skills and sensors name.
    p_spec->compiler.fusestatic = 0;

    mjsBody* world = mjs_findBody(p_spec, "world");
    mjsBody* root = mjs_findBody(p_spec, p_root.c_str());
    if (world == nullptr || root == nullptr)
    {
        throw std::runtime_error("MuJoCo: the URDF has no link");
    }
    if (p_options.floating_base)
    {
        mjsJoint* joint = mjs_addFreeJoint(root);
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

namespace
{

struct HeldPose
{
    std::unordered_map<std::string, double> qpos;
    double free_qpos[7] = {};
    bool has_free = false;
};

HeldPose holdPose(mjModel* p_model, mjData* p_data)
{
    HeldPose held;
    if (p_model == nullptr || p_data == nullptr)
    {
        return held;
    }
    for (int joint = 0; joint < p_model->njnt; ++joint)
    {
        char const* const name = mj_id2name(p_model, mjOBJ_JOINT, joint);
        if (name == nullptr)
        {
            continue;
        }
        int const address = p_model->jnt_qposadr[joint];
        if (p_model->jnt_type[joint] == mjJNT_HINGE ||
            p_model->jnt_type[joint] == mjJNT_SLIDE)
        {
            held.qpos.emplace(name, p_data->qpos[address]);
        }
        else if (p_model->jnt_type[joint] == mjJNT_FREE &&
                 std::string(name) == "robotik_base")
        {
            mju_copy(held.free_qpos, p_data->qpos + address, 7);
            held.has_free = true;
        }
    }
    return held;
}

void restorePose(mjModel* p_model, mjData* p_data, HeldPose const& p_held)
{
    for (int joint = 0; joint < p_model->njnt; ++joint)
    {
        char const* const name = mj_id2name(p_model, mjOBJ_JOINT, joint);
        if (name == nullptr)
        {
            continue;
        }
        int const address = p_model->jnt_qposadr[joint];
        if (p_model->jnt_type[joint] == mjJNT_HINGE ||
            p_model->jnt_type[joint] == mjJNT_SLIDE)
        {
            if (auto const found = p_held.qpos.find(name);
                found != p_held.qpos.end())
            {
                p_data->qpos[address] = found->second;
            }
        }
        else if (p_held.has_free && p_model->jnt_type[joint] == mjJNT_FREE &&
                 std::string(name) == "robotik_base")
        {
            mju_copy(p_data->qpos + address, p_held.free_qpos, 7);
        }
    }
}

std::string urdfRobotName(std::filesystem::path const& p_urdf)
{
    pugi::xml_document document;
    pugi::xml_parse_result const parsed = document.load_file(p_urdf.c_str());
    if (!parsed)
    {
        throw std::runtime_error("Failed to parse URDF '" + p_urdf.string() +
                                 "': " + parsed.description());
    }
    std::string name = document.child("robot").attribute("name").as_string();
    if (name.empty())
    {
        name = p_urdf.stem().string();
    }
    return name;
}

} // namespace

MujocoBackend::MujocoBackend(MujocoOptions p_options)
    : m_impl(std::make_unique<Impl>()), m_options(std::move(p_options))
{
}

MujocoBackend::~MujocoBackend() = default;

std::string MujocoBackend::load(std::filesystem::path const& p_urdf)
{
    std::filesystem::path const parsed = withInertia(p_urdf);
    try
    {
        SpecPtr const spec = parseSpec(parsed);
        (void)spec;
    }
    catch (...)
    {
        if (parsed != p_urdf)
        {
            std::error_code ignored;
            std::filesystem::remove(parsed, ignored);
        }
        throw;
    }

    std::string name = urdfRobotName(p_urdf);
    if (m_impl->find(name) != nullptr)
    {
        for (int index = 2;; ++index)
        {
            std::string candidate = name + "-" + std::to_string(index);
            if (m_impl->find(candidate) == nullptr)
            {
                name = std::move(candidate);
                break;
            }
        }
    }
    m_impl->chains.push_back(Impl::Chain{ name, p_urdf, parsed });
    invalidate();
    return name;
}

void MujocoBackend::attach(std::string const& p_robot1,
                           std::string const& p_joint1,
                           std::string const& p_robot2,
                           std::string const& p_joint2)
{
    if (p_robot1 == p_robot2)
    {
        throw std::runtime_error("MuJoCo: a chain cannot be attached to itself");
    }
    Impl::Chain* const parent = m_impl->find(p_robot1);
    Impl::Chain* const child = m_impl->find(p_robot2);
    if (parent == nullptr || child == nullptr)
    {
        throw std::runtime_error("MuJoCo: no kinematic chain '" +
                                 (parent == nullptr ? p_robot1 : p_robot2) + "'");
    }
    for (Impl::Graft const& graft : m_impl->grafts)
    {
        if (graft.child == p_robot2)
        {
            throw std::runtime_error("MuJoCo: '" + p_robot2 +
                                     "' is already attached");
        }
    }
    SpecPtr const parent_spec = parseSpec(parent->parsed);
    SpecPtr const child_spec = parseSpec(child->parsed);
    if (findMount(parent_spec.get(), p_joint1) == nullptr)
    {
        throw std::runtime_error("MuJoCo: '" + p_robot1 +
                                 "' has no link or joint '" + p_joint1 + "'");
    }
    if (findMount(child_spec.get(), p_joint2) == nullptr)
    {
        throw std::runtime_error("MuJoCo: '" + p_robot2 +
                                 "' has no link or joint '" + p_joint2 + "'");
    }
    m_impl->grafts.push_back(
        Impl::Graft{ p_robot1, p_joint1, p_robot2, p_joint2 });
    invalidate();
}

void MujocoBackend::detach(std::string const& p_robot1,
                           std::string const& p_joint1,
                           std::string const& p_robot2,
                           std::string const& p_joint2)
{
    std::vector<Impl::Graft>& grafts = m_impl->grafts;
    auto const found = std::find_if(
        grafts.begin(), grafts.end(), [&](Impl::Graft const& p_graft) {
            return p_graft.parent == p_robot1 && p_graft.parent_mount == p_joint1 &&
                   p_graft.child == p_robot2 && p_graft.child_mount == p_joint2;
        });
    if (found == grafts.end())
    {
        throw std::runtime_error("MuJoCo: '" + p_robot2 +
                                 "' is not attached to '" + p_robot1 + "'");
    }
    grafts.erase(found);
    invalidate();
}

void MujocoBackend::invalidate()
{
    m_impl->dirty = true;
    if (m_impl->model != nullptr)
    {
        compile();
        if (m_impl->robot != nullptr)
        {
            bind(*m_impl->robot);
        }
    }
}

void MujocoBackend::compile()
{
    if (m_impl->chains.empty())
    {
        throw std::runtime_error("MuJoCo: no kinematic chain loaded");
    }

    HeldPose const held = holdPose(m_impl->model, m_impl->data);

    std::unordered_set<std::string> children;
    for (Impl::Graft const& graft : m_impl->grafts)
    {
        children.insert(graft.child);
    }
    Impl::Chain* host_chain = nullptr;
    for (Impl::Chain& chain : m_impl->chains)
    {
        if (!children.contains(chain.name))
        {
            host_chain = &chain;
            break;
        }
    }
    if (host_chain == nullptr)
    {
        throw std::runtime_error("MuJoCo: the kinematic chains form a cycle");
    }

    std::unordered_map<std::string, SpecPtr> specs;
    for (Impl::Chain const& chain : m_impl->chains)
    {
        specs.emplace(chain.name, parseSpec(chain.parsed));
    }
    mjSpec* const host = specs.at(host_chain->name).get();
    std::vector<mjsBody*> const host_roots = rootBodies(host);
    if (host_roots.empty())
    {
        throw std::runtime_error("MuJoCo: the URDF has no link");
    }
    std::string const host_root =
        mjs_getString(mjs_getName(host_roots.front()->element));
    if (mjs_setDeepCopy(host, 1) != 0)
    {
        throw std::runtime_error(std::string("MuJoCo failed to copy a chain: ") +
                                 mjs_getError(host));
    }
    for (Impl::Chain const& chain : m_impl->chains)
    {
        if (chain.name == host_chain->name || children.contains(chain.name))
        {
            continue;
        }
        mjsBody* const world = mjs_findBody(host, "world");
        for (mjsBody* root : rootBodies(specs.at(chain.name).get()))
        {
            hang(host, world, root);
        }
    }

    std::unordered_set<std::string> grafted;
    bool progress = true;
    while (progress)
    {
        progress = false;
        for (Impl::Graft const& graft : m_impl->grafts)
        {
            if (grafted.contains(graft.child))
            {
                continue;
            }
            bool const parent_ready = graft.parent == host_chain->name ||
                                      !children.contains(graft.parent) ||
                                      grafted.contains(graft.parent);
            if (!parent_ready)
            {
                continue;
            }
            mjsBody* const parent = findMount(host, graft.parent_mount);
            mjsBody* const child =
                findMount(specs.at(graft.child).get(), graft.child_mount);
            if (parent == nullptr)
            {
                throw std::runtime_error("MuJoCo: '" + graft.parent +
                                         "' has no link or joint '" +
                                         graft.parent_mount + "'");
            }
            if (child == nullptr)
            {
                throw std::runtime_error("MuJoCo: '" + graft.child +
                                         "' has no link or joint '" +
                                         graft.child_mount + "'");
            }
            hang(host, parent, child);
            grafted.insert(graft.child);
            progress = true;
        }
    }
    if (grafted.size() != m_impl->grafts.size())
    {
        throw std::runtime_error("MuJoCo: the kinematic chains form a cycle");
    }

    edit(host, m_options, host_root);
    mjModel* const fresh = mj_compile(host, nullptr);
    std::string const compile_error = mjs_getError(host);
    if (fresh == nullptr)
    {
        throw std::runtime_error("MuJoCo failed to compile the kinematic chains: " +
                                 compile_error);
    }
    specs.clear();

    mj_deleteData(m_impl->data);
    mj_deleteModel(m_impl->model);
    m_impl->data = nullptr;
    m_impl->model = fresh;
    m_impl->root = -1;
    m_impl->free_qpos = -1;
    m_impl->free_dof = -1;

    // Reflected rotor inertia and viscous friction of real gear motors. Without
    // them a light wrist makes the explicit PD loop unstable at 1 ms. The free
    // joint of a floating base is not a motor.
    for (int joint = 0; joint < fresh->njnt; ++joint)
    {
        if (fresh->jnt_type[joint] != mjJNT_HINGE &&
            fresh->jnt_type[joint] != mjJNT_SLIDE)
        {
            continue;
        }
        int const dof = fresh->jnt_dofadr[joint];
        fresh->dof_armature[dof] =
            std::max(fresh->dof_armature[dof], ROTOR_ARMATURE);
        fresh->dof_damping[dof] =
            std::max(fresh->dof_damping[dof], JOINT_DAMPING);
    }
    m_impl->root = mj_name2id(fresh, mjOBJ_BODY, host_root.c_str());
    if (int const joint = mj_name2id(fresh, mjOBJ_JOINT, "robotik_base");
        joint >= 0)
    {
        m_impl->free_qpos = fresh->jnt_qposadr[joint];
        m_impl->free_dof = fresh->jnt_dofadr[joint];
    }
    m_impl->floor = mj_name2id(fresh, mjOBJ_GEOM, "robotik_floor");
    m_impl->data = mj_makeData(fresh);
    restorePose(fresh, m_impl->data, held);
    mj_forward(fresh, m_impl->data);
    m_impl->stale_wrenches = true;
    m_impl->dirty = false;
}

void MujocoBackend::attach(Robot& p_robot)
{
    m_impl->robot = &p_robot;
    if (m_impl->model == nullptr || m_impl->dirty)
    {
        compile();
    }
    bind(p_robot);
}

void MujocoBackend::bind(Robot& p_robot)
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
