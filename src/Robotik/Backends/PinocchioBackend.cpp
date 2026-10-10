// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Backends/PinocchioBackend.hpp"

// Pinocchio's joint variant has more alternatives than Boost.MPL's default
// list. These have to be set before any Boost header.
#ifndef BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#    define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif
#ifndef BOOST_MPL_LIMIT_LIST_SIZE
#    define BOOST_MPL_LIMIT_LIST_SIZE 30
#endif
#ifndef BOOST_MPL_LIMIT_VECTOR_SIZE
#    define BOOST_MPL_LIMIT_VECTOR_SIZE 30
#endif
#ifndef BOOST_VARIANT_LIMIT_TYPES
#    define BOOST_VARIANT_LIMIT_TYPES 30
#endif

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial.hpp>

#include <Eigen/Core>

#include <memory>
#include <stdexcept>

#define IK_MAX_ITERATIONS 500
#define IK_STEP 0.5
#define IK_DAMPING 1e-4
#define IK_TOLERANCE 1e-4

namespace robotik
{

struct PinocchioBackend::Impl
{
    pinocchio::Model model;
    std::unique_ptr<pinocchio::Data> data;
    Eigen::VectorXd q;
    Eigen::VectorXd v;
};

static std::vector<double> toVector(Eigen::VectorXd const& p_value)
{
    return {p_value.data(), p_value.data() + p_value.size()};
}

//! Pose of tcp in the tool_mount frame. The mount itself is the identity
//! on the robot flange, so this is also the pose of tcp on the flange.
static pinocchio::SE3 tcpInMount(std::filesystem::path const& p_tool)
{
    pinocchio::Model tool;
    pinocchio::urdf::buildModel(p_tool.string(), tool);
    if (!tool.existFrame("tool_mount") || !tool.existFrame("tcp"))
    {
        throw std::runtime_error("Tool '" + p_tool.string() +
                                 "' needs frames tool_mount and tcp");
    }
    pinocchio::Data data(tool);
    Eigen::VectorXd const q = pinocchio::neutral(tool);
    pinocchio::forwardKinematics(tool, data, q);
    pinocchio::updateFramePlacements(tool, data);
    return data.oMf[tool.getFrameId("tool_mount")].inverse() *
           data.oMf[tool.getFrameId("tcp")];
}

PinocchioBackend::PinocchioBackend(std::filesystem::path const& p_urdf,
                                   std::filesystem::path const& p_tool)
    : m_impl(std::make_unique<Impl>())
{
    pinocchio::urdf::buildModel(p_urdf.string(), m_impl->model);
    if (!p_tool.empty())
    {
        if (!m_impl->model.existFrame("flange"))
        {
            throw std::runtime_error("Robot '" + p_urdf.string() +
                                     "' has no flange frame");
        }
        pinocchio::SE3 const tcp = tcpInMount(p_tool);
        pinocchio::FrameIndex const flange_id =
            m_impl->model.getFrameId("flange");
        pinocchio::Frame const& flange = m_impl->model.frames[flange_id];
        pinocchio::JointIndex const parent = flange.parentJoint;
        pinocchio::SE3 const placement = flange.placement * tcp;
        m_impl->model.addFrame(pinocchio::Frame("tcp",
                                               parent,
                                               flange_id,
                                               placement,
                                               pinocchio::OP_FRAME),
                              false);
    }
    m_impl->data = std::make_unique<pinocchio::Data>(m_impl->model);
    m_impl->q = pinocchio::neutral(m_impl->model);
    m_impl->v = Eigen::VectorXd::Zero(m_impl->model.nv);
    updateKinematics();
}

PinocchioBackend::~PinocchioBackend() = default;

std::size_t PinocchioBackend::nq() const
{
    return static_cast<std::size_t>(m_impl->model.nq);
}

std::size_t PinocchioBackend::nv() const
{
    return static_cast<std::size_t>(m_impl->model.nv);
}

std::span<double> PinocchioBackend::configuration()
{
    return { m_impl->q.data(), static_cast<std::size_t>(m_impl->q.size()) };
}

std::span<double const> PinocchioBackend::configuration() const
{
    return { m_impl->q.data(), static_cast<std::size_t>(m_impl->q.size()) };
}

std::span<double> PinocchioBackend::velocity()
{
    return { m_impl->v.data(), static_cast<std::size_t>(m_impl->v.size()) };
}

std::span<double const> PinocchioBackend::velocity() const
{
    return { m_impl->v.data(), static_cast<std::size_t>(m_impl->v.size()) };
}

void PinocchioBackend::updateKinematics()
{
    pinocchio::forwardKinematics(m_impl->model, *m_impl->data, m_impl->q, m_impl->v);
    pinocchio::updateFramePlacements(m_impl->model, *m_impl->data);
}

bool PinocchioBackend::hasJoint(std::string const& p_name) const
{
    return m_impl->model.existJointName(p_name);
}

std::size_t PinocchioBackend::jointId(std::string const& p_name) const
{
    return static_cast<std::size_t>(m_impl->model.getJointId(p_name));
}

int PinocchioBackend::qIndex(std::string const& p_name) const
{
    if (!hasJoint(p_name))
    {
        return -1;
    }
    auto const id = m_impl->model.getJointId(p_name);
    if (m_impl->model.nqs[id] != 1)
    {
        return -1;
    }
    return m_impl->model.idx_qs[id];
}

int PinocchioBackend::vIndex(std::string const& p_name) const
{
    if (!hasJoint(p_name))
    {
        return -1;
    }
    auto const id = m_impl->model.getJointId(p_name);
    if (m_impl->model.nvs[id] != 1)
    {
        return -1;
    }
    return m_impl->model.idx_vs[id];
}

bool PinocchioBackend::hasFrame(std::string const& p_name) const
{
    return m_impl->model.existFrame(p_name);
}

std::size_t PinocchioBackend::frameId(std::string const& p_name) const
{
    return static_cast<std::size_t>(m_impl->model.getFrameId(p_name));
}

Pose PinocchioBackend::framePose(std::string const& p_frame) const
{
    if (!hasFrame(p_frame))
    {
        throw std::invalid_argument("Unknown Pinocchio frame: " + p_frame);
    }
    auto const id = m_impl->model.getFrameId(p_frame);
    pinocchio::SE3 const& placement = m_impl->data->oMf[id];
    Eigen::Quaterniond const rotation(placement.rotation());
    return Pose{ { placement.translation().x(),
                   placement.translation().y(),
                   placement.translation().z() },
                 { rotation.w(), rotation.x(), rotation.y(), rotation.z() } };
}

std::optional<std::vector<double>>
PinocchioBackend::solveIK(std::string const& p_frame,
                          Pose const& p_target,
                          std::span<double const> p_seed) const
{
    if (!hasFrame(p_frame))
    {
        return std::nullopt;
    }

    Eigen::VectorXd q = m_impl->q;
    if (p_seed.size() == nq())
    {
        q = Eigen::Map<Eigen::VectorXd const>(p_seed.data(),
                                              static_cast<Eigen::Index>(p_seed.size()));
    }

    Eigen::Quaterniond rotation(p_target.rotation.w,
                                p_target.rotation.x,
                                p_target.rotation.y,
                                p_target.rotation.z);
    if (rotation.norm() < 1e-12)
    {
        rotation = Eigen::Quaterniond::Identity();
    }
    rotation.normalize();
    pinocchio::SE3 const oMdes(rotation.toRotationMatrix(),
                               Eigen::Vector3d(p_target.position.x,
                                               p_target.position.y,
                                               p_target.position.z));

    auto const frame = m_impl->model.getFrameId(p_frame);
    pinocchio::Data data(m_impl->model);
    Eigen::MatrixXd jacobian(6, m_impl->model.nv);

    for (int iteration = 0; iteration < IK_MAX_ITERATIONS; ++iteration)
    {
        pinocchio::forwardKinematics(m_impl->model, data, q);
        pinocchio::updateFramePlacements(m_impl->model, data);

        pinocchio::SE3 const iMd = data.oMf[frame].actInv(oMdes);
        Eigen::Matrix<double, 6, 1> const error = pinocchio::log6(iMd).toVector();
        if (error.norm() < IK_TOLERANCE)
        {
            return toVector(q);
        }

        pinocchio::computeFrameJacobian(
            m_impl->model, data, q, frame, pinocchio::LOCAL, jacobian);
        pinocchio::Data::Matrix6 jlog;
        pinocchio::Jlog6(iMd.inverse(), jlog);
        jacobian = -jlog * jacobian;

        Eigen::Matrix<double, 6, 6> jjt = jacobian * jacobian.transpose();
        jjt.diagonal().array() += IK_DAMPING;
        Eigen::VectorXd const velocity = -jacobian.transpose() * jjt.ldlt().solve(error);
        q = pinocchio::integrate(m_impl->model, q, velocity * IK_STEP);
        // MuJoCo enforces the URDF limits: a target beyond them is never reached.
        q = q.cwiseMax(m_impl->model.lowerPositionLimit)
                .cwiseMin(m_impl->model.upperPositionLimit);
    }

    return std::nullopt;
}

} // namespace robotik
