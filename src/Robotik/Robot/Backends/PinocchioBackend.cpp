// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/Backends/PinocchioBackend.hpp"

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
#include <Eigen/SVD>

#include <cmath>
#include <memory>
#include <stdexcept>

#define IK_MAX_ITERATIONS 500
#define IK_STEP 0.5
#define IK_DAMPING 1e-4
#define IK_TOLERANCE 1e-4
/// Largest joint move of one iteration (rad or m, norm over the joints).
#define IK_MAX_JOINT_STEP 0.2
/// Joint change of one iteration under which a part goal has stopped at its
/// closest point.
#define IK_STALL 1e-9
/// A singular value of the Jacobian below this is a direction no joint
/// produces (m or rad per joint unit).
#define JACOBIAN_RANK_TOLERANCE 1e-3
/// A direction is movable when the joints produce at least this share of it.
#define MOVABLE_SHARE 0.9

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
    return { p_value.data(), p_value.data() + p_value.size() };
}

/// Pose of tcp in the tool_mount frame. The mount itself is the identity
/// on the robot flange, so this is also the pose of tcp on the flange.
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
        m_impl->model.addFrame(
            pinocchio::Frame(
                "tcp", parent, flange_id, placement, pinocchio::OP_FRAME),
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
    pinocchio::forwardKinematics(
        m_impl->model, *m_impl->data, m_impl->q, m_impl->v);
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

std::vector<std::string> PinocchioBackend::linkFrames() const
{
    std::vector<std::string> names;
    for (pinocchio::Frame const& frame : m_impl->model.frames)
    {
        if (frame.type == pinocchio::BODY && frame.parentJoint != 0)
        {
            names.push_back(frame.name);
        }
    }
    return names;
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
    return framePose(
        static_cast<std::size_t>(m_impl->model.getFrameId(p_frame)));
}

Pose PinocchioBackend::framePose(std::size_t p_frame) const
{
    pinocchio::SE3 const& placement = m_impl->data->oMf[p_frame];
    Eigen::Quaterniond const rotation(placement.rotation());
    return Pose{ { placement.translation().x(),
                   placement.translation().y(),
                   placement.translation().z() },
                 { rotation.w(), rotation.x(), rotation.y(), rotation.z() } };
}

std::optional<std::vector<double>>
PinocchioBackend::solveIK(std::string const& p_frame,
                          Pose const& p_target,
                          std::span<double const> p_seed,
                          IkGoal p_goal) const
{
    if (!hasFrame(p_frame))
    {
        return std::nullopt;
    }

    Eigen::VectorXd q = m_impl->q;
    if (p_seed.size() == nq())
    {
        q = Eigen::Map<Eigen::VectorXd const>(
            p_seed.data(), static_cast<Eigen::Index>(p_seed.size()));
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

    Eigen::VectorXd error;
    Eigen::MatrixXd rows;
    for (int iteration = 0; iteration < IK_MAX_ITERATIONS; ++iteration)
    {
        pinocchio::forwardKinematics(m_impl->model, data, q);
        pinocchio::updateFramePlacements(m_impl->model, data);
        pinocchio::SE3 const& oMf = data.oMf[frame];

        if (p_goal == IkGoal::Whole)
        {
            pinocchio::SE3 const iMd = oMf.actInv(oMdes);
            error = pinocchio::log6(iMd).toVector();
        }
        else if (p_goal == IkGoal::Position)
        {
            // In the base frame; the joints must close the gap.
            error = oMf.translation() - oMdes.translation();
        }
        else
        {
            error =
                -(oMf.rotation() * pinocchio::log3(oMf.rotation().transpose() *
                                                   oMdes.rotation()));
        }
        if (error.norm() < IK_TOLERANCE)
        {
            return toVector(q);
        }

        if (p_goal == IkGoal::Whole)
        {
            pinocchio::computeFrameJacobian(
                m_impl->model, data, q, frame, pinocchio::LOCAL, jacobian);
            pinocchio::Data::Matrix6 jlog;
            pinocchio::Jlog6(oMf.actInv(oMdes).inverse(), jlog);
            jacobian = -jlog * jacobian;
            rows = jacobian;
        }
        else
        {
            // Base axes at the frame origin: the error above is in them.
            pinocchio::computeFrameJacobian(m_impl->model,
                                            data,
                                            q,
                                            frame,
                                            pinocchio::LOCAL_WORLD_ALIGNED,
                                            jacobian);
            rows = p_goal == IkGoal::Position ? jacobian.topRows(3)
                                              : jacobian.bottomRows(3);
        }

        // Damped least squares in joint space: stable when the frame has
        // fewer joints than the 6 degrees of the error (link1 of an arm
        // cannot translate along X). The 6x6 form amplifies the error
        // that no joint can reduce by 1 / damping.
        Eigen::MatrixXd jtj = rows.transpose() * rows;
        jtj.diagonal().array() += IK_DAMPING;
        Eigen::VectorXd velocity = -jtj.ldlt().solve(rows.transpose() * error);
        if (!velocity.allFinite())
        {
            return std::nullopt;
        }
        // A bounded step per iteration: far targets are approached, not jumped
        // to.
        double const norm = velocity.norm() * IK_STEP;
        if (norm > IK_MAX_JOINT_STEP)
        {
            velocity *= IK_MAX_JOINT_STEP / norm;
        }
        Eigen::VectorXd const previous = q;
        q = pinocchio::integrate(m_impl->model, q, velocity * IK_STEP);
        // MuJoCo enforces the URDF limits: a target beyond them is never
        // reached.
        q = q.cwiseMax(m_impl->model.lowerPositionLimit)
                .cwiseMin(m_impl->model.upperPositionLimit);
        // A part of the pose may be out of reach of few joints: they stop
        // at the closest point (least squares), which is the answer.
        if (p_goal != IkGoal::Whole && (q - previous).norm() < IK_STALL)
        {
            return toVector(q);
        }
    }

    if (p_goal != IkGoal::Whole && q.allFinite())
    {
        return toVector(q);
    }
    return std::nullopt;
}

/// Jacobian of @p_frame at the current @b q, in the base axes at the frame
/// origin: linear rows then angular rows.
static Eigen::MatrixXd alignedJacobian(pinocchio::Model const& p_model,
                                       Eigen::VectorXd const& p_q,
                                       pinocchio::FrameIndex p_frame)
{
    pinocchio::Data data(p_model);
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, p_model.nv);
    pinocchio::computeFrameJacobian(
        p_model, data, p_q, p_frame, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
    return jacobian;
}

std::size_t PinocchioBackend::chainDofs(std::string const& p_frame) const
{
    if (!hasFrame(p_frame))
    {
        throw std::invalid_argument("Unknown Pinocchio frame: " + p_frame);
    }
    // A joint above the frame always moves it: a revolute one turns it, a
    // prismatic one shifts it. A joint elsewhere gives a zero column.
    Eigen::MatrixXd const jacobian = alignedJacobian(
        m_impl->model, m_impl->q, m_impl->model.getFrameId(p_frame));
    std::size_t count = 0;
    for (Eigen::Index column = 0; column < jacobian.cols(); ++column)
    {
        count += jacobian.col(column).squaredNorm() > 0.0 ? 1U : 0U;
    }
    return count;
}

/// Share of the unit direction @p_axis that the columns of @p_rows
/// produce: 1 when it lies in their span, 0 when no joint helps.
static double producedShare(Eigen::MatrixXd const& p_rows, Eigen::Index p_axis)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> const svd(p_rows, Eigen::ComputeThinU);
    Eigen::VectorXd const& values = svd.singularValues();
    double share = 0.0;
    for (Eigen::Index i = 0; i < values.size(); ++i)
    {
        if (values(i) > JACOBIAN_RANK_TOLERANCE)
        {
            double const along = svd.matrixU()(p_axis, i);
            share += along * along;
        }
    }
    return std::sqrt(share);
}

std::array<bool, 6> PinocchioBackend::movableAxes(std::string const& p_frame,
                                                  IkGoal p_goal) const
{
    if (!hasFrame(p_frame))
    {
        throw std::invalid_argument("Unknown Pinocchio frame: " + p_frame);
    }
    Eigen::MatrixXd const jacobian = alignedJacobian(
        m_impl->model, m_impl->q, m_impl->model.getFrameId(p_frame));
    std::array<bool, 6> movable{};
    if (jacobian.cols() == 0)
    {
        return movable;
    }
    for (Eigen::Index axis = 0; axis < 6; ++axis)
    {
        double share = 0.0;
        if (p_goal == IkGoal::Whole)
        {
            share = producedShare(jacobian, axis);
        }
        else
        {
            // A translation frees the orientation, a rotation the position.
            Eigen::Index const first = axis < 3 ? 0 : 3;
            share = producedShare(jacobian.middleRows(first, 3), axis - first);
        }
        movable[static_cast<std::size_t>(axis)] = share > MOVABLE_SHARE;
    }
    return movable;
}

} // namespace robotik
