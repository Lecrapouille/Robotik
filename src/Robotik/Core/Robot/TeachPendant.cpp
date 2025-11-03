/**
 * @file TeachPendant.cpp
 * @brief Implementation of the teach pendant for the robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/TeachPendant.hpp"
#include <chrono>
#include <stdexcept>

namespace robotik
{

TeachPendant::TeachPendant(Robot& robot)
    : robot_(robot),
      control_mode_(ControlMode::JOINT),
      state_(State::IDLE),
      speed_factor_(0.5),
      recording_start_time_(0.0),
      is_recording_(false)
{
    size_t n_joints = robot_.blueprint().numJoints();
    joint_velocity_.resize(n_joints, 0.0);
    target_joint_positions_ = robot_.state().joint_positions;

    // Limites par défaut
    joint_limits_lower_.resize(n_joints, -M_PI);
    joint_limits_upper_.resize(n_joints, M_PI);
    workspace_min_ = Eigen::Vector3d(-2.0, -2.0, 0.0);
    workspace_max_ = Eigen::Vector3d(2.0, 2.0, 2.0);
}

// === Contrôle articulaire ===
bool TeachPendant::moveJoint(size_t joint_idx, double delta, double speed)
{
    if (state_ == State::EMERGENCY_STOP)
        return false;
    if (joint_idx >= robot_.blueprint().numJoints())
        return false;

    auto current = robot_.state().joint_positions;
    current[joint_idx] += delta * speed_factor_ * speed;

    if (!checkJointLimits(current))
        return false;

    setState(State::MANUAL_CONTROL);
    target_joint_positions_ = current;
    robot_.setJointPositions(current);

    if (is_recording_)
    {
        recordWaypoint("auto_" + std::to_string(waypoints_.size()));
    }

    return true;
}

bool TeachPendant::moveJoints(const std::vector<double>& deltas, double speed)
{
    if (state_ == State::EMERGENCY_STOP)
        return false;
    if (deltas.size() != robot_.blueprint().numJoints())
        return false;

    auto current = robot_.state().joint_positions;
    for (size_t i = 0; i < deltas.size(); ++i)
    {
        current[i] += deltas[i] * speed_factor_ * speed;
    }

    if (!checkJointLimits(current))
        return false;

    setState(State::MANUAL_CONTROL);
    target_joint_positions_ = current;
    robot_.setJointPositions(current);

    if (is_recording_)
    {
        recordWaypoint("auto_" + std::to_string(waypoints_.size()));
    }

    return true;
}

bool TeachPendant::moveToJointConfig(const std::vector<double>& target,
                                     double duration)
{
    if (state_ == State::EMERGENCY_STOP)
        return false;
    if (!checkJointLimits(target))
        return false;

    // Créer une trajectoire simple entre position actuelle et cible
    auto current = robot_.state().joint_positions;
    // TODO: Utiliser la classe Trajectory pour interpoler
    // trajectory_->addWaypoint(current, 0.0);
    // trajectory_->addWaypoint(target, duration);

    target_joint_positions_ = target;
    robot_.setJointPositions(target);

    return true;
}

// === Contrôle cartésien ===
bool TeachPendant::moveCartesian(const Eigen::Vector3d& translation,
                                 double speed)
{
    if (state_ == State::EMERGENCY_STOP)
        return false;
    if (control_mode_ != ControlMode::CARTESIAN)
        return false;
    if (!end_effector_ || !ik_solver_)
        return false;

    auto current_transform = end_effector_->worldTransform();
    Eigen::Affine3d target_pose(current_transform.cast<double>());
    target_pose.translation() += translation * speed_factor_ * speed;

    if (!checkWorkspaceLimits(target_pose))
        return false;

    // Convert Eigen::Affine3d to Pose (6D vector)
    Pose target_pose_6d;
    target_pose_6d.head<3>() = target_pose.translation();
    Eigen::Vector3d euler = target_pose.rotation().eulerAngles(0, 1, 2);
    target_pose_6d.tail<3>() = euler;

    // Résoudre la cinématique inverse
    if (!ik_solver_->solve(robot_, *end_effector_, target_pose_6d))
    {
        return false;
    }

    auto target_joints = ik_solver_->solution();
    setState(State::MANUAL_CONTROL);
    robot_.setJointPositions(target_joints);
    target_joint_positions_ = target_joints;

    if (is_recording_)
    {
        recordWaypoint("auto_" + std::to_string(waypoints_.size()));
    }

    return true;
}

bool TeachPendant::rotateCartesian(const Eigen::Vector3d& rotation_axis,
                                   double angle,
                                   double speed)
{
    if (state_ == State::EMERGENCY_STOP)
        return false;
    if (control_mode_ != ControlMode::CARTESIAN)
        return false;
    if (!end_effector_ || !ik_solver_)
        return false;

    auto current_transform = end_effector_->worldTransform();
    Eigen::Affine3d target_pose(current_transform.cast<double>());
    Eigen::AngleAxisd rotation(angle * speed_factor_ * speed,
                               rotation_axis.normalized());
    target_pose.rotate(rotation);

    // Convert Eigen::Affine3d to Pose (6D vector)
    Pose target_pose_6d;
    target_pose_6d.head<3>() = target_pose.translation();
    Eigen::Vector3d euler = target_pose.rotation().eulerAngles(0, 1, 2);
    target_pose_6d.tail<3>() = euler;

    if (!ik_solver_->solve(robot_, *end_effector_, target_pose_6d))
    {
        return false;
    }

    auto target_joints = ik_solver_->solution();
    setState(State::MANUAL_CONTROL);
    robot_.setJointPositions(target_joints);
    target_joint_positions_ = target_joints;

    return true;
}

// === Gestion des waypoints ===
size_t TeachPendant::recordWaypoint(const std::string& label)
{
    Waypoint wp;
    wp.joint_positions = robot_.state().joint_positions;
    if (end_effector_)
    {
        wp.pose =
            Eigen::Affine3d(end_effector_->worldTransform().cast<double>());
    }
    else
    {
        wp.pose = Eigen::Affine3d::Identity();
    }
    wp.timestamp = std::chrono::duration<double>(
                       std::chrono::steady_clock::now().time_since_epoch())
                       .count();
    wp.label =
        label.empty() ? "waypoint_" + std::to_string(waypoints_.size()) : label;

    waypoints_.push_back(wp);
    return waypoints_.size() - 1;
}

void TeachPendant::clearWaypoints()
{
    waypoints_.clear();
}

bool TeachPendant::renameWaypoint(size_t idx, const std::string& new_label)
{
    if (idx >= waypoints_.size())
        return false;
    waypoints_[idx].label = new_label;
    return true;
}

bool TeachPendant::goToWaypoint(size_t idx, double duration)
{
    if (idx >= waypoints_.size())
        return false;
    return moveToJointConfig(waypoints_[idx].joint_positions, duration);
}

// === Enregistrement de trajectoire ===
void TeachPendant::startRecording()
{
    is_recording_ = true;
    recording_start_time_ =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    clearWaypoints();
    setState(State::RECORDING);
    recordWaypoint("start");
}

void TeachPendant::stopRecording()
{
    if (is_recording_)
    {
        recordWaypoint("end");
        is_recording_ = false;
        setState(State::IDLE);
    }
}

// === Sécurité ===
void TeachPendant::emergencyStop()
{
    setState(State::EMERGENCY_STOP);
    joint_velocity_.assign(joint_velocity_.size(), 0.0);
    // Arrêt immédiat du robot
}

void TeachPendant::reset()
{
    if (state_ == State::EMERGENCY_STOP)
    {
        setState(State::IDLE);
    }
}

bool TeachPendant::isInSafePosition() const
{
    if (collision_check_callback_ && !collision_check_callback_())
    {
        return false;
    }

    bool joint_limits_ok = checkJointLimits(robot_.state().joint_positions);
    bool workspace_ok = true;
    if (end_effector_)
    {
        auto transform = end_effector_->worldTransform();
        Eigen::Affine3d pose(transform.cast<double>());
        workspace_ok = checkWorkspaceLimits(pose);
    }

    return joint_limits_ok && workspace_ok;
}

// === Méthodes internes ===
bool TeachPendant::checkJointLimits(const std::vector<double>& positions) const
{
    for (size_t i = 0; i < positions.size(); ++i)
    {
        if (positions[i] < joint_limits_lower_[i] ||
            positions[i] > joint_limits_upper_[i])
        {
            return false;
        }
    }
    return true;
}

bool TeachPendant::checkWorkspaceLimits(const Eigen::Affine3d& pose) const
{
    auto pos = pose.translation();
    return (pos.x() >= workspace_min_.x() && pos.x() <= workspace_max_.x() &&
            pos.y() >= workspace_min_.y() && pos.y() <= workspace_max_.y() &&
            pos.z() >= workspace_min_.z() && pos.z() <= workspace_max_.z());
}

void TeachPendant::setState(State new_state)
{
    if (state_ != new_state)
    {
        state_ = new_state;
        if (state_change_callback_)
        {
            state_change_callback_(new_state);
        }
    }
}

void TeachPendant::setSpeedFactor(double factor)
{
    speed_factor_ = std::clamp(factor, 0.0, 1.0);
}

std::vector<double> TeachPendant::getCurrentJointPositions() const
{
    return robot_.state().joint_positions;
}

Eigen::Affine3d TeachPendant::getCurrentPose() const
{
    if (end_effector_)
    {
        return Eigen::Affine3d(end_effector_->worldTransform().cast<double>());
    }
    return Eigen::Affine3d::Identity();
}

void TeachPendant::setIKSolver(std::unique_ptr<IKSolver> solver)
{
    ik_solver_ = std::move(solver);
}

void TeachPendant::setEndEffector(Node const& end_effector)
{
    end_effector_ = &end_effector;
}

void TeachPendant::setControlMode(ControlMode mode)
{
    control_mode_ = mode;
}

void TeachPendant::deleteWaypoint(size_t idx)
{
    if (idx < waypoints_.size())
    {
        waypoints_.erase(waypoints_.begin() + idx);
    }
}

bool TeachPendant::playRecordedTrajectory(bool loop)
{
    // TODO: Implement trajectory playback
    return false;
}

void TeachPendant::stopTrajectory()
{
    // TODO: Implement trajectory stop
}

std::shared_ptr<Trajectory> TeachPendant::getRecordedTrajectory() const
{
    return current_trajectory_;
}

bool TeachPendant::loadTrajectory(std::shared_ptr<Trajectory> traj)
{
    current_trajectory_ = traj;
    return true;
}

void TeachPendant::setCollisionCheckCallback(std::function<bool()> callback)
{
    collision_check_callback_ = callback;
}

void TeachPendant::updateRobotState(double dt)
{
    // Update robot state based on current control mode
    // This can be expanded to handle trajectory following, etc.
}

void TeachPendant::setStateChangeCallback(std::function<void(State)> callback)
{
    state_change_callback_ = callback;
}

void TeachPendant::setWaypointReachedCallback(
    std::function<void(size_t)> callback)
{
    waypoint_reached_callback_ = callback;
}

bool TeachPendant::moveToPose(const Eigen::Affine3d& target_pose,
                              double duration)
{
    if (state_ == State::EMERGENCY_STOP)
        return false;
    if (!end_effector_ || !ik_solver_)
        return false;

    // Convert Eigen::Affine3d to Pose (6D vector)
    Pose target_pose_6d;
    target_pose_6d.head<3>() = target_pose.translation();
    Eigen::Vector3d euler = target_pose.rotation().eulerAngles(0, 1, 2);
    target_pose_6d.tail<3>() = euler;

    if (!ik_solver_->solve(robot_, *end_effector_, target_pose_6d))
    {
        return false;
    }

    auto target_joints = ik_solver_->solution();
    robot_.setJointPositions(target_joints);
    target_joint_positions_ = target_joints;

    return true;
}

bool TeachPendant::executeCartesianMotion(const Eigen::Affine3d& target_pose,
                                          double dt)
{
    // TODO: Implement smooth cartesian motion with velocity control
    return moveToPose(target_pose, dt);
}

} // namespace robotik