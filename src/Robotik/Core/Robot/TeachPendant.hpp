/**
 * @file TeachPendant.hpp
 * @brief Teach pendant for the robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace robotik
{

class TeachPendant
{
public:

    enum class ControlMode
    {
        JOINT,     // Contrôle articulaire direct
        CARTESIAN, // Contrôle cartésien (position + orientation)
        TRAJECTORY // Suivi de trajectoire
    };

    enum class State
    {
        IDLE,
        MANUAL_CONTROL,
        RECORDING,
        PLAYING,
        EMERGENCY_STOP
    };

    struct Waypoint
    {
        std::vector<double> joint_positions;
        Eigen::Affine3d pose;
        double timestamp;
        std::string label;
    };

    TeachPendant(Robot& robot);

    // === Contrôle de base ===
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const
    {
        return control_mode_;
    }
    State getState() const
    {
        return state_;
    }

    // === Contrôle articulaire ===
    bool moveJoint(size_t joint_idx, double delta, double speed = 1.0);
    bool moveJoints(const std::vector<double>& deltas, double speed = 1.0);
    bool moveToJointConfig(const std::vector<double>& target,
                           double duration = 5.0);

    // === Contrôle cartésien ===
    bool moveCartesian(const Eigen::Vector3d& translation, double speed = 1.0);
    bool rotateCartesian(const Eigen::Vector3d& rotation_axis,
                         double angle,
                         double speed = 1.0);
    bool moveToPose(const Eigen::Affine3d& target_pose, double duration = 5.0);

    // === Gestion des waypoints ===
    size_t recordWaypoint(const std::string& label = "");
    void deleteWaypoint(size_t idx);
    void clearWaypoints();
    bool renameWaypoint(size_t idx, const std::string& new_label);
    const std::vector<Waypoint>& getWaypoints() const
    {
        return waypoints_;
    }
    bool goToWaypoint(size_t idx, double duration = 5.0);

    // === Trajectoires ===
    void startRecording();
    void stopRecording();
    bool playRecordedTrajectory(bool loop = false);
    void stopTrajectory();
    std::shared_ptr<Trajectory> getRecordedTrajectory() const;
    bool loadTrajectory(std::shared_ptr<Trajectory> traj);

    // === Vitesse et limites ===
    void setSpeedFactor(double factor); // 0.0 à 1.0
    double getSpeedFactor() const
    {
        return speed_factor_;
    }
    void setJointLimits(const std::vector<double>& lower,
                        const std::vector<double>& upper);
    void setCartesianWorkspace(const Eigen::Vector3d& min,
                               const Eigen::Vector3d& max);

    // === Teach Pendant Configuration ===
    void setIKSolver(std::unique_ptr<IKSolver> solver);
    void setEndEffector(Node const& end_effector);

    // === Sécurité ===
    void emergencyStop();
    void reset();
    bool isInSafePosition() const;
    void setCollisionCheckCallback(std::function<bool()> callback);

    // === État du robot ===
    std::vector<double> getCurrentJointPositions() const;
    Eigen::Affine3d getCurrentPose() const;
    void updateRobotState(double dt);

    // === Callbacks ===
    void setStateChangeCallback(std::function<void(State)> callback);
    void setWaypointReachedCallback(std::function<void(size_t)> callback);

private:

    Robot& robot_;
    std::unique_ptr<IKSolver> ik_solver_;
    std::shared_ptr<Trajectory> current_trajectory_;
    Node const* end_effector_ = nullptr;

    ControlMode control_mode_;
    State state_;

    std::vector<Waypoint> waypoints_;
    std::vector<double> joint_velocity_;
    std::vector<double> target_joint_positions_;

    double speed_factor_;
    double recording_start_time_;
    bool is_recording_;

    // Limites de sécurité
    std::vector<double> joint_limits_lower_;
    std::vector<double> joint_limits_upper_;
    Eigen::Vector3d workspace_min_;
    Eigen::Vector3d workspace_max_;

    // Callbacks
    std::function<bool()> collision_check_callback_;
    std::function<void(State)> state_change_callback_;
    std::function<void(size_t)> waypoint_reached_callback_;

    // Méthodes internes
    bool checkJointLimits(const std::vector<double>& positions) const;
    bool checkWorkspaceLimits(const Eigen::Affine3d& pose) const;
    void setState(State new_state);
    bool executeCartesianMotion(const Eigen::Affine3d& target_pose, double dt);
};

} // namespace robotik