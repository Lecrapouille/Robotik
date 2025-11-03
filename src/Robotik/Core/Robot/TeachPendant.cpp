/**
 * @file TeachPendant.cpp
 * @brief Implémentation du teach pendant pour le contrôle interactif du robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/TeachPendant.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/ControlledRobot.hpp"
#include "Robotik/Core/Solvers/Trajectory.hpp"

#include <iostream>

namespace robotik
{

// ----------------------------------------------------------------------------
TeachPendant::TeachPendant()
    : m_robot(nullptr),
      m_controlled_robot(nullptr),
      m_ik_solver(nullptr),
      m_end_effector(nullptr)
{
}

// ============================================================================
// Configuration
// ============================================================================

// ----------------------------------------------------------------------------
void TeachPendant::setRobot(application::ControlledRobot& p_robot)
{
    m_robot = &p_robot;
    m_controlled_robot = &p_robot;
    m_end_effector = p_robot.end_effector;
}

// ----------------------------------------------------------------------------
void TeachPendant::setIKSolver(IKSolver* p_solver)
{
    m_ik_solver = p_solver;
}

// ----------------------------------------------------------------------------
void TeachPendant::setEndEffector(Node const& p_end_effector)
{
    m_end_effector = &p_end_effector;
}

// ============================================================================
// Contrôle articulaire (Joint Space)
// ============================================================================

// ----------------------------------------------------------------------------
bool TeachPendant::moveJoint(size_t p_joint_idx, double p_delta, double p_speed)
{
    if (!m_robot || !m_controlled_robot)
        return false;

    // Impossible de bouger pendant la lecture d'une trajectoire
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
        return false;

    // Vérifier que l'index est valide
    if (p_joint_idx >= m_robot->blueprint().numJoints())
        return false;

    // Calculer la nouvelle position cible
    auto target = m_robot->state().joint_positions;
    target[p_joint_idx] += p_delta * m_controlled_robot->speed_factor * p_speed;

    // Vérifier les limites via forEachJoint
    bool within_limits = true;
    m_robot->blueprint().forEachJoint(
        [&](Joint const& joint, size_t index)
        {
            if (index == p_joint_idx)
            {
                auto [min, max] = joint.limits();
                if (target[index] < min || target[index] > max)
                {
                    within_limits = false;
                }
            }
        });

    if (!within_limits)
        return false;

    // Appliquer le mouvement
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;
    m_robot->setJointPositions(target);

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::moveJoints(const std::vector<double>& p_deltas,
                              double p_speed)
{
    if (!m_robot || !m_controlled_robot)
        return false;

    // Impossible de bouger pendant la lecture d'une trajectoire
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
        return false;

    // Vérifier que le nombre de deltas correspond au nombre de joints
    if (p_deltas.size() != m_robot->blueprint().numJoints())
        return false;

    // Calculer les nouvelles positions cibles
    auto target = m_robot->state().joint_positions;
    for (size_t i = 0; i < p_deltas.size(); ++i)
    {
        target[i] += p_deltas[i] * m_controlled_robot->speed_factor * p_speed;
    }

    // Vérifier toutes les limites
    bool within_limits = true;
    m_robot->blueprint().forEachJoint(
        [&](Joint const& joint, size_t index)
        {
            auto [min, max] = joint.limits();
            if (target[index] < min || target[index] > max)
            {
                within_limits = false;
            }
        });

    if (!within_limits)
        return false;

    // Appliquer le mouvement
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;
    m_robot->setJointPositions(target);

    return true;
}

// ============================================================================
// Contrôle cartésien (Task Space)
// ============================================================================

// ----------------------------------------------------------------------------
bool TeachPendant::moveCartesian(const Eigen::Vector3d& p_translation,
                                 double p_speed)
{
    if (!m_robot || !m_controlled_robot || !m_end_effector || !m_ik_solver)
        return false;

    // Vérifier le mode de contrôle
    if (m_controlled_robot->control_mode !=
        application::ControlledRobot::ControlMode::CARTESIAN)
        return false;

    // Impossible de bouger pendant la lecture d'une trajectoire
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
        return false;

    // Obtenir la transformation actuelle de l'effecteur (Matrix4d)
    Transform current_transform = m_end_effector->worldTransform();

    // Appliquer la translation
    current_transform.block<3, 1>(0, 3) +=
        p_translation * m_controlled_robot->speed_factor * p_speed;

    // Convertir en Pose 6D pour le solveur IK
    Pose target_pose;
    target_pose.head<3>() = current_transform.block<3, 1>(0, 3);

    // Extraire les angles d'Euler de la rotation
    Eigen::Matrix3d R = current_transform.block<3, 3>(0, 0);
    target_pose.tail<3>() = R.eulerAngles(0, 1, 2);

    // Résoudre la cinématique inverse
    if (!m_ik_solver->solve(*m_robot, *m_end_effector, target_pose))
    {
        return false;
    }

    // Appliquer la solution IK
    m_robot->setJointPositions(m_ik_solver->solution());
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;

    return true;
}

// ----------------------------------------------------------------------------
bool TeachPendant::rotateCartesian(const Eigen::Vector3d& p_rotation_axis,
                                   double p_angle,
                                   double p_speed)
{
    if (!m_robot || !m_controlled_robot || !m_end_effector || !m_ik_solver)
        return false;

    // Vérifier le mode de contrôle
    if (m_controlled_robot->control_mode !=
        application::ControlledRobot::ControlMode::CARTESIAN)
        return false;

    // Impossible de bouger pendant la lecture d'une trajectoire
    if (m_controlled_robot->state ==
        application::ControlledRobot::State::PLAYING)
        return false;

    // Obtenir la transformation actuelle de l'effecteur
    Transform current_transform = m_end_effector->worldTransform();

    // Créer la rotation via Rodrigues (AngleAxis)
    Eigen::AngleAxisd rotation(p_angle * m_controlled_robot->speed_factor *
                                   p_speed,
                               p_rotation_axis.normalized());

    // Appliquer la rotation à la partie rotationnelle de la transformation
    Eigen::Matrix3d R = current_transform.block<3, 3>(0, 0);
    R = rotation.toRotationMatrix() * R;
    current_transform.block<3, 3>(0, 0) = R;

    // Convertir en Pose 6D pour le solveur IK
    Pose target_pose;
    target_pose.head<3>() = current_transform.block<3, 1>(0, 3);
    target_pose.tail<3>() = R.eulerAngles(0, 1, 2);

    // Résoudre la cinématique inverse
    if (!m_ik_solver->solve(*m_robot, *m_end_effector, target_pose))
    {
        return false;
    }

    // Appliquer la solution IK
    m_robot->setJointPositions(m_ik_solver->solution());
    m_controlled_robot->state =
        application::ControlledRobot::State::MANUAL_CONTROL;

    return true;
}

// ============================================================================
// Gestion des waypoints
// ============================================================================

// ----------------------------------------------------------------------------
size_t TeachPendant::recordWaypoint(const std::string& p_label)
{
    if (!m_robot || !m_controlled_robot)
        return 0;

    // Créer un waypoint avec la position actuelle
    Trajectory::States waypoint;
    waypoint.position = m_robot->state().joint_positions;
    waypoint.velocity.resize(waypoint.position.size(), 0.0);
    waypoint.acceleration.resize(waypoint.position.size(), 0.0);
    waypoint.time = 0.0; // Sera recalculé lors du playback

    // Ajouter aux waypoints du robot
    m_controlled_robot->waypoints.push_back(waypoint);

    std::cout << "📍 Recorded waypoint "
              << m_controlled_robot->waypoints.size() - 1 << ": "
              << (p_label.empty() ? "unnamed" : p_label) << std::endl;

    return m_controlled_robot->waypoints.size() - 1;
}

// ----------------------------------------------------------------------------
void TeachPendant::deleteWaypoint(size_t p_idx)
{
    if (!m_controlled_robot)
        return;

    if (p_idx < m_controlled_robot->waypoints.size())
    {
        m_controlled_robot->waypoints.erase(
            m_controlled_robot->waypoints.begin() + p_idx);
    }
}

// ----------------------------------------------------------------------------
void TeachPendant::clearWaypoints()
{
    if (!m_controlled_robot)
        return;

    m_controlled_robot->waypoints.clear();
}

// ----------------------------------------------------------------------------
bool TeachPendant::goToWaypoint(size_t p_idx, double p_duration)
{
    if (!m_robot || !m_controlled_robot)
        return false;

    // Vérifier que l'index est valide
    if (p_idx >= m_controlled_robot->waypoints.size())
        return false;

    // Créer une trajectoire entre la position actuelle et le waypoint
    auto const& target_waypoint = m_controlled_robot->waypoints[p_idx];
    auto current_pos = m_robot->state().joint_positions;

    m_controlled_robot->playing_trajectory =
        std::make_unique<JointSpaceTrajectory>(
            current_pos, target_waypoint.position, p_duration);

    m_controlled_robot->trajectory_time = 0.0;
    m_controlled_robot->state = application::ControlledRobot::State::PLAYING;

    std::cout << "🎯 Going to waypoint " << p_idx << std::endl;

    return true;
}

// ============================================================================
// Lecture de trajectoire
// ============================================================================

// ----------------------------------------------------------------------------
bool TeachPendant::playRecordedTrajectory(bool p_loop)
{
    if (!m_robot || !m_controlled_robot)
        return false;

    // Vérifier qu'il y a au moins 2 waypoints
    if (m_controlled_robot->waypoints.size() < 2)
    {
        std::cout << "⚠️ Need at least 2 waypoints to play trajectory"
                  << std::endl;
        return false;
    }

    // Pour l'instant: trajectoire simple entre premier et dernier waypoint
    // TODO: Créer une trajectoire multi-segments passant par tous les waypoints
    auto const& wps = m_controlled_robot->waypoints;
    double duration_per_segment = 3.0; // 3 secondes entre waypoints

    m_controlled_robot->playing_trajectory =
        std::make_unique<JointSpaceTrajectory>(wps.front().position,
                                               wps.back().position,
                                               duration_per_segment *
                                                   (wps.size() - 1));

    m_controlled_robot->trajectory_time = 0.0;
    m_controlled_robot->state = application::ControlledRobot::State::PLAYING;

    std::cout << "▶️ Playing trajectory with " << wps.size() << " waypoints"
              << std::endl;

    return true;
}

// ----------------------------------------------------------------------------
void TeachPendant::stopTrajectory()
{
    if (!m_controlled_robot)
        return;

    m_controlled_robot->playing_trajectory.reset();
    m_controlled_robot->state = application::ControlledRobot::State::IDLE;

    std::cout << "⏹️ Trajectory stopped" << std::endl;
}

// ----------------------------------------------------------------------------
void TeachPendant::update(double p_dt)
{
    if (!m_robot || !m_controlled_robot)
        return;

    // Vérifier qu'on est en mode PLAYING avec une trajectoire
    if (m_controlled_robot->state !=
        application::ControlledRobot::State::PLAYING)
        return;

    if (!m_controlled_robot->playing_trajectory)
        return;

    // Avancer dans le temps
    m_controlled_robot->trajectory_time += p_dt;

    // Vérifier si la trajectoire est terminée
    if (m_controlled_robot->trajectory_time >=
        m_controlled_robot->playing_trajectory->duration())
    {
        // Fin de la trajectoire
        std::cout << "✓ Trajectory completed" << std::endl;
        m_controlled_robot->playing_trajectory.reset();
        m_controlled_robot->state = application::ControlledRobot::State::IDLE;
        return;
    }

    // Évaluer la position cible à ce temps
    auto target_positions = m_controlled_robot->playing_trajectory->getPosition(
        m_controlled_robot->trajectory_time);

    // Appliquer avec limites de vitesse
    m_robot->applyJointTargetsWithSpeedLimit(target_positions, p_dt);
}

} // namespace robotik
