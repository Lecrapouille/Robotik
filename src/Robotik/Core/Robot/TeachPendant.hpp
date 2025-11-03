/**
 * @file TeachPendant.hpp
 * @brief Teach pendant pour le contrôle interactif du robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

namespace robotik
{

// Forward declaration
namespace application
{
class ControlledRobot;
}

// ****************************************************************************
//! \brief Teach pendant pour le contrôle interactif du robot.
//!
//! Cette classe implémente un teach pendant virtuel permettant de:
//! - Contrôler le robot en mode articulaire (joint space)
//! - Contrôler le robot en mode cartésien (task space) avec IK
//! - Enregistrer des waypoints (positions clés)
//! - Rejouer des trajectoires via les waypoints
//!
//! Le teach pendant est stateless: l'état (mode de contrôle, waypoints,
//! trajectoire) est stocké dans ControlledRobot. Cela permet de partager
//! un seul TeachPendant entre plusieurs robots.
//!
//! Architecture:
//! - Controller: possède un TeachPendant partagé
//! - ControlledRobot: possède l'état (control_mode, waypoints, trajectoire)
//! - TeachPendant: outil qui opère sur un robot via setRobot()
// ****************************************************************************
class TeachPendant
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructeur par défaut.
    // ------------------------------------------------------------------------
    TeachPendant();

    // ========================================================================
    // Configuration
    // ========================================================================

    // ------------------------------------------------------------------------
    //! \brief Configure le robot sur lequel opérer.
    //! \param p_robot Robot contrôlé.
    // ------------------------------------------------------------------------
    void setRobot(application::ControlledRobot& p_robot);

    // ------------------------------------------------------------------------
    //! \brief Configure le solveur de cinématique inverse.
    //! \param p_solver Pointeur vers le solveur IK (non possédé).
    // ------------------------------------------------------------------------
    void setIKSolver(IKSolver* p_solver);

    // ------------------------------------------------------------------------
    //! \brief Configure l'effecteur terminal pour le contrôle cartésien.
    //! \param p_end_effector Référence vers le nœud effecteur.
    // ------------------------------------------------------------------------
    void setEndEffector(Node const& p_end_effector);

    // ========================================================================
    // Contrôle articulaire (Joint Space)
    // ========================================================================

    // ------------------------------------------------------------------------
    //! \brief Déplace un joint spécifique.
    //! \param p_joint_idx Index du joint.
    //! \param p_delta Variation de position (rad ou m).
    //! \param p_speed Facteur de vitesse multiplicatif.
    //! \return true si le mouvement a été appliqué.
    // ------------------------------------------------------------------------
    bool moveJoint(size_t p_joint_idx, double p_delta, double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Déplace plusieurs joints simultanément.
    //! \param p_deltas Variations de position pour chaque joint.
    //! \param p_speed Facteur de vitesse multiplicatif.
    //! \return true si le mouvement a été appliqué.
    // ------------------------------------------------------------------------
    bool moveJoints(const std::vector<double>& p_deltas, double p_speed = 1.0);

    // ========================================================================
    // Contrôle cartésien (Task Space)
    // ========================================================================

    // ------------------------------------------------------------------------
    //! \brief Déplace l'effecteur en translation cartésienne.
    //! \param p_translation Vecteur de translation (m).
    //! \param p_speed Facteur de vitesse multiplicatif.
    //! \return true si le mouvement a été appliqué.
    // ------------------------------------------------------------------------
    bool moveCartesian(const Eigen::Vector3d& p_translation,
                       double p_speed = 1.0);

    // ------------------------------------------------------------------------
    //! \brief Applique une rotation à l'effecteur.
    //! \param p_rotation_axis Axe de rotation (normalisé).
    //! \param p_angle Angle de rotation (rad).
    //! \param p_speed Facteur de vitesse multiplicatif.
    //! \return true si le mouvement a été appliqué.
    // ------------------------------------------------------------------------
    bool rotateCartesian(const Eigen::Vector3d& p_rotation_axis,
                         double p_angle,
                         double p_speed = 1.0);

    // ========================================================================
    // Gestion des waypoints
    // ========================================================================

    // ------------------------------------------------------------------------
    //! \brief Enregistre la position actuelle du robot comme waypoint.
    //! \param p_label Label optionnel pour le waypoint.
    //! \return Index du waypoint enregistré.
    // ------------------------------------------------------------------------
    size_t recordWaypoint(const std::string& p_label = "");

    // ------------------------------------------------------------------------
    //! \brief Supprime un waypoint.
    //! \param p_idx Index du waypoint à supprimer.
    // ------------------------------------------------------------------------
    void deleteWaypoint(size_t p_idx);

    // ------------------------------------------------------------------------
    //! \brief Efface tous les waypoints enregistrés.
    // ------------------------------------------------------------------------
    void clearWaypoints();

    // ------------------------------------------------------------------------
    //! \brief Déplace le robot vers un waypoint enregistré.
    //! \param p_idx Index du waypoint cible.
    //! \param p_duration Durée du mouvement (s).
    //! \return true si le mouvement a démarré.
    // ------------------------------------------------------------------------
    bool goToWaypoint(size_t p_idx, double p_duration = 3.0);

    // ========================================================================
    // Lecture de trajectoire
    // ========================================================================

    // ------------------------------------------------------------------------
    //! \brief Démarre la lecture des waypoints enregistrés.
    //! \param p_loop Si true, boucle la trajectoire.
    //! \return true si la lecture a démarré.
    // ------------------------------------------------------------------------
    bool playRecordedTrajectory(bool p_loop = false);

    // ------------------------------------------------------------------------
    //! \brief Arrête la lecture de la trajectoire en cours.
    // ------------------------------------------------------------------------
    void stopTrajectory();

    // ------------------------------------------------------------------------
    //! \brief Met à jour l'état du robot (avance dans la trajectoire).
    //! \param p_dt Pas de temps (s).
    // ------------------------------------------------------------------------
    void update(double p_dt);

private:

    //! \brief Pointeur vers le robot contrôlé (peut changer via setRobot).
    Robot* m_robot = nullptr;

    //! \brief Pointeur vers le robot contrôlé avec son état.
    application::ControlledRobot* m_controlled_robot = nullptr;

    //! \brief Pointeur vers le solveur IK (partagé, non possédé).
    IKSolver* m_ik_solver = nullptr;

    //! \brief Pointeur vers l'effecteur terminal.
    Node const* m_end_effector = nullptr;
};

} // namespace robotik
