/**
 * @file PinocchioBackend.hpp
 * @brief Pinocchio kinematics and damped least-squares IK for one URDF.
 */

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robotik
{

/**
 * @brief Position and orientation of a frame in the robot base.
 *
 * Translation in meters; quaternion @c (qw, qx, qy, qz) in scalar-first order.
 */
struct Pose
{
    double px = 0.0; ///< X position in meters.
    double py = 0.0; ///< Y position in meters.
    double pz = 0.0; ///< Z position in meters.
    double qw = 1.0; ///< Quaternion scalar part.
    double qx = 0.0; ///< Quaternion X.
    double qy = 0.0; ///< Quaternion Y.
    double qz = 0.0; ///< Quaternion Z.
};

/**
 * @brief Analytical robot model: configuration, FK, and IK.
 *
 * Not stored on ECS entities; owned by @ref RobotRuntime. Joint indices here
 * may differ from MuJoCo indices—use @ref ecs::PinocchioJointBinding on links.
 *
 * @example
 * @code
 * robotik::PinocchioBackend kin("arm.urdf");
 * kin.setConfiguration(seed_q);
 * kin.updateKinematics();
 * robotik::Pose tcp = kin.framePose("link6");
 * if (auto q = kin.solveIK("link6", target_pose, seed_q))
 *     applyJointTargets(*q);
 * @endcode
 */
class PinocchioBackend
{
public:

    /**
     * @brief Builds the Pinocchio model from URDF.
     * @param p_urdf Path to URDF file.
     */
    explicit PinocchioBackend(std::string const& p_urdf);

    ~PinocchioBackend();

    PinocchioBackend(PinocchioBackend const&) = delete;
    PinocchioBackend& operator=(PinocchioBackend const&) = delete;

    /** @brief Number of generalized coordinates @c nq. */
    [[nodiscard]] std::size_t nq() const;

    /** @brief Number of velocity DoFs @c nv. */
    [[nodiscard]] std::size_t nv() const;

    /** @brief Copies @p_q into internal configuration (must match @ref nq). */
    void setConfiguration(std::vector<double> const& p_q);

    /** @brief Copies @p_v into internal velocity. */
    void setVelocity(std::vector<double> const& p_v);

    /** @brief Current @c q vector. */
    [[nodiscard]] std::vector<double> configuration() const;

    /** @brief Current @c v vector. */
    [[nodiscard]] std::vector<double> velocity() const;

    /** @brief Runs forward kinematics and frame placements. */
    void updateKinematics();

    /** @brief True if @p_name is a Pinocchio joint. */
    [[nodiscard]] bool hasJoint(std::string const& p_name) const;

    /** @brief Index in @c q for joint @p_name, or -1. */
    [[nodiscard]] int qIndex(std::string const& p_name) const;

    /** @brief Index in @c v for joint @p_name, or -1. */
    [[nodiscard]] int vIndex(std::string const& p_name) const;

    /** @brief Pinocchio joint id. */
    [[nodiscard]] std::size_t jointId(std::string const& p_name) const;

    /** @brief True if @p_name is a frame (link) in the model. */
    [[nodiscard]] bool hasFrame(std::string const& p_name) const;

    /** @brief Frame index for @p_name. */
    [[nodiscard]] std::size_t frameId(std::string const& p_name) const;

    /**
     * @brief World pose of frame @p_frame after @ref updateKinematics.
     * @throws std::invalid_argument if the frame is unknown.
     */
    [[nodiscard]] Pose framePose(std::string const& p_frame) const;

    /**
     * @brief Damped least-squares IK toward @p_target for frame @p_frame.
     * @param p_frame Link or frame name (e.g. tool link).
     * @param p_target Desired pose in the robot base frame.
     * @param p_seed Initial @c q; if size mismatches @ref nq, internal @c q is used.
     * @return Joint vector on success, or empty if iteration did not converge.
     */
    [[nodiscard]] std::optional<std::vector<double>>
    solveIK(std::string const& p_frame,
            Pose const& p_target,
            std::vector<double> const& p_seed) const;

private:

    /** @brief Pinocchio model, data, and configuration buffers. */
    struct Impl;

    /** @brief Heap-allocated implementation. */
    std::unique_ptr<Impl> m_impl;
};

} // namespace robotik
