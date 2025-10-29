/**
 * @file Trajectory.hpp
 * @brief Trajectory generation system for robotic motion planning.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Types.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! \brief Constraints for each articulation
// ****************************************************************************
struct JointLimits
{
    double max_velocity = std::numeric_limits<double>::infinity();
    double max_acceleration = std::numeric_limits<double>::infinity();
    double max_jerk = std::numeric_limits<double>::infinity();
};

// ****************************************************************************
//! \brief Base interface for trajectories
// ****************************************************************************
class Trajectory
{
public:

    // ************************************************************************
    //! \brief Point of a trajectory evaluated at a specific time
    // ************************************************************************
    struct States
    {
        JointValues position;
        JointValues velocity;
        JointValues acceleration;
        double time;
    };

    virtual ~Trajectory() = default;

    // ------------------------------------------------------------------------
    //! \brief Evaluate trajectory at time t.
    //! \param t Time at which to evaluate the trajectory.
    //! \return TrajectoryPoint containing position, velocity, and acceleration.
    // ------------------------------------------------------------------------
    virtual Trajectory::States evaluate(double t) const = 0;

    // ------------------------------------------------------------------------
    //! \brief Get total duration of the trajectory.
    //! \return Duration in seconds.
    // ------------------------------------------------------------------------
    virtual double duration() const = 0;

    // ------------------------------------------------------------------------
    //! \brief Validate the trajectory.
    //! \return True if trajectory is valid.
    // ------------------------------------------------------------------------
    virtual bool isValid() const
    {
        return true;
    }

    // ------------------------------------------------------------------------
    //! \brief Get position at time t.
    // ------------------------------------------------------------------------
    JointValues getPosition(double t) const
    {
        return evaluate(t).position;
    }

    // ------------------------------------------------------------------------
    //! \brief Get velocity at time t.
    // ------------------------------------------------------------------------
    JointValues getVelocity(double t) const
    {
        return evaluate(t).velocity;
    }

    // ------------------------------------------------------------------------
    //! \brief Get acceleration at time t.
    // ------------------------------------------------------------------------
    JointValues getAcceleration(double t) const
    {
        return evaluate(t).acceleration;
    }
};

// ****************************************************************************
//! \brief Quintic polynomial (5th degree) for one joint.
//!
//! Allows specifying position, velocity and acceleration at both endpoints.
//! The polynomial is of the form:
//! p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
// ****************************************************************************
class QuinticPolynomial
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor for quintic polynomial.
    //!
    //! \param p0 Initial position
    //! \param v0 Initial velocity
    //! \param a0 Initial acceleration
    //! \param p1 Final position
    //! \param v1 Final velocity
    //! \param a1 Final acceleration
    //! \param duration Time duration
    // ------------------------------------------------------------------------
    QuinticPolynomial(double p0,
                      double v0,
                      double a0,
                      double p1,
                      double v1,
                      double a1,
                      double duration)
    {
        if (duration <= 0)
            throw std::invalid_argument("Duration must be positive");

        T_ = duration;

        // Coefficients of the polynomial: p(t) = a0 + a1*t + a2*t² + a3*t³ +
        // a4*t⁴ + a5*t⁵
        a0_ = p0;
        a1_ = v0;
        a2_ = a0 / 2.0;

        double T2 = T_ * T_;
        double T3 = T2 * T_;
        double T4 = T3 * T_;
        double T5 = T4 * T_;

        // Solve system for a3, a4, a5
        a3_ =
            (20 * p1 - 20 * p0 - (8 * v1 + 12 * v0) * T_ - (3 * a0 - a1) * T2) /
            (2 * T3);
        a4_ = (30 * p0 - 30 * p1 + (14 * v1 + 16 * v0) * T_ +
               (3 * a0 - 2 * a1) * T2) /
              (2 * T4);
        a5_ = (12 * p1 - 12 * p0 - (6 * v1 + 6 * v0) * T_ - (a0 - a1) * T2) /
              (2 * T5);
    }

    // ------------------------------------------------------------------------
    //! \brief Evaluate position at time t.
    // ------------------------------------------------------------------------
    double position(double t) const
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        return a0_ + a1_ * t + a2_ * t2 + a3_ * t3 + a4_ * t4 + a5_ * t5;
    }

    // ------------------------------------------------------------------------
    //! \brief Evaluate velocity at time t.
    // ------------------------------------------------------------------------
    double velocity(double t) const
    {
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        return a1_ + 2 * a2_ * t + 3 * a3_ * t2 + 4 * a4_ * t3 + 5 * a5_ * t4;
    }

    // ------------------------------------------------------------------------
    //! \brief Evaluate acceleration at time t.
    // ------------------------------------------------------------------------
    double acceleration(double t) const
    {
        double t2 = t * t;
        double t3 = t2 * t;
        return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * t2 + 20 * a5_ * t3;
    }

private:

    double a0_, a1_, a2_, a3_, a4_, a5_;
    double T_;
};

// ****************************************************************************
//! \brief Trajectory in joint space using quintic polynomials.
// ****************************************************************************
class JointSpaceTrajectory: public Trajectory
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor with start and goal configurations.
    //!
    //! Velocity and acceleration are zero at both endpoints.
    //!
    //! \param start Starting joint configuration.
    //! \param goal Goal joint configuration.
    //! \param duration Time duration for the trajectory.
    // ------------------------------------------------------------------------
    JointSpaceTrajectory(const JointValues& start,
                         const JointValues& goal,
                         double duration)
        : m_duration(duration)
    {
        if (start.size() != goal.size())
        {
            throw std::invalid_argument("Start and goal must have same size");
        }

        size_t n_joints = start.size();
        m_polynomials.reserve(n_joints);

        // Create a polynomial for each joint
        // Zero velocity and acceleration at endpoints by default
        for (size_t i = 0; i < n_joints; ++i)
        {
            m_polynomials.emplace_back(
                start[i],
                0.0,
                0.0, // position, velocity, acceleration initial
                goal[i],
                0.0,
                0.0, // position, velocity, acceleration final
                duration);
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Constructor with specified velocities and accelerations.
    //!
    //! \param start Starting joint configuration.
    //! \param start_vel Starting joint velocities.
    //! \param start_acc Starting joint accelerations.
    //! \param goal Goal joint configuration.
    //! \param goal_vel Goal joint velocities.
    //! \param goal_acc Goal joint accelerations.
    //! \param duration Time duration for the trajectory.
    // ------------------------------------------------------------------------
    JointSpaceTrajectory(const JointValues& start,
                         const JointValues& start_vel,
                         const JointValues& start_acc,
                         const JointValues& goal,
                         const JointValues& goal_vel,
                         const JointValues& goal_acc,
                         double duration)
        : m_duration(duration)
    {
        if (start.size() != goal.size() || start.size() != start_vel.size() ||
            start.size() != start_acc.size() ||
            start.size() != goal_vel.size() || start.size() != goal_acc.size())
        {
            throw std::invalid_argument("All configs must have same size");
        }

        size_t n_joints = start.size();
        m_polynomials.reserve(n_joints);

        for (size_t i = 0; i < n_joints; ++i)
        {
            m_polynomials.emplace_back(start[i],
                                       start_vel[i],
                                       start_acc[i],
                                       goal[i],
                                       goal_vel[i],
                                       goal_acc[i],
                                       duration);
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Evaluate trajectory at time t.
    // ------------------------------------------------------------------------
    Trajectory::States evaluate(double t) const override
    {
        t = std::clamp(t, 0.0, m_duration);

        Trajectory::States states;
        states.time = t;
        states.position.resize(m_polynomials.size());
        states.velocity.resize(m_polynomials.size());
        states.acceleration.resize(m_polynomials.size());

        for (size_t i = 0; i < m_polynomials.size(); ++i)
        {
            states.position[i] = m_polynomials[i].position(t);
            states.velocity[i] = m_polynomials[i].velocity(t);
            states.acceleration[i] = m_polynomials[i].acceleration(t);
        }

        return states;
    }

    // ------------------------------------------------------------------------
    //! \brief Get trajectory duration.
    // ------------------------------------------------------------------------
    double duration() const override
    {
        return m_duration;
    }

private:

    std::vector<QuinticPolynomial> m_polynomials;
    double m_duration;
};

// ****************************************************************************
//! \brief Interface for trajectory generators.
// ****************************************************************************
class TrajectoryGenerator
{
public:

    virtual ~TrajectoryGenerator() = default;

    // ------------------------------------------------------------------------
    //! \brief Generate a trajectory.
    //!
    //! \param start Starting configuration.
    //! \param goal Goal configuration.
    //! \param duration Time duration.
    //! \return Unique pointer to generated trajectory.
    // ------------------------------------------------------------------------
    virtual std::unique_ptr<Trajectory>
    generate(const std::vector<double>& start,
             const std::vector<double>& goal,
             double duration) const = 0;
};

// ****************************************************************************
//! \brief Generator for joint-space trajectories.
// ****************************************************************************
class JointSpaceGenerator: public TrajectoryGenerator
{
public:

    JointSpaceGenerator() = default;

    // ------------------------------------------------------------------------
    //! \brief Set joint limits for trajectory generation.
    //!
    //! \param limits Vector of joint limits.
    // ------------------------------------------------------------------------
    void setJointLimits(const std::vector<JointLimits>& limits)
    {
        m_limits = limits;
    }

    // ------------------------------------------------------------------------
    //! \brief Generate joint-space trajectory.
    // ------------------------------------------------------------------------
    std::unique_ptr<Trajectory> generate(const JointValues& start,
                                         const JointValues& goal,
                                         double duration) const override
    {
        // TODO: Verify limits and adjust duration if necessary
        if (!m_limits.empty())
        {
            // Compute minimum duration based on constraints
            // For now, use provided duration
        }

        return std::make_unique<JointSpaceTrajectory>(start, goal, duration);
    }

private:

    std::vector<JointLimits> m_limits;
};

} // namespace robotik
