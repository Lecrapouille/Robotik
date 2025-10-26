/**
 * @file TimingController.hpp
 * @brief Controller for managing frame rate, physics timing and FPS
 * calculation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>

namespace robotik::renderer::application
{

// ****************************************************************************
//! \brief Controller for managing timing, frame rate and update calculations.
//! This class can be used for rendering loops, physics loops, or any fixed
//! timestep loop.
// ****************************************************************************
class TimingController
{
public:

    //! \brief Callback type for rate updates (e.g., FPS, UPS).
    using RateCallback = std::function<void(size_t)>;
    //! \brief Callback type for update callbacks.
    using UpdateCallback = std::function<void(float)>;

    // ----------------------------------------------------------------------------
    //! \brief Initialize the timing controller.
    //! \param p_target_rate Target rate per second (FPS or UPS).
    //! \param p_rate_callback Optional callback to be called when rate is
    //! updated.
    // ----------------------------------------------------------------------------
    void initialize(size_t p_target_rate,
                    RateCallback const& p_rate_callback = nullptr);

    // ----------------------------------------------------------------------------
    //! \brief Start a new frame. Should be called at the beginning of each
    //! frame.
    //! \return Delta time in seconds since the last frame.
    // ----------------------------------------------------------------------------
    float startFrame();

    // ----------------------------------------------------------------------------
    //! \brief End the current frame. This method updates rate and limits the
    //! framerate to the target rate.
    // ----------------------------------------------------------------------------
    void endFrame();

    // ----------------------------------------------------------------------------
    //! \brief Run a fixed timestep loop. This method should be called in a
    //! separate thread and will run until stopped.
    //! \param p_running Atomic boolean to control the loop.
    //! \param p_update_callback Callback to be called for each update.
    // ----------------------------------------------------------------------------
    void runLoop(std::atomic<bool> const& p_running,
                 UpdateCallback const& p_update_callback) const;

    // ----------------------------------------------------------------------------
    //! \brief Get the current rate (FPS/UPS).
    //! \return Current rate value.
    // ----------------------------------------------------------------------------
    float currentRate() const;

    // ----------------------------------------------------------------------------
    //! \brief Get the current frame count.
    //! \return The number of frames rendered since initialization.
    // ----------------------------------------------------------------------------
    size_t frameCount() const;

private:

    // ----------------------------------------------------------------------------
    //! \brief Update the rate calculation based on frame count.
    // ----------------------------------------------------------------------------
    void updateRate();

    // ----------------------------------------------------------------------------
    //! \brief Limit the framerate to the target FPS.
    // ----------------------------------------------------------------------------
    void limitFramerate() const;

private:

    //! \brief Target time per frame (1.0 / target_rate).
    float m_target_frame_time = 0.0f;
    //! \brief Current rate (FPS/UPS).
    float m_current_rate = 0.0f;
    //! \brief Time point of the last frame.
    std::chrono::high_resolution_clock::time_point m_last_frame_time;
    //! \brief Time point of the last rate update.
    std::chrono::high_resolution_clock::time_point m_last_rate_time;
    //! \brief Total number of frames/updates.
    size_t m_frame_count{ 0 };
    //! \brief Frame/update count at the last rate update.
    size_t m_last_frame_count{ 0 };
    //! \brief Callback to be called when rate is updated.
    RateCallback m_rate_callback;
};

} // namespace robotik::renderer::application
