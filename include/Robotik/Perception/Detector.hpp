// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Detector.hpp
//! @brief Perception stages and the pipeline chaining them.
//!
//! Robotik knows what a @ref Detector is, not which algorithms exist: color
//! blobs, AprilTag, YOLO or OpenCV detectors live in applications and plug in
//! here. Stages run in order on the same @ref Detections, so a later stage
//! can refine what an earlier one found (@ref DepthEstimator adds 3D points).
#pragma once

#include "Robotik/Perception/Detection.hpp"

#include <concepts>
#include <memory>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief One perception stage: reads a frame, adds or refines detections.
// ****************************************************************************
class Detector
{
public:

    virtual ~Detector() = default;

    // -------------------------------------------------------------------------
    //! @brief Processes @p_frame; appends to or refines @p_detections.
    // -------------------------------------------------------------------------
    virtual void detect(CameraFrame const& p_frame,
                        Detections& p_detections) = 0;
};

// ****************************************************************************
//! @brief Ordered list of stages producing one @ref Detections per frame.
//!
//! The output buffer is reused from frame to frame.
//!
//! @code
//! robotik::PerceptionPipeline pipeline;
//! auto& colors = pipeline.add<ColorDetector>();
//! colors.add("red_cube", {0.9f, 0.1f, 0.1f});
//! pipeline.add<robotik::DepthEstimator>();
//! camera.onFrame([&](auto const& p_frame) {
//!     world_model.update(pipeline.process(p_frame));
//! });
//! @endcode
// ****************************************************************************
class PerceptionPipeline
{
public:

    // -------------------------------------------------------------------------
    //! @brief Appends a stage built in place; returns it for configuration.
    // -------------------------------------------------------------------------
    template <std::derived_from<Detector> T, typename... Args>
    T& add(Args&&... p_args)
    {
        auto stage = std::make_unique<T>(std::forward<Args>(p_args)...);
        T& reference = *stage;
        m_stages.push_back(std::move(stage));
        return reference;
    }

    // -------------------------------------------------------------------------
    //! @brief Appends an existing stage.
    // -------------------------------------------------------------------------
    Detector& add(std::unique_ptr<Detector> p_stage)
    {
        m_stages.push_back(std::move(p_stage));
        return *m_stages.back();
    }

    // -------------------------------------------------------------------------
    //! @brief Runs every stage on @p_frame.
    //! @return Detections of this frame (valid until the next call).
    // -------------------------------------------------------------------------
    Detections const& process(CameraFrame const& p_frame);

    //! @brief Output of the last @ref process.
    [[nodiscard]] Detections const& detections() const
    {
        return m_detections;
    }

    [[nodiscard]] bool empty() const
    {
        return m_stages.empty();
    }

    //! @brief Drops the stages and the last output.
    void clear()
    {
        m_stages.clear();
        m_detections = {};
    }

private:

    std::vector<std::unique_ptr<Detector>> m_stages;
    Detections m_detections;
};

// ****************************************************************************
//! @brief Gives a 3D point to detections from the depth image of an RGB-D
//! camera: median depth around the detection center.
// ****************************************************************************
class DepthEstimator final: public Detector
{
public:

    //! @param p_radius Half size in pixels of the sampled window.
    explicit DepthEstimator(int p_radius = 2) : m_radius(p_radius) {}

    void detect(CameraFrame const& p_frame, Detections& p_detections) override;

private:

    int m_radius;
    std::vector<float> m_samples;
};

} // namespace robotik
