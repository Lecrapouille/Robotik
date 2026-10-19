// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Detector.hpp
/// Perception stages and the pipeline chaining them.
///
/// Robotik knows what a @ref Detector is, not which algorithms exist: color
/// blobs, AprilTag, YOLO or OpenCV detectors live in applications and plug in
/// here. Stages run in order on the same @ref Detections, so a later stage
/// can refine what an earlier one found (@ref DepthEstimator adds 3D points).
#pragma once

#include "Robotik/Perception/Detection.hpp"

#include <concepts>
#include <memory>
#include <utility>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// One perception stage: reads a frame, adds or refines detections.
///
/// @code
/// class BrightSpot final : public robotik::Detector
/// {
/// public:
///     void detect(robotik::CameraFrame const& p_frame,
///                 robotik::Detections& p_detections) override
///     {
///         robotik::Detection spot;
///         spot.label = "light";
///         spot.confidence = 1.0f;
///         // ... find the brightest pixel of p_frame, fill box and center
///         p_detections.items.push_back(spot);
///     }
/// };
/// @endcode
// ****************************************************************************
class Detector
{
public:

    virtual ~Detector() = default;

    // -------------------------------------------------------------------------
    /// Processes one frame.
    /// @param p_frame Image (and depth) of the camera.
    /// @param p_detections What the earlier stages found; append to it or
    ///   refine it.
    // -------------------------------------------------------------------------
    virtual void detect(CameraFrame const& p_frame,
                        Detections& p_detections) = 0;
};

// ****************************************************************************
/// Ordered list of stages producing one @ref Detections per frame.
///
/// The output buffer is reused from frame to frame.
///
/// @code
/// robotik::PerceptionPipeline pipeline;
/// auto& colors = pipeline.add<ColorDetector>();
/// colors.add("red_cube", {0.9f, 0.1f, 0.1f});
/// pipeline.add<robotik::DepthEstimator>();
/// camera.onFrame([&](auto const& p_frame) {
///     world_model.update(pipeline.process(p_frame));
/// });
/// @endcode
// ****************************************************************************
class PerceptionPipeline
{
public:

    // -------------------------------------------------------------------------
    /// Appends a stage built in place.
    /// @tparam T Concrete detector type.
    /// @param p_args Arguments of the constructor of @p T.
    /// @return The new stage, for configuration.
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
    /// Appends an existing stage.
    /// @param p_stage Stage to own.
    /// @return The stage.
    // -------------------------------------------------------------------------
    Detector& add(std::unique_ptr<Detector> p_stage)
    {
        m_stages.push_back(std::move(p_stage));
        return *m_stages.back();
    }

    // -------------------------------------------------------------------------
    /// Runs every stage on a frame, in order.
    /// @param p_frame Camera frame.
    /// @return Detections of this frame (valid until the next call).
    // -------------------------------------------------------------------------
    Detections const& process(CameraFrame const& p_frame);

    // -------------------------------------------------------------------------
    /// @return Output of the last @ref process.
    // -------------------------------------------------------------------------
    [[nodiscard]] Detections const& detections() const
    {
        return m_detections;
    }

    // -------------------------------------------------------------------------
    /// @return True when the pipeline has no stage.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool empty() const
    {
        return m_stages.empty();
    }

    // -------------------------------------------------------------------------
    /// Drops the stages and the last output.
    // -------------------------------------------------------------------------
    void clear()
    {
        m_stages.clear();
        m_detections = {};
    }

private:

    /// Stages, in run order.
    std::vector<std::unique_ptr<Detector>> m_stages;
    /// Output buffer, reused from frame to frame.
    Detections m_detections;
};

// ****************************************************************************
/// Gives a 3D point to detections from the depth image of an RGB-D camera:
/// median depth around the detection center. Put it after the detectors
/// that find the objects.
///
/// @code
/// pipeline.add<ColorDetector>();
/// pipeline.add<robotik::DepthEstimator>(3); // 7x7 pixel window
/// @endcode
// ****************************************************************************
class DepthEstimator final: public Detector
{
public:

    /// @param p_radius Half size in pixels of the sampled window.
    explicit DepthEstimator(int p_radius = 2) : m_radius(p_radius) {}

    /// Fills @c position of each detection that has a valid depth.
    void detect(CameraFrame const& p_frame, Detections& p_detections) override;

private:

    /// Half size in pixels of the sampled window.
    int m_radius;
    /// Depth values of the window, reused to compute the median.
    std::vector<float> m_samples;
};

} // namespace robotik
