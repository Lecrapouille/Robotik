// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Perception/Detector.hpp"

#include <opencv2/core.hpp>

#include <cstdint>
#include <vector>

struct apriltag_family;
struct apriltag_detector;

// AprilTag tag36h11 detection with a full pose per tag (AprilRobotics
// library). Label "tag36h11", id = tag id, pose in the optical frame.
class AprilTagDetector final: public robotik::Detector
{
public:

    // @p_tag_size: edge of the black square (m).
    explicit AprilTagDetector(double p_tag_size);
    ~AprilTagDetector() override;
    AprilTagDetector(AprilTagDetector const&) = delete;
    AprilTagDetector& operator=(AprilTagDetector const&) = delete;

    void detect(robotik::CameraFrame const& p_frame, robotik::Detections& p_detections) override;

private:

    double m_tag_size;
    apriltag_family* m_family;
    apriltag_detector* m_detector;
    cv::Mat m_gray;
};

// Dark line on a light floor, searched in a band of rows ahead of the robot
// (OpenCV threshold and connected components). The blob nearest to the last
// one wins, so tags beside the track do not steal the line.
// Label "line", center = blob centroid.
class LineDetector final: public robotik::Detector
{
public:

    void detect(robotik::CameraFrame const& p_frame, robotik::Detections& p_detections) override;

private:

    cv::Mat m_gray;
    cv::Mat m_mask;
    cv::Mat m_labels;
    cv::Mat m_stats;
    cv::Mat m_centroids;
    double m_last_x = -1.0;
};
