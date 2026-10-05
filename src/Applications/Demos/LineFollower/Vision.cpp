// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Vision.hpp"

#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

#define LINE_BAND_TOP 0.70
#define LINE_BAND_BOTTOM 0.98
#define LINE_THRESHOLD 80
#define LINE_MIN_AREA 40
#define LINE_TRACKING_PX 60.0
#define TAG_MARGIN_SCALE 0.01f
#define TAG_MIN_MARGIN 40.0f
#define TAG_MIN_SIDE_PX 18.0
#define TAG_HAMMING_BITS 1
#define TAG_MAX_DISTANCE_M 1.5

// Wraps the RGB8 image without copying it.
static cv::Mat wrap(robotik::Image const& p_image)
{
    return cv::Mat(static_cast<int>(p_image.height()),
                   static_cast<int>(p_image.width()),
                   CV_8UC3,
                   const_cast<std::uint8_t*>(p_image.bytes().data()),
                   p_image.stride());
}

AprilTagDetector::AprilTagDetector(double p_tag_size)
    : m_tag_size(p_tag_size), m_family(tag36h11_create()), m_detector(apriltag_detector_create())
{
    apriltag_detector_add_family_bits(m_detector, m_family, TAG_HAMMING_BITS);
    m_detector->quad_decimate = 1.0f;
    m_detector->nthreads = 1;
}

AprilTagDetector::~AprilTagDetector()
{
    apriltag_detector_destroy(m_detector);
    tag36h11_destroy(m_family);
}

void AprilTagDetector::detect(robotik::CameraFrame const& p_frame, robotik::Detections& p_detections)
{
    if (p_frame.rgb.format() != robotik::PixelFormat::RGB8 || p_frame.rgb.empty())
    {
        return;
    }
    cv::cvtColor(wrap(p_frame.rgb), m_gray, cv::COLOR_RGB2GRAY);
    image_u8_t image{ m_gray.cols, m_gray.rows, static_cast<int32_t>(m_gray.step[0]), m_gray.data };
    zarray_t* found = apriltag_detector_detect(m_detector, &image);

    robotik::CameraIntrinsics const& intrinsics = p_frame.intrinsics;
    for (int i = 0; i < zarray_size(found); ++i)
    {
        apriltag_detection_t* tag = nullptr;
        zarray_get(found, i, &tag);
        double const side = std::hypot(tag->p[0][0] - tag->p[1][0], tag->p[0][1] - tag->p[1][1]);
        if (tag->decision_margin < TAG_MIN_MARGIN || side < TAG_MIN_SIDE_PX)
        {
            continue;
        }
        apriltag_detection_info_t info{ tag, m_tag_size, intrinsics.fx, intrinsics.fy,
                                        intrinsics.cx, intrinsics.cy };
        apriltag_pose_t pose;
        (void)estimate_tag_pose(&info, &pose);
        double const distance = std::hypot(MATD_EL(pose.t, 0, 0), MATD_EL(pose.t, 1, 0), MATD_EL(pose.t, 2, 0));
        if (distance > TAG_MAX_DISTANCE_M)
        {
            matd_destroy(pose.R);
            matd_destroy(pose.t);
            continue;
        }

        robotik::Detection detection;
        detection.label = "tag36h11";
        detection.id = tag->id;
        detection.confidence = std::clamp(tag->decision_margin * TAG_MARGIN_SCALE, 0.0f, 1.0f);
        detection.center = { static_cast<float>(tag->c[0]), static_cast<float>(tag->c[1]) };
        double x0 = tag->p[0][0], x1 = x0, y0 = tag->p[0][1], y1 = y0;
        for (auto const& corner : tag->p)
        {
            x0 = std::min(x0, corner[0]);
            x1 = std::max(x1, corner[0]);
            y0 = std::min(y0, corner[1]);
            y1 = std::max(y1, corner[1]);
        }
        detection.box = { static_cast<int>(x0), static_cast<int>(y0),
                          static_cast<int>(x1), static_cast<int>(y1) };
        auto r = [&pose](int p_row, int p_col) { return MATD_EL(pose.R, p_row, p_col); };
        detection.pose = robotik::Pose{
            { MATD_EL(pose.t, 0, 0), MATD_EL(pose.t, 1, 0), MATD_EL(pose.t, 2, 0) },
            robotik::Quaternion::basis({ r(0, 0), r(1, 0), r(2, 0) },
                                       { r(0, 1), r(1, 1), r(2, 1) },
                                       { r(0, 2), r(1, 2), r(2, 2) }) };
        matd_destroy(pose.R);
        matd_destroy(pose.t);
        p_detections.items.push_back(std::move(detection));
    }
    apriltag_detections_destroy(found);
}

void LineDetector::detect(robotik::CameraFrame const& p_frame, robotik::Detections& p_detections)
{
    if (p_frame.rgb.format() != robotik::PixelFormat::RGB8 || p_frame.rgb.empty())
    {
        return;
    }
    cv::Mat const rgb = wrap(p_frame.rgb);
    int const top = static_cast<int>(LINE_BAND_TOP * rgb.rows);
    int const bottom = static_cast<int>(LINE_BAND_BOTTOM * rgb.rows);
    cv::cvtColor(rgb.rowRange(top, bottom), m_gray, cv::COLOR_RGB2GRAY);
    cv::threshold(m_gray, m_mask, LINE_THRESHOLD, 255, cv::THRESH_BINARY_INV);
    int const count = cv::connectedComponentsWithStats(m_mask, m_labels, m_stats, m_centroids, 8);

    double const reference = m_last_x >= 0.0 ? m_last_x : 0.5 * rgb.cols;
    int best = -1;
    double best_distance = 0.0;
    for (int label = 1; label < count; ++label)
    {
        if (m_stats.at<int>(label, cv::CC_STAT_AREA) < LINE_MIN_AREA)
        {
            continue;
        }
        double const distance = std::abs(m_centroids.at<double>(label, 0) - reference);
        if (best < 0 || distance < best_distance)
        {
            best = label;
            best_distance = distance;
        }
    }
    if (best < 0 || (m_last_x >= 0.0 && best_distance > LINE_TRACKING_PX))
    {
        m_last_x = -1.0;
        return;
    }
    m_last_x = m_centroids.at<double>(best, 0);

    robotik::Detection detection;
    detection.label = "line";
    detection.confidence = 1.0f;
    detection.center = { static_cast<float>(m_last_x),
                         static_cast<float>(top + m_centroids.at<double>(best, 1)) };
    int const x = m_stats.at<int>(best, cv::CC_STAT_LEFT);
    int const y = top + m_stats.at<int>(best, cv::CC_STAT_TOP);
    detection.box = { x, y, x + m_stats.at<int>(best, cv::CC_STAT_WIDTH) - 1,
                      y + m_stats.at<int>(best, cv::CC_STAT_HEIGHT) - 1 };
    p_detections.items.push_back(std::move(detection));
}
