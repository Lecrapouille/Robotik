#include "Robotik/private/Conversions.hpp"
#include <iomanip>

namespace robotik::utils
{

// ----------------------------------------------------------------------------
Eigen::Matrix3d eulerToRotation(double p_rx, double p_ry, double p_rz)
{
    Eigen::AngleAxisd roll_angle(p_rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(p_ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(p_rz, Eigen::Vector3d::UnitZ());

    return yaw_angle.toRotationMatrix() * pitch_angle.toRotationMatrix() *
           roll_angle.toRotationMatrix();
}

// ----------------------------------------------------------------------------
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& p_rot)
{
    return p_rot.eulerAngles(2, 1, 0).reverse();
}

// ----------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          const Eigen::Matrix3d& p_rotation)
{
    Transform transform = Transform::Identity();
    transform.block<3, 3>(0, 0) = p_rotation;
    transform.block<3, 1>(0, 3) = p_translation;
    return transform;
}

// ----------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          double p_rx,
                          double p_ry,
                          double p_rz)
{
    return createTransform(p_translation, eulerToRotation(p_rx, p_ry, p_rz));
}

// ----------------------------------------------------------------------------
// FIXME: replace Eigen::Vector3d with Eigen::Vector3f
Eigen::Vector3d getTranslation(Transform const& p_transform)
{
    return p_transform.block<3, 1>(0, 3);
}

// ----------------------------------------------------------------------------
Eigen::Matrix3d getRotation(Transform const& p_transform)
{
    return p_transform.block<3, 3>(0, 0);
}

// ----------------------------------------------------------------------------
Pose transformToPose(Transform const& p_transform)
{
    Pose pose;
    pose.segment<3>(0) = getTranslation(p_transform);
    pose.segment<3>(3) = rotationToEuler(getRotation(p_transform));
    return pose;
}

// ----------------------------------------------------------------------------
Transform poseToTransform(Pose const& p_pose)
{
    return createTransform(p_pose.segment<3>(0),
                           double(p_pose(3)),
                           double(p_pose(4)),
                           double(p_pose(5)));
}

// ----------------------------------------------------------------------------
Transform dhTransform(double p_a, double p_alpha, double p_d, double p_theta)
{
    Transform transform = Transform::Identity();

    double cos_theta = std::cos(p_theta);
    double sin_theta = std::sin(p_theta);
    double cos_alpha = std::cos(p_alpha);
    double sin_alpha = std::sin(p_alpha);

    transform(0, 0) = cos_theta;
    transform(0, 1) = -sin_theta * cos_alpha;
    transform(0, 2) = sin_theta * sin_alpha;
    transform(0, 3) = p_a * cos_theta;

    transform(1, 0) = sin_theta;
    transform(1, 1) = cos_theta * cos_alpha;
    transform(1, 2) = -cos_theta * sin_alpha;
    transform(1, 3) = p_a * sin_theta;

    transform(2, 0) = 0;
    transform(2, 1) = sin_alpha;
    transform(2, 2) = cos_alpha;
    transform(2, 3) = p_d;

    return transform;
}

// ----------------------------------------------------------------------------
std::string printTransform(const Transform& p_transform, int p_precision)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(p_precision);

    Eigen::Vector3d xyz = robotik::utils::getTranslation(p_transform);
    Eigen::Matrix3d rot = robotik::utils::getRotation(p_transform);
    Eigen::Vector3d rpy = robotik::utils::rotationToEuler(rot);

    oss << "xyz = (" << xyz.x() << ", " << xyz.y() << ", " << xyz.z() << ")";
    oss << ", rpy = (" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << ")";

    return oss.str();
}

// ----------------------------------------------------------------------------
std::string printPose(const std::string& name, const Pose& pose)
{
    std::ostringstream oss;
    oss << name << ": [" << pose(0) << ", " << pose(1) << ", " << pose(2)
        << ", " << pose(3) << ", " << pose(4) << ", " << pose(5) << "]";
    return oss.str();
}

} // namespace robotik::utils