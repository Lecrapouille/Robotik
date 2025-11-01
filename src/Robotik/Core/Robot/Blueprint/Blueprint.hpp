/**
 * @file Blueprint.hpp
 * @brief Blueprint class - Representation of a robot's kinematic structure
 * (model).
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/Component.hpp"
#include "Robotik/Core/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"

namespace robotik
{

// ****************************************************************************
//! \brief Pure data structures for the kinematic model (flat array storage).
//!
//! These structures hold static robot data in cache-friendly flat arrays.
//! They replace the pointer-based tree traversal with index-based access.
//! This enables:
//! - Cache-friendly sequential memory access
//! - Immutable Blueprint (Model) that can be shared across multiple States
//! - Parallel forward kinematics computations
//!
//! Index guarantee: The index i in m_joint_data[i] IS the joint's index.
//! No lookup needed. Use m_joint_name_to_index only when you have a name
//! and need to find its index.
// ****************************************************************************

// ****************************************************************************
//! \brief Static data for a single joint in the kinematic model.
// ****************************************************************************
struct JointData
{
    std::string name;         //! Joint name (for lookup and debugging)
    Joint::Type type;         //! Joint type (REVOLUTE, PRISMATIC, etc.)
    size_t parent_link_index; //! Index of parent link (joint placement is
                              //! relative to this link)
    Transform placement;  //! Static transform from URDF (joint origin relative
                          //! to parent link)
    Eigen::Vector3d axis; //! Joint axis in local frame
    double position_min;  //! Minimum position limit
    double position_max;  //! Maximum position limit
    double velocity_max;  //! Maximum velocity limit
    double effort_max;    //! Maximum effort (torque/force) limit
    double damping;       //! Damping coefficient
    double friction;      //! Friction coefficient
};

// ****************************************************************************
//! \brief Static data for a single link in the kinematic model.
// ****************************************************************************
struct LinkData
{
    std::string name;          //! Link name (for lookup and debugging)
    size_t parent_joint_index; //! Index of parent joint
    Transform placement;       //! Static offset from parent joint
    Inertial inertial;         //! Mass properties (mass, CoM, inertia tensor)
    std::vector<size_t> geometry_indices; //! Indices into m_geometry_data
};

// ****************************************************************************
//! \brief Static data for geometry (visual/collision).
// ****************************************************************************
struct GeometryData
{
    std::string name;               //! Geometry name
    size_t parent_link_index;       //! Which link owns this geometry
    Transform local_transform;      //! Transform relative to parent link
    Geometry::Type type;            //! Type (BOX, CYLINDER, SPHERE, MESH)
    std::vector<double> parameters; //! Dimensions (depends on type)
    std::string mesh_path;          //! Path to mesh file (if type is MESH)
    Eigen::Vector3f color;          //! RGB color for visualization
};

// ****************************************************************************
//! \brief Static data for sensors attached to links.
// ****************************************************************************
struct SensorData
{
    std::string name;          //! Sensor name
    size_t parent_link_index;  //! Which link the sensor is attached to
    Transform local_transform; //! Transform relative to parent link
    // Additional sensor-specific data can be added here
    // For now, keep it minimal - extended by derived sensor types
};

// ****************************************************************************
//! \brief Static data for actuators driving joints.
// ****************************************************************************
struct ActuatorData
{
    std::string name;   //! Actuator name
    size_t joint_index; //! Which joint this actuator drives
    // Additional actuator-specific data can be added here
    // For now, keep it minimal - extended by derived actuator types
};

// *********************************************************************************
//! \brief Class managing a robot's tree blueprint as static robot model.
//!
//! This class encapsulates the structural information for describing a robot:
//! the topology of joints and links, their geometric relationships, and the
//! hierarchical tree structure. It represents the Pinocchio's Model separation
//! pattern (Model/Data) where Data is the structure holding robot's states.
//!
//! Key components:
//! - Kinematic tree: Hierarchical structure of joints and links.
//! - Store other objects: sensors, actuators, geometries, ...
//! - Root node: Base link of the robot (i.e. fixed to ground/table).
//! - Joint/Link caches: Fast lookup tables for components by name.
//! - Sensor/Actuator registries: access to components attached to the
//! robot tree.
//!
//! For dynamic computations, use the Robot class which takes a Blueprint
//! and operates on State objects.
// *********************************************************************************
class Blueprint
{
    // Friend declaration for the visitor that caches blueprint
    friend class CacheBlueprintVisitor;

public:

    // ------------------------------------------------------------------------
    //! \brief Constructor with root node (legacy, for backward compatibility).
    //! \param p_root Root node of the kinematic tree.
    // ------------------------------------------------------------------------
    explicit Blueprint(Node::Ptr p_root);

    // ------------------------------------------------------------------------
    //! \brief Constructor with flat arrays (new architecture).
    //!
    //! This constructor directly initializes the Blueprint with pre-built flat
    //! arrays, bypassing tree construction. This is the preferred method for
    //! URDF loaders and other model parsers.
    //!
    //! \param p_joint_data Flat array of joint data
    //! \param p_link_data Flat array of link data
    //! \param p_geometry_data Flat array of geometry data
    //! \param p_sensor_data Flat array of sensor data (optional)
    //! \param p_actuator_data Flat array of actuator data (optional)
    // ------------------------------------------------------------------------
    explicit Blueprint(std::vector<JointData>&& p_joint_data,
                       std::vector<LinkData>&& p_link_data,
                       std::vector<GeometryData>&& p_geometry_data,
                       std::vector<SensorData>&& p_sensor_data = {},
                       std::vector<ActuatorData>&& p_actuator_data = {});

    // ------------------------------------------------------------------------
    //! \brief Get the number of actuable joints.
    //! \return Number of joints (excluding fixed joints).
    // ------------------------------------------------------------------------
    inline size_t numJoints() const
    {
        return m_joint_data.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the number of links.
    //! \return Number of links.
    // ------------------------------------------------------------------------
    inline size_t numLinks() const
    {
        return m_link_data.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the number of geometries.
    //! \return Number of geometries.
    // ------------------------------------------------------------------------
    inline size_t numGeometries() const
    {
        return m_geometry_data.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get joint data by index.
    //! \param index Index of the joint (0 to numJoints()-1)
    //! \return Const reference to joint data
    //! \note Index i in m_joint_data[i] IS the joint's index. No lookup.
    // ------------------------------------------------------------------------
    inline JointData const& jointData(size_t index) const
    {
        return m_joint_data[index];
    }

    // ------------------------------------------------------------------------
    //! \brief Get link data by index.
    //! \param index Index of the link (0 to numLinks()-1)
    //! \return Const reference to link data
    // ------------------------------------------------------------------------
    inline LinkData const& linkData(size_t index) const
    {
        return m_link_data[index];
    }

    // ------------------------------------------------------------------------
    //! \brief Get geometry data by index.
    //! \param index Index of the geometry
    //! \return Const reference to geometry data
    // ------------------------------------------------------------------------
    inline GeometryData const& geometryData(size_t index) const
    {
        return m_geometry_data[index];
    }

    // ------------------------------------------------------------------------
    //! \brief Get sensor data by index.
    //! \param index Index of the sensor
    //! \return Const reference to sensor data
    // ------------------------------------------------------------------------
    inline SensorData const& sensorData(size_t index) const
    {
        return m_sensor_data[index];
    }

    // ------------------------------------------------------------------------
    //! \brief Get actuator data by index.
    //! \param index Index of the actuator
    //! \return Const reference to actuator data
    // ------------------------------------------------------------------------
    inline ActuatorData const& actuatorData(size_t index) const
    {
        return m_actuator_data[index];
    }

    // ------------------------------------------------------------------------
    //! \brief Find joint index by name.
    //! \param p_name Name of the joint
    //! \return Index of the joint
    //! \throw RobotikException if joint not found
    // ------------------------------------------------------------------------
    size_t jointIndex(std::string const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find link index by name.
    //! \param p_name Name of the link
    //! \return Index of the link
    //! \throw RobotikException if link not found
    // ------------------------------------------------------------------------
    size_t linkIndex(std::string const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find geometry index by name.
    //! \param p_name Name of the geometry
    //! \return Index of the geometry
    //! \throw RobotikException if geometry not found
    // ------------------------------------------------------------------------
    size_t geometryIndex(std::string const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find sensor index by name.
    //! \param p_name Name of the sensor
    //! \return Index of the sensor
    //! \throw RobotikException if sensor not found
    // ------------------------------------------------------------------------
    size_t sensorIndex(std::string const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find actuator index by name.
    //! \param p_name Name of the actuator
    //! \return Index of the actuator
    //! \throw RobotikException if actuator not found
    // ------------------------------------------------------------------------
    size_t actuatorIndex(std::string const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Iterate over all joints using flat array.
    //! \param p_callback Callback function: void(JointData const&, size_t
    //! index)
    //!
    //! Example:
    //! \code
    //!   blueprint.forEachJointData([](JointData const& joint, size_t i) {
    //!       std::cout << "Joint " << i << ": " << joint.name << std::endl;
    //!   });
    //! \endcode
    //!
    //! \note Index i is guaranteed to be the joint's index (i == joint index).
    //!       No lookup needed. This is cache-friendly sequential access.
    // ------------------------------------------------------------------------
    template <typename Callback>
    void forEachJointData(Callback&& p_callback) const
    {
        for (size_t i = 0; i < m_joint_data.size(); ++i)
        {
            p_callback(m_joint_data[i], i);
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Iterate over all links using flat array.
    //! \param p_callback Callback function: void(LinkData const&, size_t index)
    //!
    //! \note Index i is guaranteed to be the link's index (i == link index).
    // ------------------------------------------------------------------------
    template <typename Callback>
    void forEachLinkData(Callback&& p_callback) const
    {
        for (size_t i = 0; i < m_link_data.size(); ++i)
        {
            p_callback(m_link_data[i], i);
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Iterate over all geometries using flat array.
    //! \param p_callback Callback function: void(GeometryData const&, size_t
    //! index)
    // ------------------------------------------------------------------------
    template <typename Callback>
    void forEachGeometryData(Callback&& p_callback) const
    {
        for (size_t i = 0; i < m_geometry_data.size(); ++i)
        {
            p_callback(m_geometry_data[i], i);
        }
    }

private:

    // Flat array storage for cache-friendly algorithms
    //! \brief Flat array of joint data (index = joint index)
    std::vector<JointData> m_joint_data;
    //! \brief Flat array of link data (index = link index)
    std::vector<LinkData> m_link_data;
    //! \brief Flat array of geometry data (index = geometry index)
    std::vector<GeometryData> m_geometry_data;
    //! \brief Flat array of sensor data (index = sensor index)
    std::vector<SensorData> m_sensor_data;
    //! \brief Flat array of actuator data (index = actuator index)
    std::vector<ActuatorData> m_actuator_data;

    // Name-to-index mappings (for when you have a name and need the index)
    //! \brief Map from joint name to index in m_joint_data
    std::unordered_map<std::string, size_t> m_joint_name_to_index;
    //! \brief Map from link name to index in m_link_data
    std::unordered_map<std::string, size_t> m_link_name_to_index;
    //! \brief Map from geometry name to index in m_geometry_data
    std::unordered_map<std::string, size_t> m_geometry_name_to_index;
    //! \brief Map from sensor name to index in m_sensor_data
    std::unordered_map<std::string, size_t> m_sensor_name_to_index;
    //! \brief Map from actuator name to index in m_actuator_data
    std::unordered_map<std::string, size_t> m_actuator_name_to_index;
};

} // namespace robotik
