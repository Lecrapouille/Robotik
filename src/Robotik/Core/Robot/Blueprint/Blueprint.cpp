/**
 * @file Blueprint.cpp
 * @brief Implementation of Blueprint class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Common/Exception.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Blueprint::Blueprint(std::vector<JointData>&& p_joint_data,
                     std::vector<LinkData>&& p_link_data,
                     std::vector<GeometryData>&& p_geometry_data,
                     std::vector<SensorData>&& p_sensor_data,
                     std::vector<ActuatorData>&& p_actuator_data)
{
    // Move flat arrays
    m_joint_data = std::move(p_joint_data);
    m_link_data = std::move(p_link_data);
    m_geometry_data = std::move(p_geometry_data);
    m_sensor_data = std::move(p_sensor_data);
    m_actuator_data = std::move(p_actuator_data);

    // Build name-to-index mappings
    for (size_t i = 0; i < m_joint_data.size(); ++i)
    {
        m_joint_name_to_index[m_joint_data[i].name] = i;
    }

    for (size_t i = 0; i < m_link_data.size(); ++i)
    {
        m_link_name_to_index[m_link_data[i].name] = i;
    }

    for (size_t i = 0; i < m_geometry_data.size(); ++i)
    {
        m_geometry_name_to_index[m_geometry_data[i].name] = i;
    }

    for (size_t i = 0; i < m_sensor_data.size(); ++i)
    {
        m_sensor_name_to_index[m_sensor_data[i].name] = i;
    }

    for (size_t i = 0; i < m_actuator_data.size(); ++i)
    {
        m_actuator_name_to_index[m_actuator_data[i].name] = i;
    }

    // No tree structure needed - we work directly with flat arrays
}

// ----------------------------------------------------------------------------
size_t Blueprint::jointIndex(std::string const& p_name) const
{
    auto const& it = m_joint_name_to_index.find(p_name);
    if (it == m_joint_name_to_index.end())
    {
        throw RobotikException("Joint not found: " + p_name);
    }
    return it->second;
}

// ----------------------------------------------------------------------------
size_t Blueprint::linkIndex(std::string const& p_name) const
{
    auto const& it = m_link_name_to_index.find(p_name);
    if (it == m_link_name_to_index.end())
    {
        throw RobotikException("Link not found: " + p_name);
    }
    return it->second;
}

// ----------------------------------------------------------------------------
size_t Blueprint::geometryIndex(std::string const& p_name) const
{
    auto const& it = m_geometry_name_to_index.find(p_name);
    if (it == m_geometry_name_to_index.end())
    {
        throw RobotikException("Geometry not found: " + p_name);
    }
    return it->second;
}

// ----------------------------------------------------------------------------
size_t Blueprint::sensorIndex(std::string const& p_name) const
{
    auto const& it = m_sensor_name_to_index.find(p_name);
    if (it == m_sensor_name_to_index.end())
    {
        throw RobotikException("Sensor not found: " + p_name);
    }
    return it->second;
}

// ----------------------------------------------------------------------------
size_t Blueprint::actuatorIndex(std::string const& p_name) const
{
    auto const& it = m_actuator_name_to_index.find(p_name);
    if (it == m_actuator_name_to_index.end())
    {
        throw RobotikException("Actuator not found: " + p_name);
    }
    return it->second;
}

} // namespace robotik
