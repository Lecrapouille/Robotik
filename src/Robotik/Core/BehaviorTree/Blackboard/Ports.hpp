/**
 * @file Ports.hpp
 * @brief Port definitions for blackboard communication.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include <optional>
#include <string>
#include <typeindex>
#include <unordered_map>

namespace robotik::bt {

// ****************************************************************************
//! \brief Enum representing the direction of a Blackboard port.
//! Directions are similar to C++ function parameters.
//! - Input: The port is in read only mode.
//! - Output: The port is in write only mode.
//! - InOut: The port is in read and write mode.
// ****************************************************************************
enum class PortDirection
{
    Input,  ///< The port is an input for the tree node.
    Output, ///< The port is an output for the tree node.
    InOut,  ///< The port is an input and output for the tree node.
};

// ****************************************************************************
//! \brief Struct representing a port.
//!
//! A port is a connection point between a node and a blackboard. It is used to
//! pass data between a node and a blackboard.
//!
//! Key features:
//! - Type-safe storage using std::any for values of any type.
//! - Hierarchical structure: child blackboards can access parent data.
//! - Automatic parent lookup when a key is not found locally.
// ****************************************************************************
template <typename T>
struct Port
{
    Port(std::string p_name,
         PortDirection p_direction,
         std::optional<T> p_default_value = std::nullopt)
        : name(std::move(p_name)),
          direction(p_direction),
          default_value(p_default_value)
    {
    }

    //! \brief The name of the port.
    std::string name;
    //! \brief The direction of the port.
    PortDirection direction;
    //! \brief The default value of the port.
    std::optional<T> default_value;
};

// ****************************************************************************
//! \brief Class representing a list of ports.
//!
//! A port list is a collection of ports that are used to pass data between a
//! node and a blackboard. It is used to define the ports that a node provides
//! and the ports that a node requires.
//!
//! Key features:
//! - Type-safe storage using std::any for values of any type.
//! - Hierarchical structure: child blackboards can access parent data.
//! - Automatic parent lookup when a key is not found locally.
//!
//! Usage example:
//! \code
//!   PortList ports;
//!   ports.addInput<int>("input1");
//!   ports.addOutput<std::string>("output1");
//! \endcode
//!
//! The port list can be used to get the ports that a node provides and the
//! ports that a node requires.
// ****************************************************************************
class PortList
{
public:

    // ------------------------------------------------------------------------
    //! \brief Add an input port.
    //! \param[in] p_name The name of the input port.
    //! \param[in] p_default_value The default value of the input port.
    // ------------------------------------------------------------------------
    template <typename T>
    void addInput(const std::string& p_name,
                  std::optional<T> p_default_value = std::nullopt)
    {
        m_inputs[p_name] = PortInfo{typeid(T), p_default_value.has_value()};
    }

    // ------------------------------------------------------------------------
    //! \brief Add an output port.
    //! \param[in] p_name The name of the output port.
    // ------------------------------------------------------------------------
    template <typename T>
    void addOutput(const std::string& p_name)
    {
        m_outputs[p_name] = PortInfo{typeid(T), false};
    }

    // ------------------------------------------------------------------------
    //! \brief Check if a port is an input port.
    //! \param[in] p_name The name of the port.
    //! \return True if the port is an input port, false otherwise.
    // ------------------------------------------------------------------------
    bool isInput(const std::string& p_name) const
    {
        return m_inputs.find(p_name) != m_inputs.end();
    }

    // ------------------------------------------------------------------------
    //! \brief Check if a port is an output port.
    //! \param[in] p_name The name of the port.
    //! \return True if the port is an output port, false otherwise.
    // ------------------------------------------------------------------------
    bool isOutput(const std::string& p_name) const
    {
        return m_outputs.find(p_name) != m_outputs.end();
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Struct representing a port information.
    //!
    //! A port information is a collection of information about a port. It is
    //! used to store the type of the port and if the port has a default value.
    // ------------------------------------------------------------------------
    struct PortInfo
    {
        //! \brief The type of the port (e.g. int, double, std::string, etc.)
        std::type_index type;
        //! \brief True if the port has a default value, false otherwise.
        bool has_default;

        PortInfo() : type(typeid(void)), has_default(false) {}
        PortInfo(std::type_index t, bool d) : type(t), has_default(d) {}
    };

    //! \brief The input ports.
    std::unordered_map<std::string, PortInfo> m_inputs;
    //! \brief The output ports.
    std::unordered_map<std::string, PortInfo> m_outputs;
};

} // namespace robotik::bt
