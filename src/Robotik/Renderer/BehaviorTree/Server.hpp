/**
 * @file VisualizerServer.hpp
 * @brief TCP server for behavior tree visualization
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include <SFML/Network.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

// ****************************************************************************
//! \briefTCP server that receives behavior tree data from clients
//!
//! The server listens for incoming connections and receives YAML data
//! representing behavior trees. This allows runtime visualization of
//! behavior trees running in other processes.
// ****************************************************************************
class Server
{
public:

    ~Server();

    // ------------------------------------------------------------------------
    //! \brief Start the TCP server
    //! \param p_port Port to listen on (default: 8888)
    //! \return true if server started successfully
    // ------------------------------------------------------------------------
    bool start(uint16_t const p_port = 8888);

    // ------------------------------------------------------------------------
    //! \brief Stop the server
    // ------------------------------------------------------------------------
    void stop();

    // ------------------------------------------------------------------------
    //! \brief Update server state (check for connections, receive data)
    //! Call this each frame
    // ------------------------------------------------------------------------
    void update();

    // ------------------------------------------------------------------------
    //! \brief Check if a client is connected
    // ------------------------------------------------------------------------
    bool isConnected() const
    {
        return m_connected;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if tree data has been received
    // ------------------------------------------------------------------------
    bool hasReceivedTree() const
    {
        return m_has_tree;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the received YAML data
    // ------------------------------------------------------------------------
    std::string getYamlData() const
    {
        return m_yaml_data;
    }

    // ------------------------------------------------------------------------
    //! \brief Clear received data flag
    // ------------------------------------------------------------------------
    void clearTreeData()
    {
        m_has_tree = false;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if state updates have been received
    // ------------------------------------------------------------------------
    bool hasStateUpdate() const
    {
        return m_states_updated;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the state of a node by its ID.
    //! \param[in] p_node_id The unique ID of the node.
    //! \return The status (0=INVALID, 1=RUNNING, 2=SUCCESS, 3=FAILURE), or 0
    //! if not found.
    // ------------------------------------------------------------------------
    int getNodeState(int p_node_id) const
    {
        auto it = m_node_states.find(p_node_id);
        return (it != m_node_states.end()) ? it->second : 0;
    }

    // ------------------------------------------------------------------------
    //! \brief Clear the states update flag after reading.
    // ------------------------------------------------------------------------
    void clearStateUpdate()
    {
        m_states_updated = false;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Parse a status message and update node states.
    //! \param[in] msg The message in format
    //! "S:node_id:status,node_id:status,..."
    // ------------------------------------------------------------------------
    void parseStatusMessage(std::string const& msg);

    std::unique_ptr<sf::TcpListener> m_listener;
    std::unique_ptr<sf::TcpSocket> m_client_socket;
    bool m_connected = false;
    bool m_has_tree = false;
    uint16_t m_port = 8888;
    std::string m_yaml_data;
    std::string m_receive_buffer; //!< Buffer for partial messages

    //! \brief Node states by node ID (0=INVALID, 1=RUNNING, 2=SUCCESS,
    //! 3=FAILURE)
    std::unordered_map<int, int> m_node_states;
    //! \brief Flag indicating if states have been updated since last read
    bool m_states_updated = false;
};
