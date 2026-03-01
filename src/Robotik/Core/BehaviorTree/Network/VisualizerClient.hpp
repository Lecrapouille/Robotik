/**
 * @file VisualizerClient.hpp
 * @brief TCP client for sending behavior tree state to visualizer.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include <SFML/Network.hpp>

#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace robotik::bt {

// Forward declarations
class Tree;
class Node;
class Composite;
class Decorator;
class SubTreeNode;

// ****************************************************************************
//! \brief TCP client that sends behavior tree state to a visualizer server.
//!
//! The VisualizerClient connects to an Oakular server (or compatible) and
//! sends:
//! - Initial tree structure as YAML
//! - State changes (node status updates) during execution
//!
//! Usage:
//! \code
//!   auto client = std::make_shared<VisualizerClient>();
//!   if (client->connect("localhost", 8888)) {
//!       tree.setVisualizerClient(client);
//!       client->sendTree(tree);
//!   }
//! \endcode
// ****************************************************************************
class VisualizerClient
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    VisualizerClient() = default;

    // ------------------------------------------------------------------------
    //! \brief Destructor - disconnects if connected.
    // ------------------------------------------------------------------------
    ~VisualizerClient()
    {
        disconnect();
    }

    // ------------------------------------------------------------------------
    //! \brief Connect to visualizer server.
    //! \param[in] p_host Server hostname or IP address.
    //! \param[in] p_port Server port (default: 8888).
    //! \return true if connection successful.
    // ------------------------------------------------------------------------
    bool connect(std::string const& p_host, uint16_t p_port = 8888)
    {
        if (m_socket.connect(p_host, p_port, sf::seconds(2)) !=
            sf::Socket::Status::Done)
        {
            return false;
        }
        m_connected = true;
        m_socket.setBlocking(false);
        return true;
    }

    // ------------------------------------------------------------------------
    //! \brief Disconnect from server.
    // ------------------------------------------------------------------------
    void disconnect()
    {
        if (m_connected)
        {
            m_socket.disconnect();
            m_connected = false;
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Check if connected to server.
    //! \return true if connected.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isConnected() const
    {
        return m_connected;
    }

    // ------------------------------------------------------------------------
    //! \brief Send the tree structure to the visualizer.
    //! \param[in] p_tree The tree to send.
    //! \return true if sent successfully.
    // ------------------------------------------------------------------------
    bool sendTree(Tree const& p_tree);

    // ------------------------------------------------------------------------
    //! \brief Send state changes after a tick.
    //! Sends only nodes whose status changed since last send.
    //! \param[in] p_tree The tree to scan for changes.
    // ------------------------------------------------------------------------
    void sendStateChanges(Tree const& p_tree);

private:

    // ------------------------------------------------------------------------
    //! \brief Send a message with length prefix.
    //! \param[in] p_type Message type prefix ('T' for tree, 'S' for status).
    //! \param[in] p_data The data to send.
    //! \return true if sent successfully.
    // ------------------------------------------------------------------------
    bool sendMessage(char p_type, std::string const& p_data)
    {
        if (!m_connected)
        {
            return false;
        }

        std::string message = std::string(1, p_type) + ":" + p_data + "\n";
        size_t sent = 0;
        auto status = m_socket.send(message.data(), message.size(), sent);
        return (status == sf::Socket::Status::Done ||
                status == sf::Socket::Status::Partial);
    }

    // ------------------------------------------------------------------------
    //! \brief Collect node states recursively.
    //! \param[in] p_node The node to start from.
    //! \param[out] p_states Output string stream for state data.
    // ------------------------------------------------------------------------
    void collectNodeStates(Node const& p_node, std::ostringstream& p_states);

    sf::TcpSocket m_socket;
    bool m_connected = false;
};

} // namespace robotik::bt
