/**
 * @file VisualizerServer.cpp
 * @brief TCP server implementation using SFML
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Core/BehaviorTree/Network/VisualizerServer.hpp"

#include <iostream>
#include <sstream>

namespace robotik::bt {

// ----------------------------------------------------------------------------
VisualizerServer::~VisualizerServer()
{
    stop();
}

// ----------------------------------------------------------------------------
bool VisualizerServer::start(uint16_t const p_port)
{
    m_port = p_port;

    m_listener = std::make_unique<sf::TcpListener>();
    m_listener->setBlocking(false);

    if (m_listener->listen(int(m_port)) != sf::Socket::Done)
    {
        std::cerr << "Failed to bind to port " << int(m_port) << std::endl;
        m_listener.reset();
        return false;
    }

    std::cout << "VisualizerServer listening on port " << int(m_port)
              << std::endl;
    return true;
}

// ----------------------------------------------------------------------------
void VisualizerServer::stop()
{
    if (m_client_socket)
    {
        m_client_socket->disconnect();
        m_client_socket.reset();
    }

    if (m_listener)
    {
        m_listener->close();
        m_listener.reset();
    }

    m_connected = false;
    m_has_tree = false;
    m_yaml_data.clear();
    m_receive_buffer.clear();
    m_node_states.clear();
    m_states_updated = false;

    std::cout << "VisualizerServer stopped" << std::endl;
}

// ----------------------------------------------------------------------------
void VisualizerServer::parseStatusMessage(std::string const& msg)
{
    // Format: "S:node_id:status,node_id:status,...\n"
    if (msg.size() < 3 || msg[0] != 'S' || msg[1] != ':')
    {
        return;
    }

    std::string data = msg.substr(2);

    if (!data.empty() && data.back() == '\n')
    {
        data.pop_back();
    }

    if (data.empty())
    {
        return;
    }

    std::istringstream stream(data);
    std::string pair;

    while (std::getline(stream, pair, ','))
    {
        size_t colon_pos = pair.find(':');
        if (colon_pos == std::string::npos)
        {
            continue;
        }

        try
        {
            int node_id = std::stoi(pair.substr(0, colon_pos));
            int status = std::stoi(pair.substr(colon_pos + 1));
            m_node_states[node_id] = status;
        }
        catch (std::exception const&)
        {
            continue;
        }
    }

    m_states_updated = true;
}

// ----------------------------------------------------------------------------
void VisualizerServer::update()
{
    if (!m_listener)
        return;

    // Check for new connections
    if (!m_connected)
    {
        m_client_socket = std::make_unique<sf::TcpSocket>();

        if (m_listener->accept(*m_client_socket) == sf::Socket::Done)
        {
            std::cout << "Visualizer client connected: "
                      << m_client_socket->getRemoteAddress() << ":"
                      << m_client_socket->getRemotePort() << std::endl;

            m_client_socket->setBlocking(false);
            m_connected = true;
            m_receive_buffer.clear();
            m_has_tree = false;
            m_yaml_data.clear();
            m_node_states.clear();
            m_states_updated = false;
        }
        else
        {
            m_client_socket.reset();
        }
    }

    // Receive data from connected client
    if (m_connected && m_client_socket)
    {
        std::size_t received;
        char buffer[4096];

        sf::Socket::Status status =
            m_client_socket->receive(buffer, sizeof(buffer), received);

        if (status == sf::Socket::Done)
        {
            m_receive_buffer.append(buffer, received);

            // Process complete messages (ending with newline)
            size_t newline_pos;
            while ((newline_pos = m_receive_buffer.find('\n')) !=
                   std::string::npos)
            {
                std::string message =
                    m_receive_buffer.substr(0, newline_pos + 1);
                m_receive_buffer.erase(0, newline_pos + 1);

                // Check message type
                if (message.rfind("YAML:", 0) == 0)
                {
                    // YAML message - accumulate until END_YAML
                    m_yaml_data += message.substr(5);
                }
                else if (message.rfind("END_YAML", 0) == 0)
                {
                    // End of YAML message
                    m_has_tree = true;
                    std::cout << "Received tree data (" << m_yaml_data.size()
                              << " bytes)" << std::endl;
                }
                else if (message.rfind("S:", 0) == 0)
                {
                    // Status update message
                    parseStatusMessage(message);
                }
                else if (!m_has_tree)
                {
                    // Could be continuation of YAML data
                    m_yaml_data += message;

                    // Check for end marker in accumulated data
                    size_t end_pos = m_yaml_data.find("END_YAML");
                    if (end_pos != std::string::npos)
                    {
                        m_yaml_data = m_yaml_data.substr(0, end_pos);
                        m_has_tree = true;
                        std::cout << "Received tree data ("
                                  << m_yaml_data.size() << " bytes)"
                                  << std::endl;
                    }
                }
            }
        }
        else if (status == sf::Socket::Disconnected)
        {
            std::cout << "Visualizer client disconnected" << std::endl;
            m_client_socket.reset();
            m_connected = false;
            m_yaml_data.clear();
            m_receive_buffer.clear();
            m_has_tree = false;
            m_node_states.clear();
            m_states_updated = false;
        }
    }
}

} // namespace robotik::bt
