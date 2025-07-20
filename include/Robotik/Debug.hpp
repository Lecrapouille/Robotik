#pragma once

#include "Robotik/Robot.hpp"

namespace robotik::debug
{

// ****************************************************************************
//! \brief Print the robot to the console.
// ****************************************************************************
void printRobot(const Robot& p_robot);

// ****************************************************************************
//! \brief Print the robot scene graph in hierarchical format.
// ****************************************************************************
void printSceneGraph(const Robot& p_robot);

// ****************************************************************************
//! \brief Print a node with its transformation matrices.
// ****************************************************************************
void printNode(Node const& p_node, size_t p_depth);

// ****************************************************************************
//! \brief Print a node in simplified scene graph format.
// ****************************************************************************
void printSceneNodeSimple(Node const& p_node, bool p_isLast, size_t p_depth);

// ****************************************************************************
//! \brief Print the joint to the console.
// ****************************************************************************
void printJoint(const Joint& p_joint);

// ****************************************************************************
//! \brief Print the link to the console.
// ****************************************************************************
// void printLink(const Link& p_link);

// ****************************************************************************
//! \brief Print a transformation matrix with proper formatting.
// ****************************************************************************
void printTransform(Transform const& p_transform, const std::string& p_indent);

// ****************************************************************************
//! \brief Get string representation of joint type.
// ****************************************************************************
std::string jointTypeString(Joint::Type p_type);

// ****************************************************************************
//! \brief Get string representation of joint type for scene graph display.
// ****************************************************************************
std::string jointTypeStringCompact(Joint::Type p_type);

// ****************************************************************************
//! \brief Format transformation matrix as readable string.
// ****************************************************************************
std::string formatTransform(const Transform& p_transform);

} // namespace robotik::debug