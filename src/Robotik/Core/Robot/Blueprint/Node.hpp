/**
 * @file Node.hpp
 * @brief Blueprint node class - Representation of a node in the kinematic
 * blueprint.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace robotik
{

// Forward declarations for Visitor pattern
class NodeVisitor;
class ConstNodeVisitor;

// ****************************************************************************
//! \brief Class representing a node inside the tree blueprint.
//!
//! A tree blueprint is a tree data structure that represents the spatial
//! relationships between its different components. Each node in this blueprint
//! represents a physical or logical component that has:
//! - A position and orientation in 3D space (transform).
//! - A relationship to a parent component (inheritance of transforms).
//! - Potentially multiple child components.
//!
//! A tree blueprint is more widely known under the following names:
//! - Scene graph: but this a name is used in video games.
//! - Kinematic tree: but this is constrain to Link and Joint.
//! Our tree blueprint is more general (scene tree) and can store other objects
//! (sensors, actuators, visual geometries, ...).
//!
//! Physical representation in robotics:
//! - Robot links (rigid bodies connecting joints).
//! - Robot joints (coordinate systems at joint locations).
//! - End-effector frames (end-effector coordinate systems).
//! - Sensors (camera, lidar, ...).
//! - Actuators (motor, ...).
//! - Geometries (for visualization and collision detection).
//!
//! The tree blueprint enables:
//! - Forward kinematics calculation (computing end-effector pose).
//! - Coordinate frame transformations between robot components.
//! - Hierarchical motion propagation (parent motion affects all children).
//! - Efficient spatial queries and collision detection.
//! - Visualization and rendering of robot components.
//!
//! Example robot blueprint:
//! Base -> Shoulder -> Upper_Arm -> Elbow -> Forearm -> Wrist -> End_Effector
// ****************************************************************************
class Node
{
public:

    using Ptr = std::unique_ptr<Node>;

    // ------------------------------------------------------------------------
    //! \brief Constructor. Initializes the node with a name and transformation
    //! to identity matrix.
    //! \param p_name Name of the node.
    // ------------------------------------------------------------------------
    explicit Node(std::string const& p_name);

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor to ensure proper cleanup of derived classes.
    // ------------------------------------------------------------------------
    virtual ~Node() = default;

    // ------------------------------------------------------------------------
    //! \brief Disable copy constructor and copy assignment due to unique_ptr
    // ------------------------------------------------------------------------
    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Enable move constructor and move assignment
    // ------------------------------------------------------------------------
    Node(Node&&) = default;
    Node& operator=(Node&&) = default;

    // ------------------------------------------------------------------------
    //! \brief Create a new node of type T derived from Node.
    //! \tparam T Type of the node to create (must inherit from Node).
    //! \param p_args Arguments passed to the constructor.
    //! \return Unique pointer to the new node.
    // ------------------------------------------------------------------------
    template <typename T, typename... Args>
    static std::unique_ptr<T> create(Args&&... p_args)
    {
        static_assert(std::is_base_of<Node, T>::value,
                      "T must inherit from Node");
        return std::make_unique<T>(std::forward<Args>(p_args)...);
    }

    // ------------------------------------------------------------------------
    //! \brief Create and store a new child node of type T derived from Node.
    //! \tparam T Type of the child node to create (must inherit from Node).
    //! \param p_args Arguments passed to the constructor.
    //! \return Reference to the added child node.
    // ------------------------------------------------------------------------
    template <typename T, typename... Args>
    T& createChild(Args&&... p_args)
    {
        auto child = Node::create<T>(std::forward<Args>(p_args)...);
        T& ref = *child;
        addChild(std::move(child));
        return ref;
    }

    // ------------------------------------------------------------------------
    //! \brief Add a child node created outside of this class.
    //!
    //! This method is used to add a child node that was created outside of
    //! this class.
    //! \param p_child Pointer to the child node.
    //! \note p_child is moved into this class.
    // ------------------------------------------------------------------------
    void addChild(Node::Ptr p_child)
    {
        m_children.push_back(std::move(p_child));
        m_children.back()->m_parent = this;
        m_children.back()->markTransformsDirty();
    }

    // ------------------------------------------------------------------------
    //! \brief Const getter to children of the node.
    //! \return Const vector of child nodes.
    // ------------------------------------------------------------------------
    inline std::vector<std::unique_ptr<Node>> const& children() const
    {
        return m_children;
    }

    // ------------------------------------------------------------------------
    //! \brief Non-const getter to children of the node.
    //! \return Vector of child nodes.
    // ------------------------------------------------------------------------
    inline std::vector<std::unique_ptr<Node>>& children()
    {
        return m_children;
    }

    // ------------------------------------------------------------------------
    //! \brief Get a child node by name.
    //! \param p_name Name of the child node.
    //! \return Pointer to the child node, or nullptr if not found.
    // ------------------------------------------------------------------------
    Node const* child(std::string const& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get a child node by name.
    //! \param p_name Name of the child node.
    //! \return Pointer to the child node, or nullptr if not found.
    // ------------------------------------------------------------------------
    Node* child(std::string const& p_name);

    // ------------------------------------------------------------------------
    //! \brief Get a node by name. This method is recursive and will search
    //! through the entire subtree rooted at this node.
    //! \param p_root Root node to start the search from.
    //! \param p_name Name of the node.
    //! \return Pointer to the node, or nullptr if not found.
    // ------------------------------------------------------------------------
    static Node const*
    find(Node const& p_root,
         std::string const& p_name); // FIXME: findChild<T>(name) et
                                     // findChildren<T>(name) et name optionel

    // ------------------------------------------------------------------------
    //! \brief Get a node by name. This method is recursive and will search
    //! through the entire subtree rooted at this node.
    //! \param p_root Root node to start the search from.
    //! \param p_name Name of the node.
    //! \return Pointer to the node, or nullptr if not found.
    // ------------------------------------------------------------------------
    static Node* find(Node& p_root, std::string const& p_name);

    // ------------------------------------------------------------------------
    //! \brief Accept a visitor (Visitor pattern).
    //!
    //! This method implements the Visitor pattern, allowing operations on
    //! nodes without adding methods to the node blueprint. The visitor will
    //! call the appropriate visit() method based on the actual node type.
    //!
    //! \param visitor The visitor to accept.
    // ------------------------------------------------------------------------
    virtual void accept(NodeVisitor& visitor);

    // ------------------------------------------------------------------------
    //! \brief Accept a const visitor (Visitor pattern).
    //! \param visitor The const visitor to accept.
    // ------------------------------------------------------------------------
    virtual void accept(ConstNodeVisitor& visitor) const;

    // ------------------------------------------------------------------------
    //! \brief Traverse the node tree using a visitor.
    //!
    //! This method traverses the tree recursively using the Visitor pattern.
    //! The visitor maintains its own depth state during traversal.
    //!
    //! \param visitor The visitor to use for traversal.
    // ------------------------------------------------------------------------
    void traverse(NodeVisitor& visitor);

    // ------------------------------------------------------------------------
    //! \brief Traverse the node tree using a const visitor.
    //! \param visitor The const visitor to use for traversal.
    // ------------------------------------------------------------------------
    void traverse(ConstNodeVisitor& visitor) const;

    // ------------------------------------------------------------------------
    //! \brief Traverse the node tree recursively and apply a function to each
    //! node.
    //!
    //! This is the const version of traverse, which does nothing since const
    //! methods cannot modify the object state.
    //! \tparam Function Type of the function to apply (lambda, function
    //! pointer, etc.)
    //! \param p_function Function to apply to each node. Should accept
    //! Node& parameter.
    //! \param p_depth Depth of the node in the tree.
    // ------------------------------------------------------------------------
    template <
        typename Function,
        typename = typename std::enable_if<
            !std::is_base_of<ConstNodeVisitor,
                             typename std::decay<Function>::type>::value &&
            !std::is_base_of<NodeVisitor,
                             typename std::decay<Function>::type>::value>::type>
    void traverse(Function&& p_function, size_t p_depth = 0) const
    {
        // Apply the function to the current node
        p_function(*this, p_depth);

        // Recursively traverse all children
        for (auto const& child : m_children)
        {
            child->traverse(std::forward<Function>(p_function), p_depth + 1);
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Traverse the node tree recursively and apply a function to each
    //! node.
    //!
    //! This method performs a depth-first traversal of the node tree starting
    //! from this node. It applies the provided function to each node in the
    //! tree, including the current node and all its descendants.
    //!
    //! The traversal order is:
    //! 1. Apply function to current node
    //! 2. Recursively traverse all children
    //!
    //! This is useful for operations that need to be applied to all nodes
    //! in a subtree, such as:
    //! - Collecting all joints in a robot arm
    //! - Updating properties of all nodes
    //! - Searching for specific node types
    //!
    //! \tparam Function Type of the function to apply (lambda, function
    //! pointer, etc.)
    //! \param p_function Function to apply to each node. Should accept
    //! Node& parameter.
    //! \param p_depth Depth of the node in the tree.
    // ------------------------------------------------------------------------
    template <
        typename Function,
        typename = typename std::enable_if<
            !std::is_base_of<ConstNodeVisitor,
                             typename std::decay<Function>::type>::value &&
            !std::is_base_of<NodeVisitor,
                             typename std::decay<Function>::type>::value>::type>
    void traverse(Function&& p_function, size_t p_depth = 0)
    {
        // Apply the function to the current node
        p_function(*this, p_depth);

        // Recursively traverse all children
        for (auto const& child : m_children)
        {
            child->traverse(std::forward<Function>(p_function), p_depth + 1);
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Set the local transform of the node.
    //!
    //! The local transform defines the position and orientation of this
    //! node relative to its parent in the blueprint. In the context of
    //! robotics, this typically corresponds to:
    //! - For a link: its position/orientation relative to the parent joint
    //! - For a joint: the transformation it applies (rotation/translation)
    //!
    //! This 4x4 homogeneous matrix encodes:
    //! - Translation (first 3 elements of the 4th column)
    //! - Rotation (3x3 upper left submatrix)
    //! - Scale factor (typically 1 in robotics)
    //!
    //! \param p_transform Local homogeneous transformation matrix 4x4
    // ------------------------------------------------------------------------
    virtual void localTransform(Transform const& p_transform);

    // ------------------------------------------------------------------------
    //! \brief Get the local transform of the node.
    //!
    //! The local transform describes the position and orientation of this
    //! node relative to its parent in the blueprint. This transformation is
    //! expressed in the parent's coordinate frame.
    //!
    //! In robotics, this transformation represents:
    //! - The geometric offset between two consecutive links
    //! - The transformation applied by a joint (rotation or translation)
    //! - The physical dimensions of the robot's links
    //!
    //! \return Constant reference to the local transformation matrix 4x4
    // ------------------------------------------------------------------------
    virtual Transform const& localTransform() const
    {
        return m_local_transform;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the world transform of the node.
    //!
    //! The world transform describes the absolute position and orientation
    //! of this node in the global/base coordinate frame. This
    //! transformation is calculated by multiplying all local transforms
    //! from the root to this node.
    //!
    //! In robotics, this transformation represents:
    //! - Position/orientation of a link in the robot's workspace
    //! - End-effector pose relative to the robot's base
    //! - Absolute configuration of any part of the robot
    //!
    //! The world transform = T_base * T_joint1 * ... * T_jointN * T_local
    //! where each T_jointX is
    //!
    //! LAZY EVALUATION: This method uses dirty flags to avoid redundant
    //! recalculations. The transforms are only updated when needed (when
    //! accessed and marked as dirty).
    // ------------------------------------------------------------------------
    inline Transform const& worldTransform() const
    {
        if (m_transforms_dirty)
        {
            updateWorldTransforms();
        }
        return m_world_transform;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the name of the node.
    //! \return Name of the node.
    // ------------------------------------------------------------------------
    inline const std::string& name() const
    {
        return m_name;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the node is enabled/visible.
    //!
    //! This flag controls the visibility and activation state of nodes
    //! (links, joints, sensors). When disabled, a node may not be rendered
    //! or processed depending on the use case.
    //!
    //! \return True if node is enabled, false otherwise.
    // ------------------------------------------------------------------------
    inline bool enabled() const
    {
        return m_enabled;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the enabled/visible state of the node.
    //!
    //! \param p_enabled True to enable the node, false to disable it.
    // ------------------------------------------------------------------------
    inline void enabled(bool p_enabled)
    {
        m_enabled = p_enabled;
    }

protected:

    // ------------------------------------------------------------------------
    //! \brief Mark this node and all its children as needing transform update.
    //!
    //! This method implements the "dirty flag" pattern for lazy evaluation.
    //! Instead of immediately recalculating world transforms when a joint
    //! moves, we mark the affected nodes as dirty. The actual recalculation
    //! happens only when worldTransform() is accessed.
    //!
    //! This optimization eliminates redundant recalculations when multiple
    //! joints are updated in sequence (e.g., in setNeutralPosition() or
    //! setJointPositions()), reducing complexity from O(n²) to O(n).
    //!
    //! The dirty flag propagates to all children because changing a parent's
    //! transform invalidates all descendant transforms in the kinematic chain.
    // ------------------------------------------------------------------------
    void markTransformsDirty();

    // ------------------------------------------------------------------------
    //! \brief Update the world transform of the node.
    //!
    //! Updates the world transform of this node and all its children
    //! recursively. This method implements the propagation of transforms in
    //! the robot's kinematic chain.
    //!
    //! The calculation follows the formula:
    //! - If root node: T_world = T_local
    //! - Otherwise: T_world = T_world_parent * T_local
    //!
    //! In robotics, this update is necessary when:
    //! - A joint changes value (angle or position)
    //! - The robot's configuration is modified
    //! - The direct kinematics is recalculated
    //!
    //! This method is called automatically by worldTransform() when the
    //! dirty flag is set, implementing lazy evaluation.
    // ------------------------------------------------------------------------
    void updateWorldTransforms() const;

protected:

    //! \brief Name of the node.
    std::string m_name;
    //! \brief Local transformation of the node.
    //! \note Do not use directly, use the virtual localTransform() instead that
    //! should use it.
    Transform m_local_transform;
    //! \brief World transformation of the node.
    mutable Transform m_world_transform;
    //! \brief Children of the node.
    std::vector<std::unique_ptr<Node>> m_children;
    //! \brief Parent of the node.
    Node* m_parent = nullptr;
    //! \brief Dirty flag for lazy transform evaluation.
    //! When true, indicates that the world transform needs recalculation.
    mutable bool m_transforms_dirty = false;
    //! \brief Enabled/visible flag for the node.
    //! Controls whether the node should be rendered or processed.
    bool m_enabled = true;
};

} // namespace robotik