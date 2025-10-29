/**
 * @file TestSceneNode.cpp
 * @brief Unit tests for the SceneNode class - Verification of scene graph
 * construction, management, local and world transformations, and node traversal
 * operations. The scene graph is the structure holding the robot links and
 * joints.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Core/Node.hpp"

#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for node class.
// *********************************************************************************
class SceneNodeTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        root = Node::create<Node>("root");
    }

    Node::Ptr root;
};

// *********************************************************************************
//! \brief Test node creation and basic properties.
// *********************************************************************************
TEST_F(SceneNodeTest, Creation)
{
    EXPECT_EQ(root->name(), "root");
    EXPECT_TRUE(root->children().empty());
    EXPECT_TRUE(root->localTransform().isApprox(Eigen::Matrix4d::Identity()));
    EXPECT_TRUE(root->worldTransform().isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test name() method and find() method.
// *********************************************************************************
TEST_F(SceneNodeTest, GetName)
{
    EXPECT_EQ(root->name(), "root");

    // Test getting non-existent node
    EXPECT_EQ(Node::find(*root, "nonexistent"), nullptr);
    EXPECT_EQ(Node::find(*root, ""), nullptr);

    Node const& child = root->createChild<Node>("test_child");
    EXPECT_EQ(child.name(), "test_child");
    EXPECT_EQ(Node::find(*root, "test_child"), &child);
    EXPECT_EQ(root->child("test_child"), &child);

    // Test with empty name
    Node const& empty_child = root->createChild<Node>("");
    EXPECT_EQ(empty_child.name(), "");
    EXPECT_EQ(Node::find(*root, ""), &empty_child);
    EXPECT_EQ(root->child(""), &empty_child);
}

// *********************************************************************************
//! \brief Test adding children and parent-child relationships and world
//! transform updates.
// *********************************************************************************
TEST_F(SceneNodeTest, CreateChildAndParenthood)
{
    // Create two children
    Node& child1 = root->createChild<Node>("child1");
    Node& child2 = root->createChild<Node>("child2");

    // Test that the children have the correct name
    EXPECT_EQ(child1.name(), "child1");
    EXPECT_EQ(child2.name(), "child2");

    // Test that the children are found from the root node
    EXPECT_EQ(root->children().size(), 2);
    EXPECT_EQ(Node::find(*root, "child1"), &child1);
    EXPECT_EQ(Node::find(*root, "child2"), &child2);
    EXPECT_EQ(root->child("child1"), &child1);
    EXPECT_EQ(root->child("child2"), &child2);

    // Verify parent-child relationships through world transform updates
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(0, 3) = 5.0; // Translation in x
    root->localTransform(parent_transform);

    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(1, 3) = 3.0; // Translation in y
    child1.localTransform(child_transform);

    // Compare with explicit matrix:
    // 1 0 0 5
    // 0 1 0 3
    // 0 0 1 0
    // 0 0 0 1
    Transform expected_matrix = Eigen::Matrix4d::Identity();
    expected_matrix(0, 3) = 5.0; // Translation in x from parent
    expected_matrix(1, 3) = 3.0; // Translation in y from child
    EXPECT_TRUE(child1.worldTransform().isApprox(expected_matrix));
}

// *********************************************************************************
//! \brief Test hierarchical structure with multiple levels with the find()
//! method.
// *********************************************************************************
TEST_F(SceneNodeTest, ComplexBlueprint)
{
    // Create a blueprint of nodes
    Node& child1 = root->createChild<Node>("child1");
    Node const& child2 = root->createChild<Node>("child2");
    Node& grandchild1 = child1.createChild<Node>("grandchild1");
    Node const& grandchild2 = child1.createChild<Node>("grandchild2");
    Node const& great_grandchild =
        grandchild1.createChild<Node>("great_grandchild");

    // Test that the children are found from the parent nodes
    EXPECT_EQ(root->children().size(), 2);
    EXPECT_EQ(child1.children().size(), 2);
    EXPECT_EQ(child2.children().size(), 0);
    EXPECT_EQ(grandchild1.children().size(), 1);
    EXPECT_EQ(grandchild2.children().size(), 0);
    EXPECT_EQ(great_grandchild.children().size(), 0);

    // Test that the nodes are found from the root node
    EXPECT_EQ(Node::find(*root, "child1"), &child1);
    EXPECT_EQ(Node::find(*root, "child2"), &child2);
    EXPECT_EQ(Node::find(child1, "grandchild1"), &grandchild1);
    EXPECT_EQ(Node::find(child1, "grandchild2"), &grandchild2);
    EXPECT_EQ(Node::find(grandchild1, "great_grandchild"), &great_grandchild);

    // Test getting non-existent node
    EXPECT_EQ(Node::find(*root, "nonexistent"), nullptr);

    // Test with empty name search
    EXPECT_EQ(Node::find(*root, ""), nullptr);
}

// *********************************************************************************
//! \brief Test complex world transform propagation through blueprint.
// *********************************************************************************
TEST_F(SceneNodeTest, ComplexTransformationPropagation)
{
    // Create a 3-level blueprint
    Node& level1 = root->createChild<Node>("level1");
    Node& level2 = level1.createChild<Node>("level2");
    Node& level3 = level2.createChild<Node>("level3");

    // Set transforms for each level
    Transform root_transform = Eigen::Matrix4d::Identity();
    root_transform(0, 3) = 1.0; // Translation in x
    root->localTransform(root_transform);

    Transform level1_transform = Eigen::Matrix4d::Identity();
    level1_transform(1, 3) = 2.0; // Translation in y
    level1.localTransform(level1_transform);

    Transform level2_transform = Eigen::Matrix4d::Identity();
    level2_transform(2, 3) = 3.0; // Translation in z
    level2.localTransform(level2_transform);

    Transform level3_transform = Eigen::Matrix4d::Identity();
    level3_transform(0, 3) = 4.0; // Another translation in x
    level3.localTransform(level3_transform);

    // Compare with explicit matrix:
    // 1 0 0 1
    // 0 1 0 2
    // 0 0 1 0
    // 0 0 0 1
    Transform expected_matrix = Eigen::Matrix4d::Identity();
    expected_matrix(0, 3) = 1.0; // Translation in x from root
    expected_matrix(1, 3) = 2.0; // Translation in y from level1
    EXPECT_TRUE(level1.worldTransform().isApprox(expected_matrix));

    // Compare with explicit matrix:
    // 1 0 0 1
    // 0 1 0 2
    // 0 0 1 3
    // 0 0 0 1
    expected_matrix(2, 3) = 3.0; // Translation in z from level2
    EXPECT_TRUE(level2.worldTransform().isApprox(expected_matrix));

    // Compare with explicit matrix:
    // 1 0 0 5
    // 0 1 0 2
    // 0 0 1 3
    // 0 0 0 1
    expected_matrix(0, 3) = 5.0; // Translation in x from root and level3
    EXPECT_TRUE(level3.worldTransform().isApprox(expected_matrix));
}

// *********************************************************************************
//! \brief Test local transformations with rotations.
// *********************************************************************************
TEST_F(SceneNodeTest, LocalTransformWithRotation)
{
    Transform identity = Eigen::Matrix4d::Identity();
    EXPECT_TRUE(root->localTransform().isApprox(identity));

    // Set a rotation around Z axis (90 degrees) + translation on root node
    Transform rotation_transform = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rotation(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    rotation_transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    rotation_transform(0, 3) = 1.0;
    rotation_transform(1, 3) = 2.0;
    rotation_transform(2, 3) = 3.0;
    root->localTransform(rotation_transform);
    EXPECT_TRUE(root->localTransform().isApprox(rotation_transform));

    // Create child node
    Node& level1 = root->createChild<Node>("level1");

    // Compare with explicit matrix:
    // 0 -1  0  1
    // 1  0  0  2
    // 0  0  1  3
    // 0  0  0  1
    Transform expected_matrix;
    expected_matrix << 0, -1, 0, 1, 1, 0, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1;
    EXPECT_TRUE(level1.worldTransform().isApprox(expected_matrix));

    // Create a rotation around Z axis (90 degrees)
    rotation_transform = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd rotation2(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    rotation_transform.block<3, 3>(0, 0) = rotation2.toRotationMatrix();
    level1.localTransform(rotation_transform);
    Node const& level2 = level1.createChild<Node>("level2");

    // Compare with explicit matrix:
    // -1  0  0  1
    //  0 -1  0  2
    //  0  0  1  3
    //  0  0  0  1
    expected_matrix << -1, 0, 0, 1, 0, -1, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1;
    EXPECT_TRUE(level2.worldTransform().isApprox(expected_matrix));
}

// *********************************************************************************
//! \brief Test parent node with two rotations and world transform propagation.
// *********************************************************************************
TEST_F(SceneNodeTest, ParentWithTwoRotationsAndWorldTransform)
{
    // Create child nodes
    Node& child1 = root->createChild<Node>("child1");
    Node& child2 = root->createChild<Node>("child2");

    // Create parent transform with two rotations
    Transform parent_transform = Eigen::Matrix4d::Identity();

    // First rotation: 45 degrees around Z axis
    double angle1 = M_PI / 4.0;
    Transform rotation1 = Eigen::Matrix4d::Identity();
    rotation1(0, 0) = cos(angle1);
    rotation1(0, 1) = -sin(angle1);
    rotation1(1, 0) = sin(angle1);
    rotation1(1, 1) = cos(angle1);

    // Second rotation: 30 degrees around X axis
    double angle2 = M_PI / 6.0;
    Transform rotation2 = Eigen::Matrix4d::Identity();
    rotation2(1, 1) = cos(angle2);
    rotation2(1, 2) = -sin(angle2);
    rotation2(2, 1) = sin(angle2);
    rotation2(2, 2) = cos(angle2);

    // Combine rotations and add translation
    parent_transform = rotation1 * rotation2;
    parent_transform(0, 3) = 5.0; // Translation in x
    parent_transform(1, 3) = 3.0; // Translation in y
    parent_transform(2, 3) = 2.0; // Translation in z

    // Set parent transform
    root->localTransform(parent_transform);

    // Set child transforms
    Transform child1_transform = Eigen::Matrix4d::Identity();
    child1_transform(0, 3) = 1.0; // Translation in x
    child1.localTransform(child1_transform);

    Transform child2_transform = Eigen::Matrix4d::Identity();
    child2_transform(1, 3) = 2.0; // Translation in y
    child2.localTransform(child2_transform);

    // Verify world transforms
    Transform expected_child1_world = parent_transform * child1_transform;
    Transform expected_child2_world = parent_transform * child2_transform;

    EXPECT_TRUE(child1.worldTransform().isApprox(expected_child1_world));
    EXPECT_TRUE(child2.worldTransform().isApprox(expected_child2_world));

    // Verify parent world transform equals its local transform (since it's the
    // root)
    EXPECT_TRUE(root->worldTransform().isApprox(parent_transform));

    // Verify against expected matrix:
    // 0.707107 -0.612372  0.353553 5
    // 0.707107  0.612372 -0.353553 3
    // 0         0.5       0.866025 2
    // 0         0         0        1
    Transform expected_matrix;
    expected_matrix << 0.707107, -0.612372, 0.353553, 5, 0.707107, 0.612372,
        -0.353553, 3, 0, 0.5, 0.866025, 2, 0, 0, 0, 1;
    EXPECT_TRUE(root->worldTransform().isApprox(expected_matrix, 1e-5));
}

// *********************************************************************************
//! \brief Test world transformations with multiple children.
// *********************************************************************************
TEST_F(SceneNodeTest, WorldTransformMultipleChildren)
{
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(0, 3) = 1.0; // Translation in x
    root->localTransform(parent_transform);

    Node& child1 = root->createChild<Node>("child1");
    Node& child2 = root->createChild<Node>("child2");

    Transform child1_transform = Eigen::Matrix4d::Identity();
    child1_transform(1, 3) = 2.0; // Translation in y
    child1.localTransform(child1_transform);

    Transform child2_transform = Eigen::Matrix4d::Identity();
    child2_transform(2, 3) = 3.0; // Translation in z
    child2.localTransform(child2_transform);

    // Both children should have parent transform applied
    Transform expected_child1 = parent_transform * child1_transform;
    Transform expected_child2 = parent_transform * child2_transform;
    EXPECT_TRUE(child1.worldTransform().isApprox(expected_child1));
    EXPECT_TRUE(child2.worldTransform().isApprox(expected_child2));

    // Verify against expected matrix:
    // 1 0 0 1
    // 0 1 0 2
    // 0 0 1 0
    // 0 0 0 1
    Transform expected_matrix;
    expected_matrix << 1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 0, 0, 0, 0, 1;
    EXPECT_TRUE(child1.worldTransform().isApprox(expected_matrix));

    // Verify against expected matrix:
    // 1 0 0 1
    // 0 1 0 0
    // 0 0 1 3
    // 0 0 0 1
    expected_matrix << 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 3, 0, 0, 0, 1;
    EXPECT_TRUE(child2.worldTransform().isApprox(expected_matrix));
}

// *********************************************************************************
//! \brief Test transform again.
// *********************************************************************************
TEST_F(SceneNodeTest, DirtyFlagMechanism)
{
    Node& child1 = root->createChild<Node>("child1");
    Node const& grandchild = child1.createChild<Node>("grandchild");

    // Set initial transforms
    Transform root_transform = Eigen::Matrix4d::Identity();
    root_transform(0, 3) = 1.0;
    root->localTransform(root_transform);

    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(1, 3) = 2.0;
    child1.localTransform(child_transform);

    // Store initial world transforms
    Transform initial_child_world = child1.worldTransform();
    Transform initial_grandchild_world = grandchild.worldTransform();

    // Change root transform - should mark all descendants as dirty
    root_transform(0, 3) = 5.0;
    root->localTransform(root_transform);

    // World transforms should be different now
    EXPECT_FALSE(child1.worldTransform().isApprox(initial_child_world));
    EXPECT_FALSE(
        grandchild.worldTransform().isApprox(initial_grandchild_world));
}

// *********************************************************************************
//! \brief Test performance of world transform caching.
// *********************************************************************************
TEST_F(SceneNodeTest, WorldTransformCaching)
{
    Node const& child = root->createChild<Node>("child");

    Transform test_transform = Eigen::Matrix4d::Identity();
    test_transform(0, 3) = 1.0;
    root->localTransform(test_transform);

    // First call should compute world transform
    Transform const& world1 = child.worldTransform();

    // Second call should return cached result (same reference)
    Transform const& world2 = child.worldTransform();

    // Should be the same object in memory
    EXPECT_EQ(&world1, &world2);
    EXPECT_TRUE(world1.isApprox(world2));
}

// *********************************************************************************
//! \brief Test multiple children with same name.
// *********************************************************************************
TEST_F(SceneNodeTest, MultipleChildrenWithSameName)
{
    // Test multiple children with same name (should return first found)
    Node const& child1 = root->createChild<Node>("duplicate");
    Node const& child2 = root->createChild<Node>("duplicate");

    // Test that there are two children
    EXPECT_EQ(root->children().size(), 2);
    EXPECT_NE(&child1, &child2);

    // Should return the first one found
    Node const* found = Node::find(*root, "duplicate");
    EXPECT_TRUE(found == &child1 || found == &child2);
}

// *********************************************************************************
//! \brief Test template createChild with derived types.
// *********************************************************************************
class DerivedNode: public Node
{
public:

    DerivedNode(const std::string& p_name, int p_value)
        : Node(p_name), m_value(p_value)
    {
    }

    int value() const
    {
        return m_value;
    }

    void value(int value)
    {
        m_value = value;
    }

private:

    int m_value;
};

// *********************************************************************************
//! \brief Test template createChild with derived types.
// *********************************************************************************
TEST_F(SceneNodeTest, TemplateCreateChildDerived)
{
    DerivedNode& derived = root->createChild<DerivedNode>("derived", 42);

    EXPECT_EQ(derived.name(), "derived");
    EXPECT_EQ(derived.value(), 42);
    EXPECT_EQ(Node::find(*root, "derived"), &derived);

    // Test that derived functionality works
    derived.value(100);
    EXPECT_EQ(derived.value(), 100);

    // Test that it's properly integrated in blueprint
    EXPECT_EQ(root->children().size(), 1);
    EXPECT_EQ(root->children()[0].get(), &derived);
}

// *********************************************************************************
//! \brief Test addChild with existing node (without automatic parent-child
//! relationship).
// *********************************************************************************
TEST_F(SceneNodeTest, AddExistingChildBasic)
{
    auto child_node = Node::create<Node>("external_child");
    Node* child_ptr = child_node.get();

    // Set a transform on the child before adding
    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(0, 3) = 5.0;
    child_node->localTransform(child_transform);

    // Verify the child's transform is preserved
    EXPECT_TRUE(child_ptr->localTransform().isApprox(child_transform));
    EXPECT_TRUE(child_ptr->worldTransform().isApprox(child_transform));

    // Add child to root
    Transform root_transform = Eigen::Matrix4d::Identity();
    root_transform(0, 3) = 10.0;
    root->localTransform(root_transform);
    root->addChild(std::move(child_node));

    // Verify the child is added to the root
    EXPECT_EQ(root->children().size(), 1);
    EXPECT_EQ(Node::find(*root, "external_child"), child_ptr);

    // Verify the child's transform is preserved
    Transform expected_transform = Eigen::Matrix4d::Identity();
    expected_transform(0, 3) = 15.0;
    EXPECT_TRUE(child_ptr->localTransform().isApprox(child_transform));
    EXPECT_TRUE(child_ptr->worldTransform().isApprox(expected_transform));

    // Change the root transform
    root_transform = Eigen::Matrix4d::Identity();
    root_transform(1, 3) = 3.0;
    root->localTransform(root_transform);

    // Check applied transforms
    EXPECT_TRUE(child_ptr->localTransform().isApprox(child_transform));
    expected_transform(0, 3) = 5.0;
    expected_transform(1, 3) = 3.0;
    EXPECT_TRUE(child_ptr->worldTransform().isApprox(expected_transform));
}

// *********************************************************************************
//! \brief Test recursive node search with complex blueprint.
// *********************************************************************************
TEST_F(SceneNodeTest, RecursiveNodeSearchComplex)
{
    Node& child1 = root->createChild<Node>("child1");
    Node& child2 = root->createChild<Node>("child2");
    Node& grandchild1 = child1.createChild<Node>("grandchild1");
    Node& grandchild2 = child1.createChild<Node>("grandchild2");
    Node& great_grandchild = grandchild1.createChild<Node>("great_grandchild");

    // Test that getNode searches recursively through the entire
    // subtree
    EXPECT_EQ(Node::find(*root, "child1"), &child1);
    EXPECT_EQ(Node::find(*root, "child2"), &child2);
    EXPECT_EQ(Node::find(child1, "grandchild1"), &grandchild1);
    EXPECT_EQ(Node::find(child1, "grandchild2"), &grandchild2);
    EXPECT_EQ(Node::find(grandchild1, "great_grandchild"), &great_grandchild);

    // Test search from child node
    EXPECT_EQ(Node::find(child1, "grandchild1"), &grandchild1);
    EXPECT_EQ(Node::find(child1, "grandchild2"), &grandchild2);
    EXPECT_EQ(Node::find(child1, "great_grandchild"), &great_grandchild);

    // Test that child can't find sibling
    EXPECT_EQ(Node::find(child1, "child2"), nullptr);

    // Test that grandchild can't find uncle
    EXPECT_EQ(Node::find(grandchild1, "child2"), nullptr);

    // Test search from grandchild
    EXPECT_EQ(Node::find(grandchild1, "great_grandchild"), &great_grandchild);
    EXPECT_EQ(Node::find(grandchild2, "great_grandchild"), nullptr);
}

// *********************************************************************************
//! \brief Test multiple addChild calls.
// *********************************************************************************
TEST_F(SceneNodeTest, MultipleAddChildCalls)
{
    auto child1 = Node::create<Node>("child1");
    auto child2 = Node::create<Node>("child2");
    auto child3 = Node::create<Node>("child3");

    Node* child1_ptr = child1.get();
    Node* child2_ptr = child2.get();
    Node* child3_ptr = child3.get();

    root->addChild(std::move(child1));
    root->addChild(std::move(child2));
    root->addChild(std::move(child3));

    EXPECT_EQ(root->children().size(), 3);
    EXPECT_EQ(Node::find(*root, "child1"), child1_ptr);
    EXPECT_EQ(Node::find(*root, "child2"), child2_ptr);
    EXPECT_EQ(Node::find(*root, "child3"), child3_ptr);
}

// *********************************************************************************
//! \brief Test mixing createChild and addChild.
// *********************************************************************************
TEST_F(SceneNodeTest, MixedChildCreation)
{
    // Create child using createChild
    Node& created_child = root->createChild<Node>("created");

    // Add child using addChild
    auto added_child = Node::create<Node>("added");
    Node* added_ptr = added_child.get();
    root->addChild(std::move(added_child));

    // Create another child using createChild
    Node& created_child2 = root->createChild<Node>("created2");

    EXPECT_EQ(root->children().size(), 3);
    EXPECT_EQ(Node::find(*root, "created"), &created_child);
    EXPECT_EQ(Node::find(*root, "added"), added_ptr);
    EXPECT_EQ(Node::find(*root, "created2"), &created_child2);
}

// *********************************************************************************
//! \brief Test difference between createChild and addChild behavior.
// *********************************************************************************
TEST_F(SceneNodeTest, CreateChildVsAddChildBehavior)
{
    // Set a parent transform
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(0, 3) = 10.0; // Translation in x
    root->localTransform(parent_transform);

    // Create child using createChild (should inherit parent transforms)
    Node& created_child = root->createChild<Node>("created");
    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(1, 3) = 5.0; // Translation in y
    created_child.localTransform(child_transform);

    // Add child using addChild (should NOT inherit parent transforms)
    auto added_child = Node::create<Node>("added");
    Node const* added_ptr = added_child.get();
    added_child->localTransform(child_transform); // Same local transform
    root->addChild(std::move(added_child));

    // Update world transforms
    // root->update();

    // Created child should have combined transform (parent + local)
    Transform expected_created_world = parent_transform * child_transform;
    EXPECT_TRUE(
        created_child.worldTransform().isApprox(expected_created_world));

    // Added child should have combined transform (parent + local)
    EXPECT_TRUE(added_ptr->worldTransform().isApprox(expected_created_world));
}