#include "Robotik/Robotik.hpp"

#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for Node class.
// *********************************************************************************
class NodeTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // Setup test data
        root_node = Node::create<Node>("root");
    }

    Node::Ptr root_node;
};

// *********************************************************************************
//! \brief Test Node creation and basic properties.
// *********************************************************************************
TEST_F(NodeTest, Creation)
{
    EXPECT_EQ(root_node->getName(), "root");
    EXPECT_TRUE(root_node->getChildren().empty());
    EXPECT_TRUE(
        root_node->getLocalTransform().isApprox(Eigen::Matrix4d::Identity()));
    EXPECT_TRUE(
        root_node->getWorldTransform().isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test getName() method.
// *********************************************************************************
TEST_F(NodeTest, GetName)
{
    EXPECT_EQ(root_node->getName(), "root");

    Node& child = root_node->createChild<Node>("test_child");
    EXPECT_EQ(child.getName(), "test_child");

    // Test with empty name
    Node& empty_child = root_node->createChild<Node>("");
    EXPECT_EQ(empty_child.getName(), "");
}

// *********************************************************************************
//! \brief Test adding children and parent-child relationships.
// *********************************************************************************
TEST_F(NodeTest, CreateChildAndParenthood)
{
    Node& child1 = root_node->createChild<Node>("child1");
    Node& child2 = root_node->createChild<Node>("child2");

    EXPECT_EQ(root_node->getChildren().size(), 2);
    EXPECT_EQ(root_node->getNode("child1"), &child1);
    EXPECT_EQ(root_node->getNode("child2"), &child2);
    EXPECT_EQ(child1.getName(), "child1");
    EXPECT_EQ(child2.getName(), "child2");

    // Verify parent-child relationships through world transform updates
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(0, 3) = 5.0; // Translation in x
    root_node->setLocalTransform(parent_transform);

    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(1, 3) = 3.0; // Translation in y
    child1.setLocalTransform(child_transform);

    // Update world transforms
    root_node->updateWorldTransform();

    // Child should have combined transform
    Transform expected_world = parent_transform * child_transform;
    EXPECT_TRUE(child1.getWorldTransform().isApprox(expected_world));
}

// *********************************************************************************
//! \brief Test hierarchical structure with multiple levels.
// *********************************************************************************
TEST_F(NodeTest, ComplexHierarchy)
{
    Node& child1 = root_node->createChild<Node>("child1");
    Node& child2 = root_node->createChild<Node>("child2");
    Node& grandchild1 = child1.createChild<Node>("grandchild1");
    Node& grandchild2 = child1.createChild<Node>("grandchild2");
    Node& great_grandchild = grandchild1.createChild<Node>("great_grandchild");

    EXPECT_EQ(root_node->getChildren().size(), 2);
    EXPECT_EQ(child1.getChildren().size(), 2);
    EXPECT_EQ(child2.getChildren().size(), 0);
    EXPECT_EQ(grandchild1.getChildren().size(), 1);
    EXPECT_EQ(grandchild2.getChildren().size(), 0);
    EXPECT_EQ(great_grandchild.getChildren().size(), 0);
}

// *********************************************************************************
//! \brief Test complex transformation propagation through hierarchy.
// *********************************************************************************
TEST_F(NodeTest, ComplexTransformationPropagation)
{
    // Create a 3-level hierarchy
    Node& level1 = root_node->createChild<Node>("level1");
    Node& level2 = level1.createChild<Node>("level2");
    Node& level3 = level2.createChild<Node>("level3");

    // Set transforms for each level
    Transform root_transform = Eigen::Matrix4d::Identity();
    root_transform(0, 3) = 1.0; // Translation in x
    root_node->setLocalTransform(root_transform);

    Transform level1_transform = Eigen::Matrix4d::Identity();
    level1_transform(1, 3) = 2.0; // Translation in y
    level1.setLocalTransform(level1_transform);

    Transform level2_transform = Eigen::Matrix4d::Identity();
    level2_transform(2, 3) = 3.0; // Translation in z
    level2.setLocalTransform(level2_transform);

    Transform level3_transform = Eigen::Matrix4d::Identity();
    level3_transform(0, 3) = 4.0; // Another translation in x
    level3.setLocalTransform(level3_transform);

    // Update world transforms
    root_node->updateWorldTransform();

    // Verify cumulative transforms
    Transform expected_level1 = root_transform * level1_transform;
    Transform expected_level2 = expected_level1 * level2_transform;
    Transform expected_level3 = expected_level2 * level3_transform;

    EXPECT_TRUE(level1.getWorldTransform().isApprox(expected_level1));
    EXPECT_TRUE(level2.getWorldTransform().isApprox(expected_level2));
    EXPECT_TRUE(level3.getWorldTransform().isApprox(expected_level3));
}

// *********************************************************************************
//! \brief Test local transformations with rotations.
// *********************************************************************************
TEST_F(NodeTest, LocalTransformWithRotation)
{
    Transform identity = Eigen::Matrix4d::Identity();
    EXPECT_TRUE(root_node->getLocalTransform().isApprox(identity));

    // Create a rotation around Z axis (90 degrees)
    Transform rotation_transform = Eigen::Matrix4d::Identity();
    double angle = M_PI / 2.0;
    rotation_transform(0, 0) = cos(angle);
    rotation_transform(0, 1) = -sin(angle);
    rotation_transform(1, 0) = sin(angle);
    rotation_transform(1, 1) = cos(angle);

    // Add translation
    rotation_transform(0, 3) = 1.0;
    rotation_transform(1, 3) = 2.0;
    rotation_transform(2, 3) = 3.0;

    root_node->setLocalTransform(rotation_transform);
    EXPECT_TRUE(root_node->getLocalTransform().isApprox(rotation_transform));
}

// *********************************************************************************
//! \brief Test world transformations with multiple children.
// *********************************************************************************
TEST_F(NodeTest, WorldTransformMultipleChildren)
{
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(0, 3) = 1.0; // Translation in x
    root_node->setLocalTransform(parent_transform);

    Node& child1 = root_node->createChild<Node>("child1");
    Node& child2 = root_node->createChild<Node>("child2");

    Transform child1_transform = Eigen::Matrix4d::Identity();
    child1_transform(1, 3) = 2.0; // Translation in y
    child1.setLocalTransform(child1_transform);

    Transform child2_transform = Eigen::Matrix4d::Identity();
    child2_transform(2, 3) = 3.0; // Translation in z
    child2.setLocalTransform(child2_transform);

    // Update world transforms
    root_node->updateWorldTransform();

    // Both children should have parent transform applied
    Transform expected_child1 = parent_transform * child1_transform;
    Transform expected_child2 = parent_transform * child2_transform;

    EXPECT_TRUE(child1.getWorldTransform().isApprox(expected_child1));
    EXPECT_TRUE(child2.getWorldTransform().isApprox(expected_child2));
}

// *********************************************************************************
//! \brief Test dirty flag mechanism comprehensively.
// *********************************************************************************
TEST_F(NodeTest, DirtyFlagMechanism)
{
    Node& child1 = root_node->createChild<Node>("child1");
    Node& grandchild = child1.createChild<Node>("grandchild");

    // Set initial transforms
    Transform root_transform = Eigen::Matrix4d::Identity();
    root_transform(0, 3) = 1.0;
    root_node->setLocalTransform(root_transform);

    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(1, 3) = 2.0;
    child1.setLocalTransform(child_transform);

    // Update world transforms - this should clear dirty flags
    root_node->updateWorldTransform();

    // Store initial world transforms
    Transform initial_child_world = child1.getWorldTransform();
    Transform initial_grandchild_world = grandchild.getWorldTransform();

    // Change root transform - should mark all descendants as dirty
    root_transform(0, 3) = 5.0;
    root_node->setLocalTransform(root_transform);

    // Update world transforms again
    root_node->updateWorldTransform();

    // World transforms should be different now
    EXPECT_FALSE(child1.getWorldTransform().isApprox(initial_child_world));
    EXPECT_FALSE(
        grandchild.getWorldTransform().isApprox(initial_grandchild_world));
}

// *********************************************************************************
//! \brief Test performance of world transform caching.
// *********************************************************************************
TEST_F(NodeTest, WorldTransformCaching)
{
    Node& child = root_node->createChild<Node>("child");

    Transform test_transform = Eigen::Matrix4d::Identity();
    test_transform(0, 3) = 1.0;
    root_node->setLocalTransform(test_transform);

    // First call should compute world transform
    const Transform& world1 = child.getWorldTransform();

    // Second call should return cached result (same reference)
    const Transform& world2 = child.getWorldTransform();

    // Should be the same object in memory
    EXPECT_EQ(&world1, &world2);
    EXPECT_TRUE(world1.isApprox(world2));
}

// *********************************************************************************
//! \brief Test edge cases and error conditions.
// *********************************************************************************
TEST_F(NodeTest, EdgeCases)
{
    // Test getting non-existent node
    EXPECT_EQ(root_node->getNode("nonexistent"), nullptr);

    // Test with empty name search
    EXPECT_EQ(root_node->getNode(""), nullptr);

    // Create child with empty name and try to find it
    Node& empty_child = root_node->createChild<Node>("");
    EXPECT_EQ(root_node->getNode(""), &empty_child);

    // Test searching in empty hierarchy
    auto empty_node = Node::create<Node>("empty");
    EXPECT_EQ(empty_node->getNode("anything"), nullptr);

    // Test multiple children with same name (should return first found)
    Node& child1 = root_node->createChild<Node>("duplicate");
    Node& child2 = root_node->createChild<Node>("duplicate");

    // Should return the first one found
    Node* found = root_node->getNode("duplicate");
    EXPECT_TRUE(found == &child1 || found == &child2);
}

// *********************************************************************************
//! \brief Test template createChild with derived types.
// *********************************************************************************
class DerivedNode: public Node
{
public:

    DerivedNode(const std::string& name, int value) : Node(name), m_value(value)
    {
    }

    int getValue() const
    {
        return m_value;
    }

    void setValue(int value)
    {
        m_value = value;
    }

private:

    int m_value;
};

// *********************************************************************************
//! \brief Test template createChild with derived types.
// *********************************************************************************
TEST_F(NodeTest, TemplateCreateChildDerived)
{
    DerivedNode& derived = root_node->createChild<DerivedNode>("derived", 42);

    EXPECT_EQ(derived.getName(), "derived");
    EXPECT_EQ(derived.getValue(), 42);
    EXPECT_EQ(root_node->getNode("derived"), &derived);

    // Test that derived functionality works
    derived.setValue(100);
    EXPECT_EQ(derived.getValue(), 100);

    // Test that it's properly integrated in hierarchy
    EXPECT_EQ(root_node->getChildren().size(), 1);
    EXPECT_EQ(root_node->getChildren()[0].get(), &derived);
}

// *********************************************************************************
//! \brief Test addChild with existing node (without automatic parent-child
//! relationship).
// *********************************************************************************
TEST_F(NodeTest, AddExistingChildBasic)
{
    auto child_node = Node::create<Node>("external_child");
    Node* child_ptr = child_node.get();

    // Set a transform on the child before adding
    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(0, 3) = 5.0;
    child_node->setLocalTransform(child_transform);

    root_node->addChild(std::move(child_node));

    EXPECT_EQ(root_node->getChildren().size(), 1);
    EXPECT_EQ(root_node->getNode("external_child"), child_ptr);

    // Verify the child's transform is preserved
    EXPECT_TRUE(child_ptr->getLocalTransform().isApprox(child_transform));

    // NOTE: addChild() doesn't automatically set parent-child relationship
    // so the child's world transform will be its local transform only
    EXPECT_TRUE(child_ptr->getWorldTransform().isApprox(child_transform));

    // Set parent transform - this won't affect the child's world transform
    // because addChild() doesn't set the parent relationship
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(1, 3) = 3.0;
    root_node->setLocalTransform(parent_transform);

    root_node->updateWorldTransform();

    // Child's world transform should still be just its local transform
    EXPECT_TRUE(child_ptr->getWorldTransform().isApprox(child_transform));
}

// *********************************************************************************
//! \brief Test recursive node search with complex hierarchy.
// *********************************************************************************
TEST_F(NodeTest, RecursiveNodeSearchComplex)
{
    Node& child1 = root_node->createChild<Node>("child1");
    Node& child2 = root_node->createChild<Node>("child2");
    Node& grandchild1 = child1.createChild<Node>("grandchild1");
    Node& grandchild2 = child1.createChild<Node>("grandchild2");
    Node& great_grandchild = grandchild1.createChild<Node>("great_grandchild");

    // Test that getNode searches recursively through the entire subtree
    EXPECT_EQ(root_node->getNode("child1"), &child1);
    EXPECT_EQ(root_node->getNode("child2"), &child2);
    EXPECT_EQ(root_node->getNode("grandchild1"), &grandchild1);
    EXPECT_EQ(root_node->getNode("grandchild2"), &grandchild2);
    EXPECT_EQ(root_node->getNode("great_grandchild"), &great_grandchild);

    // Test search from child node
    EXPECT_EQ(child1.getNode("grandchild1"), &grandchild1);
    EXPECT_EQ(child1.getNode("grandchild2"), &grandchild2);
    EXPECT_EQ(child1.getNode("great_grandchild"), &great_grandchild);

    // Test that child can't find sibling
    EXPECT_EQ(child1.getNode("child2"), nullptr);

    // Test that grandchild can't find uncle
    EXPECT_EQ(grandchild1.getNode("child2"), nullptr);

    // Test search from grandchild
    EXPECT_EQ(grandchild1.getNode("great_grandchild"), &great_grandchild);
    EXPECT_EQ(grandchild2.getNode("great_grandchild"), nullptr);
}

// *********************************************************************************
//! \brief Test multiple addChild calls.
// *********************************************************************************
TEST_F(NodeTest, MultipleAddChildCalls)
{
    auto child1 = Node::create<Node>("child1");
    auto child2 = Node::create<Node>("child2");
    auto child3 = Node::create<Node>("child3");

    Node* child1_ptr = child1.get();
    Node* child2_ptr = child2.get();
    Node* child3_ptr = child3.get();

    root_node->addChild(std::move(child1));
    root_node->addChild(std::move(child2));
    root_node->addChild(std::move(child3));

    EXPECT_EQ(root_node->getChildren().size(), 3);
    EXPECT_EQ(root_node->getNode("child1"), child1_ptr);
    EXPECT_EQ(root_node->getNode("child2"), child2_ptr);
    EXPECT_EQ(root_node->getNode("child3"), child3_ptr);
}

// *********************************************************************************
//! \brief Test mixing createChild and addChild.
// *********************************************************************************
TEST_F(NodeTest, MixedChildCreation)
{
    // Create child using createChild
    Node& created_child = root_node->createChild<Node>("created");

    // Add child using addChild
    auto added_child = Node::create<Node>("added");
    Node* added_ptr = added_child.get();
    root_node->addChild(std::move(added_child));

    // Create another child using createChild
    Node& created_child2 = root_node->createChild<Node>("created2");

    EXPECT_EQ(root_node->getChildren().size(), 3);
    EXPECT_EQ(root_node->getNode("created"), &created_child);
    EXPECT_EQ(root_node->getNode("added"), added_ptr);
    EXPECT_EQ(root_node->getNode("created2"), &created_child2);
}

// *********************************************************************************
//! \brief Test difference between createChild and addChild behavior.
// *********************************************************************************
TEST_F(NodeTest, CreateChildVsAddChildBehavior)
{
    // Set a parent transform
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(0, 3) = 10.0; // Translation in x
    root_node->setLocalTransform(parent_transform);

    // Create child using createChild (should inherit parent transforms)
    Node& created_child = root_node->createChild<Node>("created");
    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(1, 3) = 5.0; // Translation in y
    created_child.setLocalTransform(child_transform);

    // Add child using addChild (should NOT inherit parent transforms)
    auto added_child = Node::create<Node>("added");
    Node* added_ptr = added_child.get();
    added_child->setLocalTransform(child_transform); // Same local transform
    root_node->addChild(std::move(added_child));

    // Update world transforms
    root_node->updateWorldTransform();

    // Created child should have combined transform (parent + local)
    Transform expected_created_world = parent_transform * child_transform;
    EXPECT_TRUE(
        created_child.getWorldTransform().isApprox(expected_created_world));

    // Added child should have only its local transform (no parent relationship)
    EXPECT_TRUE(added_ptr->getWorldTransform().isApprox(child_transform));
}