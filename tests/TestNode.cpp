#include "Robotik/Robotik.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace robotik;

class NodeTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // Setup test data
        root_node = std::make_unique<Node>("root");
    }

    std::unique_ptr<Node> root_node;
};

// Test Node creation and basic properties
TEST_F(NodeTest, Creation)
{
    EXPECT_EQ(root_node->getName(), "root");
    EXPECT_TRUE(root_node->getChildren().empty());
}

// Test adding children
TEST_F(NodeTest, AddChild)
{
    Node& child1 = root_node->addChild<Node>("child1");
    Node& child2 = root_node->addChild<Node>("child2");

    EXPECT_EQ(root_node->getChildren().size(), 2);
    EXPECT_EQ(root_node->getChild("child1"), &child1);
    EXPECT_EQ(root_node->getChild("child2"), &child2);
    EXPECT_EQ(child1.getName(), "child1");
    EXPECT_EQ(child2.getName(), "child2");
}

// Test removing children
TEST_F(NodeTest, RemoveChild)
{
    Node& child1 = root_node->addChild<Node>("child1");
    Node& child2 = root_node->addChild<Node>("child2");

    root_node->removeChild("child1");

    EXPECT_EQ(root_node->getChildren().size(), 1);
    EXPECT_EQ(root_node->getChild("child2"), &child2);
    EXPECT_EQ(root_node->getChild("child1"), nullptr);
}

// Test hierarchical structure
TEST_F(NodeTest, Hierarchy)
{
    Node& child1 = root_node->addChild<Node>("child1");
    Node& grandchild = child1.addChild<Node>("grandchild");

    EXPECT_EQ(root_node->getChildren().size(), 1);
    EXPECT_EQ(child1.getChildren().size(), 1);
    EXPECT_EQ(child1.getChild("grandchild"), &grandchild);
}

// Test local transformations
TEST_F(NodeTest, LocalTransform)
{
    Transform identity = Transform::Identity();

    // Test initial identity transform
    EXPECT_TRUE(root_node->getLocalTransform().isApprox(identity));

    // Test setting a new transform
    Transform test_transform = Transform::Identity();
    test_transform(0, 3) = 1.0; // Translation in x
    test_transform(1, 3) = 2.0; // Translation in y
    test_transform(2, 3) = 3.0; // Translation in z

    root_node->setLocalTransform(test_transform);
    EXPECT_TRUE(root_node->getLocalTransform().isApprox(test_transform));
}

// Test world transformations
TEST_F(NodeTest, WorldTransform)
{
    Transform parent_transform = Transform::Identity();
    parent_transform(0, 3) = 1.0; // Translation in x

    Transform child_transform = Transform::Identity();
    child_transform(1, 3) = 2.0; // Translation in y

    root_node->setLocalTransform(parent_transform);
    Node& child1 = root_node->addChild<Node>("child1");
    child1.setLocalTransform(child_transform);

    // Update world transforms
    root_node->updateWorldTransform();

    // Check that world transform is composition of parent and local
    Transform expected_world = parent_transform * child_transform;
    EXPECT_TRUE(child1.getWorldTransform().isApprox(expected_world));
}

// Test dirty flag mechanism
TEST_F(NodeTest, DirtyFlag)
{
    Node& child1 = root_node->addChild<Node>("child1");

    // Mark dirty and check that transforms are updated
    root_node->markDirty();

    Transform new_transform = Transform::Identity();
    new_transform(0, 3) = 5.0;

    root_node->setLocalTransform(new_transform);
    root_node->updateWorldTransform();

    EXPECT_TRUE(root_node->getWorldTransform().isApprox(new_transform));
}

// Test edge cases
TEST_F(NodeTest, EdgeCases)
{
    // Test getting non-existent child
    EXPECT_EQ(root_node->getChild("nonexistent"), nullptr);

    // Test removing non-existent child (should not crash)
    EXPECT_NO_THROW(root_node->removeChild("nonexistent"));
}

// Test template addChild with derived types
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

private:

    int m_value;
};

TEST_F(NodeTest, TemplateAddChild)
{
    DerivedNode& derived = root_node->addChild<DerivedNode>("derived", 42);

    EXPECT_EQ(derived.getName(), "derived");
    EXPECT_EQ(derived.getValue(), 42);
    EXPECT_EQ(root_node->getChild("derived"), &derived);
}