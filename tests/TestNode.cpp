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
        root_node = Node::create<Node>("root");
    }

    Node::Ptr root_node;
};

// Test Node creation and basic properties
TEST_F(NodeTest, Creation)
{
    EXPECT_EQ(root_node->getName(), "root");
    EXPECT_TRUE(root_node->getChildren().empty());
}

// Test adding children
TEST_F(NodeTest, CreateChild)
{
    Node& child1 = root_node->createChild<Node>("child1");
    Node& child2 = root_node->createChild<Node>("child2");

    EXPECT_EQ(root_node->getChildren().size(), 2);
    EXPECT_EQ(root_node->getNode("child1"), &child1);
    EXPECT_EQ(root_node->getNode("child2"), &child2);
    EXPECT_EQ(child1.getName(), "child1");
    EXPECT_EQ(child2.getName(), "child2");
}

// Test hierarchical structure
TEST_F(NodeTest, Hierarchy)
{
    Node& child1 = root_node->createChild<Node>("child1");
    Node& grandchild = child1.createChild<Node>("grandchild");

    EXPECT_EQ(root_node->getChildren().size(), 1);
    EXPECT_EQ(child1.getChildren().size(), 1);
    EXPECT_EQ(child1.getNode("grandchild"), &grandchild);
}

// Test local transformations
TEST_F(NodeTest, LocalTransform)
{
    Transform identity = Eigen::Matrix4d::Identity();

    // Test initial identity transform
    EXPECT_TRUE(root_node->getLocalTransform().isApprox(identity));

    // Test setting a new transform
    Transform test_transform = Eigen::Matrix4d::Identity();
    test_transform(0, 3) = 1.0; // Translation in x
    test_transform(1, 3) = 2.0; // Translation in y
    test_transform(2, 3) = 3.0; // Translation in z

    root_node->setLocalTransform(test_transform);
    EXPECT_TRUE(root_node->getLocalTransform().isApprox(test_transform));
}

// Test world transformations
TEST_F(NodeTest, WorldTransform)
{
    Transform parent_transform = Eigen::Matrix4d::Identity();
    parent_transform(0, 3) = 1.0; // Translation in x

    Transform child_transform = Eigen::Matrix4d::Identity();
    child_transform(1, 3) = 2.0; // Translation in y

    root_node->setLocalTransform(parent_transform);
    Node& child1 = root_node->createChild<Node>("child1");
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
    root_node->createChild<Node>("child1");

    Transform new_transform = Eigen::Matrix4d::Identity();
    new_transform(0, 3) = 5.0;

    root_node->setLocalTransform(new_transform);
    root_node->updateWorldTransform();

    EXPECT_TRUE(root_node->getWorldTransform().isApprox(new_transform));
}

// Test edge cases
TEST_F(NodeTest, EdgeCases)
{
    // Test getting non-existent node
    EXPECT_EQ(root_node->getNode("nonexistent"), nullptr);
}

// Test template createChild with derived types
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

TEST_F(NodeTest, TemplateCreateChild)
{
    DerivedNode& derived = root_node->createChild<DerivedNode>("derived", 42);

    EXPECT_EQ(derived.getName(), "derived");
    EXPECT_EQ(derived.getValue(), 42);
    EXPECT_EQ(root_node->getNode("derived"), &derived);
}

// Test addChild with existing node
TEST_F(NodeTest, AddExistingChild)
{
    auto child_node = Node::create<Node>("external_child");
    Node* child_ptr = child_node.get();

    root_node->addChild(std::move(child_node));

    EXPECT_EQ(root_node->getChildren().size(), 1);
    EXPECT_EQ(root_node->getNode("external_child"), child_ptr);
}

// Test recursive node search
TEST_F(NodeTest, RecursiveNodeSearch)
{
    Node& child1 = root_node->createChild<Node>("child1");
    Node& child2 = root_node->createChild<Node>("child2");
    Node& grandchild = child1.createChild<Node>("grandchild");
    Node& great_grandchild = grandchild.createChild<Node>("great_grandchild");

    // Test that getNode searches recursively through the entire subtree
    EXPECT_EQ(root_node->getNode("child1"), &child1);
    EXPECT_EQ(root_node->getNode("child2"), &child2);
    EXPECT_EQ(root_node->getNode("grandchild"), &grandchild);
    EXPECT_EQ(root_node->getNode("great_grandchild"), &great_grandchild);

    // Test search from child node
    EXPECT_EQ(child1.getNode("grandchild"), &grandchild);
    EXPECT_EQ(child1.getNode("great_grandchild"), &great_grandchild);

    // Test that child can't find sibling
    EXPECT_EQ(child1.getNode("child2"), nullptr);
}