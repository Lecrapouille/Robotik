/**
 * @file TestRobotLoaderFactory.cpp
 * @brief Unit tests for RobotLoaderFactory - Verification of factory pattern
 * for creating robot loaders based on file extensions, extension extraction,
 * and loader registration.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Loaders/RobotLoaderFactory.hpp"
#include "Robotik/Core/Loaders/UrdfLoader.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

using namespace robotik;

// *********************************************************************************
//! \brief Mock loader for testing registration functionality.
// *********************************************************************************
class MockLoader: public RobotLoader
{
public:

    MOCK_METHOD(std::unique_ptr<Robot>, load, (const std::string&), (override));
    MOCK_METHOD(std::string, error, (), (const, override));
};

// *********************************************************************************
//! \brief Test fixture for RobotLoaderFactory class.
// *********************************************************************************
class RobotLoaderFactoryTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // Save original registry state by noting that .urdf should exist
        has_urdf_loader = true;
    }

    void TearDown() override
    {
        // Note: Factory uses static registry, so tests may affect each other
        // In a real scenario, you might want to reset the registry
    }

    bool has_urdf_loader;
};

// *********************************************************************************
//! \brief Test creating loader for supported .urdf extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateURDFLoader)
{
    auto loader = RobotLoaderFactory::create("robot.urdf");
    ASSERT_NE(loader, nullptr)
        << "Should create URDF loader for .urdf extension";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr)
        << "Should return URDFLoader instance";
}

// *********************************************************************************
//! \brief Test creating loader with uppercase extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateURDFLoaderUppercase)
{
    auto loader = RobotLoaderFactory::create("robot.URDF");
    ASSERT_NE(loader, nullptr) << "Should handle uppercase extension";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test creating loader with mixed case extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateURDFLoaderMixedCase)
{
    auto loader = RobotLoaderFactory::create("robot.UrDf");
    ASSERT_NE(loader, nullptr) << "Should handle mixed case extension";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test creating loader with full path.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateURDFLoaderWithPath)
{
    auto loader = RobotLoaderFactory::create("/path/to/robot.urdf");
    ASSERT_NE(loader, nullptr) << "Should extract extension from full path";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test creating loader with relative path.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateURDFLoaderWithRelativePath)
{
    auto loader = RobotLoaderFactory::create("data/robots/myrobot.urdf");
    ASSERT_NE(loader, nullptr) << "Should extract extension from relative path";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test creating loader for unsupported extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateUnsupportedExtension)
{
    auto loader = RobotLoaderFactory::create("robot.xyz");
    EXPECT_EQ(loader, nullptr)
        << "Should return nullptr for unsupported extension";
}

// *********************************************************************************
//! \brief Test creating loader for file without extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateNoExtension)
{
    auto loader = RobotLoaderFactory::create("robot");
    EXPECT_EQ(loader, nullptr)
        << "Should return nullptr for file without extension";
}

// *********************************************************************************
//! \brief Test creating loader for empty filename.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateEmptyFilename)
{
    auto loader = RobotLoaderFactory::create("");
    EXPECT_EQ(loader, nullptr) << "Should return nullptr for empty filename";
}

// *********************************************************************************
//! \brief Test creating loader for filename with multiple dots.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateFilenameWithMultipleDots)
{
    auto loader = RobotLoaderFactory::create("robot.backup.urdf");
    ASSERT_NE(loader, nullptr)
        << "Should use last extension for files with multiple dots";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test creating loader for filename starting with dot.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, CreateFilenameStartingWithDot)
{
    auto loader = RobotLoaderFactory::create(".urdf");
    ASSERT_NE(loader, nullptr) << "Should handle filename starting with dot";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test registering a custom loader.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, RegisterCustomLoader)
{
    RobotLoaderFactory::registerLoader(
        ".test",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    auto loader = RobotLoaderFactory::create("robot.test");
    ASSERT_NE(loader, nullptr)
        << "Should create custom loader for registered extension";
    EXPECT_NE(dynamic_cast<MockLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test registering loader with extension without dot.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, RegisterLoaderWithoutDot)
{
    RobotLoaderFactory::registerLoader(
        "sdf",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    auto loader = RobotLoaderFactory::create("robot.sdf");
    ASSERT_NE(loader, nullptr)
        << "Should automatically add dot to extension if missing";
    EXPECT_NE(dynamic_cast<MockLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test registering loader with uppercase extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, RegisterLoaderUppercase)
{
    RobotLoaderFactory::registerLoader(
        ".XML",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    // Should work with lowercase
    auto loader1 = RobotLoaderFactory::create("robot.xml");
    ASSERT_NE(loader1, nullptr) << "Should convert extension to lowercase";
    EXPECT_NE(dynamic_cast<MockLoader*>(loader1.get()), nullptr);

    // Should work with uppercase
    auto loader2 = RobotLoaderFactory::create("robot.XML");
    ASSERT_NE(loader2, nullptr) << "Should handle uppercase when querying";
    EXPECT_NE(dynamic_cast<MockLoader*>(loader2.get()), nullptr);
}

// *********************************************************************************
//! \brief Test overwriting existing loader registration.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, OverwriteExistingLoader)
{
    // Register first loader
    RobotLoaderFactory::registerLoader(
        ".custom",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    auto loader1 = RobotLoaderFactory::create("test.custom");
    ASSERT_NE(loader1, nullptr);
    EXPECT_NE(dynamic_cast<MockLoader*>(loader1.get()), nullptr);

    // Overwrite with new loader
    RobotLoaderFactory::registerLoader(
        ".custom",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    auto loader2 = RobotLoaderFactory::create("test.custom");
    ASSERT_NE(loader2, nullptr);
    // Both should be MockLoader instances
    EXPECT_NE(dynamic_cast<MockLoader*>(loader2.get()), nullptr);
}

// *********************************************************************************
//! \brief Test registering empty extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, RegisterEmptyExtension)
{
    // This should still work - empty extension will be registered as-is
    RobotLoaderFactory::registerLoader(
        "",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    // Files without extension should not match empty string extension
    // (implementation detail, but worth testing)
    auto loader = RobotLoaderFactory::create("file_without_extension");
    // This might return nullptr depending on implementation
    // The test documents the current behavior
}

// *********************************************************************************
//! \brief Test multiple custom loaders.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, MultipleCustomLoaders)
{
    RobotLoaderFactory::registerLoader(
        ".format1",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    RobotLoaderFactory::registerLoader(
        ".format2",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    auto loader1 = RobotLoaderFactory::create("file.format1");
    auto loader2 = RobotLoaderFactory::create("file.format2");

    ASSERT_NE(loader1, nullptr) << "Should create loader for .format1";
    ASSERT_NE(loader2, nullptr) << "Should create loader for .format2";
    EXPECT_NE(dynamic_cast<MockLoader*>(loader1.get()), nullptr);
    EXPECT_NE(dynamic_cast<MockLoader*>(loader2.get()), nullptr);
}

// *********************************************************************************
//! \brief Test edge case: filename with only extension.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, FilenameOnlyExtension)
{
    auto loader = RobotLoaderFactory::create(".urdf");
    ASSERT_NE(loader, nullptr)
        << "Should handle filename that is just the extension";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test edge case: very long filename.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, VeryLongFilename)
{
    std::string long_path =
        "/very/long/path/" + std::string(100, 'a') + ".urdf";
    auto loader = RobotLoaderFactory::create(long_path);
    ASSERT_NE(loader, nullptr) << "Should handle very long filenames";
    EXPECT_NE(dynamic_cast<URDFLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test edge case: extension with special characters.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, ExtensionWithSpecialCharacters)
{
    // Register a loader with unusual extension
    RobotLoaderFactory::registerLoader(
        ".ext-x",
        []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<MockLoader>(); });

    auto loader = RobotLoaderFactory::create("file.ext-x");
    ASSERT_NE(loader, nullptr)
        << "Should handle extensions with special characters";
    EXPECT_NE(dynamic_cast<MockLoader*>(loader.get()), nullptr);
}

// *********************************************************************************
//! \brief Test that created loader is independent.
// *********************************************************************************
TEST_F(RobotLoaderFactoryTest, IndependentLoaderInstances)
{
    auto loader1 = RobotLoaderFactory::create("robot1.urdf");
    auto loader2 = RobotLoaderFactory::create("robot2.urdf");

    ASSERT_NE(loader1, nullptr);
    ASSERT_NE(loader2, nullptr);

    // They should be different instances
    EXPECT_NE(loader1.get(), loader2.get())
        << "Each call to create() should return a new loader instance";
}
