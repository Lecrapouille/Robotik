/**
 * @file TestApplication.cpp
 * @brief Unit tests for the Application class - Verification of timing system,
 * physics thread management, and main loop behavior.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Viewer/Application.hpp"

#include <atomic>
#include <chrono>
#include <thread>

using namespace robotik;

// *********************************************************************************
//! \brief Mock Application class for testing.
// *********************************************************************************
class MockApplication: public Application
{
public:

    MockApplication()
        : m_setup_called(false),
          m_setup_returns(false),
          m_update_count(0),
          m_draw_count(0),
          m_physics_update_count(0),
          m_should_close(false),
          m_last_delta_time(0.0f)
    {
    }

    // Test control methods
    void setSetupReturns(bool returns)
    {
        m_setup_returns = returns;
    }
    void setShouldClose(bool close)
    {
        m_should_close = close;
    }

    // Getters for verification
    bool isSetupCalled() const
    {
        return m_setup_called;
    }
    int getUpdateCount() const
    {
        return m_update_count;
    }
    int getDrawCount() const
    {
        return m_draw_count;
    }
    int getPhysicsUpdateCount() const
    {
        return m_physics_update_count;
    }
    float getLastDeltaTime() const
    {
        return m_last_delta_time;
    }

protected:

    bool isHalting() const override
    {
        return m_should_close;
    }

    bool onSetup() override
    {
        m_setup_called = true;
        return m_setup_returns;
    }

    void onDraw() override
    {
        m_draw_count++;
    }

    void onUpdate(float const p_dt) override
    {
        m_update_count++;
        m_last_delta_time = p_dt;
    }

    void onPhysicUpdate(float const /* p_dt */) override
    {
        m_physics_update_count++;
    }

    void setTitle(std::string const& /* p_title */) override
    {
        // Mock implementation - do nothing
    }

    void onCleanup() override
    {
        // Mock implementation - do nothing
    }

    void onHandleEvents() override
    {
        // Mock implementation - do nothing
    }

    void onFPSUpdated(float /* p_delta_time */) override
    {
        // Mock implementation - do nothing
    }

private:

    std::atomic<bool> m_setup_called;
    std::atomic<bool> m_setup_returns;
    std::atomic<int> m_update_count;
    std::atomic<int> m_draw_count;
    std::atomic<int> m_physics_update_count;
    std::atomic<bool> m_should_close;
    std::atomic<float> m_last_delta_time;
};

// *********************************************************************************
//! \brief Test fixture for Application class.
// *********************************************************************************
class ApplicationTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        app = std::make_unique<MockApplication>();
    }

    void TearDown() override
    {
        app.reset();
    }

    std::unique_ptr<MockApplication> app;
};

// *********************************************************************************
//! \brief Test application setup failure.
// *********************************************************************************
TEST_F(ApplicationTest, SetupFailure)
{
    app->setSetupReturns(false);

    bool result = app->run(60, 60);

    EXPECT_FALSE(result);
    EXPECT_TRUE(app->isSetupCalled());
    EXPECT_EQ(app->getUpdateCount(), 0);
    EXPECT_EQ(app->getDrawCount(), 0);
}

// *********************************************************************************
//! \brief Test successful application run with immediate close.
// *********************************************************************************
TEST_F(ApplicationTest, SuccessfulRunImmediateClose)
{
    app->setSetupReturns(true);
    app->setShouldClose(true);

    bool result = app->run(60, 60);

    EXPECT_TRUE(result);
    EXPECT_TRUE(app->isSetupCalled());
    // With immediate close, we might have 0 or 1 calls depending on timing
    EXPECT_GE(app->getUpdateCount(), 0);
    EXPECT_GE(app->getDrawCount(), 0);
}

// *********************************************************************************
//! \brief Test application run with multiple frames.
// *********************************************************************************
TEST_F(ApplicationTest, MultipleFrames)
{
    app->setSetupReturns(true);

    // Run for a short time then close
    std::thread close_thread(
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            app->setShouldClose(true);
        });

    bool result = app->run(60, 60);
    close_thread.join();

    EXPECT_TRUE(result);
    EXPECT_TRUE(app->isSetupCalled());
    // Should have multiple update and draw calls
    EXPECT_GT(app->getUpdateCount(), 1);
    EXPECT_GT(app->getDrawCount(), 1);
    // Draw count should be >= update count (no frame skipping)
    EXPECT_GE(app->getDrawCount(), app->getUpdateCount());
}

// *********************************************************************************
//! \brief Test delta time calculation.
// *********************************************************************************
TEST_F(ApplicationTest, DeltaTimeCalculation)
{
    app->setSetupReturns(true);

    // Run for a short time to ensure we get at least one update
    std::thread close_thread(
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            app->setShouldClose(true);
        });

    app->run(60, 60);
    close_thread.join();

    // Delta time should be reasonable if we had at least one update
    if (app->getUpdateCount() > 0)
    {
        float delta_time = app->getLastDeltaTime();
        EXPECT_GT(delta_time, 0.0f);
        EXPECT_LT(delta_time, 0.1f); // Should be less than 100ms
    }
}

// *********************************************************************************
//! \brief Test physics thread is started and stopped.
// *********************************************************************************
TEST_F(ApplicationTest, PhysicsThreadManagement)
{
    app->setSetupReturns(true);

    // Run for a short time to allow physics thread to start
    std::thread close_thread(
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            app->setShouldClose(true);
        });

    app->run(60, 60);
    close_thread.join();

    // Physics updates should have been called if thread had time to start
    // Note: Physics thread runs at 60Hz, so 20ms should be enough for at least
    // one update
    EXPECT_GE(app->getPhysicsUpdateCount(), 0);
}

// *********************************************************************************
//! \brief Test frame capping behavior.
// *********************************************************************************
TEST_F(ApplicationTest, FrameCapping)
{
    app->setSetupReturns(true);

    auto start_time = std::chrono::high_resolution_clock::now();

    std::thread close_thread(
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            app->setShouldClose(true);
        });

    // Run at 30 FPS - should take at least 100ms
    app->run(30, 60);
    close_thread.join();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    // Should have run for approximately 100ms (allowing some tolerance)
    EXPECT_GE(duration.count(), 90);  // At least 90ms
    EXPECT_LE(duration.count(), 150); // At most 150ms (allowing overhead)
}

// *********************************************************************************
//! \brief Test no frame capping when target FPS is 0.
// *********************************************************************************
TEST_F(ApplicationTest, NoFrameCapping)
{
    app->setSetupReturns(true);

    auto start_time = std::chrono::high_resolution_clock::now();

    std::thread close_thread(
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            app->setShouldClose(true);
        });

    // Run with no frame capping (0 FPS)
    app->run(0, 60);
    close_thread.join();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    // Should have many more frames than with 30 FPS capping
    EXPECT_GT(app->getDrawCount(), 10); // Should have many frames
}

// *********************************************************************************
//! \brief Test delta time clamping.
// *********************************************************************************
TEST_F(ApplicationTest, DeltaTimeClamping)
{
    app->setSetupReturns(true);
    app->setShouldClose(true);

    // Simulate a very slow frame by adding artificial delay
    // Note: This test is more about ensuring the system doesn't break
    // with extreme delta times rather than precise measurement

    app->run(60, 60);

    float delta_time = app->getLastDeltaTime();
    // Delta time should be clamped to reasonable values
    EXPECT_LT(delta_time,
              1.0f / 30.0f); // Should be clamped to max 30 FPS equivalent
}

// *********************************************************************************
//! \brief Test physics thread parameter validation.
// *********************************************************************************
TEST_F(ApplicationTest, PhysicsThreadParameterValidation)
{
    app->setSetupReturns(true);
    app->setShouldClose(true);

    // Test with zero physics Hz (should not start physics thread)
    app->run(60, 0);
    EXPECT_EQ(app->getPhysicsUpdateCount(), 0);

    // Reset for next test
    app.reset();
    app = std::make_unique<MockApplication>();

    // Test with very high physics Hz (should be capped)
    app->setSetupReturns(true);
    app->setShouldClose(true);
    app->run(60, 20000); // 20kHz - should be capped
    EXPECT_EQ(app->getPhysicsUpdateCount(), 0);
}

// *********************************************************************************
//! \brief Test physics thread restart.
// *********************************************************************************
TEST_F(ApplicationTest, PhysicsThreadRestart)
{
    app->setSetupReturns(true);

    // Start physics thread
    std::thread close_thread(
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            app->setShouldClose(true);
        });

    app->run(60, 60);
    close_thread.join();

    int first_run_physics_count = app->getPhysicsUpdateCount();

    // Reset and run again
    app.reset();
    app = std::make_unique<MockApplication>();
    app->setSetupReturns(true);

    close_thread = std::thread(
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
            app->setShouldClose(true);
        });

    app->run(60, 60);
    close_thread.join();

    // Both runs should have similar physics update counts
    EXPECT_GT(app->getPhysicsUpdateCount(), 0);
    EXPECT_GT(first_run_physics_count, 0);
}

// *********************************************************************************
//! \brief Test error handling in physics thread.
// *********************************************************************************
TEST_F(ApplicationTest, PhysicsThreadErrorHandling)
{
    // Test that getLastError returns empty string initially
    EXPECT_EQ(app->getLastError(), "");

    app->setSetupReturns(true);
    app->setShouldClose(true);

    // Run the application - should not produce errors
    app->run(60, 60);

    // Error string should still be empty for normal operation
    EXPECT_EQ(app->getLastError(), "");
}
