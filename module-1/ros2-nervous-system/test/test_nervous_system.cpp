#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

// Simple test to verify the ROS2 package structure
TEST(NervousSystemTest, InitializationTest) {
    // Initialize ROS2
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    EXPECT_TRUE(rclcpp::ok());

    // Shutdown ROS2
    rclcpp::shutdown();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}