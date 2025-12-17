#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

class NervousSystemNode : public rclcpp::Node
{
public:
    NervousSystemNode() : Node("nervous_system_node")
    {
        RCLCPP_INFO(this->get_logger(), "Nervous System Node Started");

        // Subscriber for sensor inputs
        sensor_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "sensor_input", 10,
            std::bind(&NervousSystemNode::sensorCallback, this, std::placeholders::_1));

        // Publisher for motor commands
        motor_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "motor_command", 10);

        // Timer for reflex processing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&NervousSystemNode::processReflexes, this));
    }

private:
    void sensorCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received sensor data");
        // Process incoming sensor data
        last_sensor_data_ = *msg;
    }

    void processReflexes()
    {
        // Simulate neural processing and reflex responses
        if (!last_sensor_data_.name.empty()) {
            auto command_msg = geometry_msgs::msg::Twist();
            // Simple reflex response logic
            command_msg.linear.x = 0.5; // Example response
            motor_pub_->publish(command_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensor_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState last_sensor_data_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NervousSystemNode>());
    rclcpp::shutdown();
    return 0;
}