// Purpose:
// - Control a robot by sending velocity commands at regular intervals
// - Demonstrate the use of ROS2 publishers within a class
// Author: Robotisim

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node {
public:
    // Constructor: sets up the robot driver node
    RobotDriver() : Node("move_turtle"), _count(0) {
        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
        this->declare_parameter<double>("velocity", 1.0);

        auto cmdVelTopic = this->get_parameter("cmd_vel_topic").as_string();
        velocity_ = this->get_parameter("velocity").as_double();

        _publisher = this->create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic, 10);
        _timer = this->create_wall_timer(2000ms, std::bind(&RobotDriver::timerCallback, this));
    }

private:
    // Timer callback: sends velocity commands
    void timerCallback() {
        auto message = geometry_msgs::msg::Twist();
        velocity_ = this->get_parameter("velocity").as_double();
        
        message.linear.x = velocity_; // Set linear velocity
        message.angular.z = 0.0; // Set angular velocity
        RCLCPP_INFO(this->get_logger(), "Front");
        _publisher->publish(message);
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        message.linear.x = -velocity_; // Set linear velocity
        message.angular.z = 0.0; // Set angular velocity
        RCLCPP_INFO(this->get_logger(), "Back");
        _publisher->publish(message);
        
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    size_t _count;
    double velocity_;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDriver>());
    rclcpp::shutdown();
    return 0;
}
