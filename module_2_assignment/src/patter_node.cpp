// Purpose:
// - Control a robot by sending velocity commands at regular intervals
// - Demonstrate the use of ROS2 publishers within a class
// Author: 
// ros2 run module_2_assignment pattern --ros-args -p radius:=2.0

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class RobotDriver : public rclcpp::Node {
public:
    // Constructor: sets up the robot driver node
    RobotDriver() : Node("patter_node") {
        this->declare_parameter<std::string>("cmd_vel_topic", "/turtle1/cmd_vel");
        this->declare_parameter<double>("radius", 0.4);
        this->declare_parameter<std::string>("pattern", "circle");

        auto cmdVelTopic = this->get_parameter("cmd_vel_topic").as_string();
        radius_ = this->get_parameter("radius").as_double();
        pattern_ = this->get_parameter("pattern").as_string();

        _publisher = this->create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic, 10);
        _timer = this->create_wall_timer(500ms, std::bind(&RobotDriver::timerCallback, this));
        
    }

private:
    // Timer callback: sends velocity commands
    void timerCallback() {

        double radius = radius_;
        
        if (pattern_ == "spiral") {
            theta_ += 0.1;
            radius = radius_ * std::exp(0.1 * theta_);
        }

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = radius; // Set linear velocity
        message.angular.z = 1.0 / radius; // Set angular velocity
        RCLCPP_INFO(this->get_logger(), "Pattern Node");
        _publisher->publish(message);
    }

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    double radius_;
    double theta_; 
    std::string pattern_;
    
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotDriver>());
    rclcpp::shutdown();
    return 0;
}
