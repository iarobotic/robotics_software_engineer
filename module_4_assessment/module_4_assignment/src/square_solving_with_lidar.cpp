#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

enum class RobotState
{
    MOVING_STRAIGHT,
    TURNING_LEFT,
    TURNING_RIGHT,
    OUT_OF_MAZE
};

class MazeSolvingWithIMU : public rclcpp::Node
{
public:
    MazeSolvingWithIMU() : Node("maze_solving_imu"), _state(RobotState::MOVING_STRAIGHT)
    {
        _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        _laser_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&MazeSolvingWithIMU::lidarCallback, this, std::placeholders::_1));
            
        _imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&MazeSolvingWithIMU::imuCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
            "Linear Acceleration: x=%.2f, y=%.2f, z=%.2f",
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
            
        RCLCPP_INFO(this->get_logger(),
            "Angular Velocity: x=%.2f, y=%.2f, z=%.2f",
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);
            
        RCLCPP_INFO(this->get_logger(),
            "Orientation (Quaternion): x=%.2f, y=%.2f, z=%.2f, w=%.2f",
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidarMsg)
    {
        float rightObstacle = *std::min_element(lidarMsg->ranges.begin() + 260,
                                                lidarMsg->ranges.begin() + 280);
        float frontObstacle = *std::min_element(lidarMsg->ranges.begin() + 340,
                                                lidarMsg->ranges.begin() + 360);

        RCLCPP_INFO(this->get_logger(), "Front: %f, Right: %f", frontObstacle, rightObstacle);

        if (frontObstacle < _frontThreshold)
        {
            _state = RobotState::TURNING_LEFT;
        }
        else if (rightObstacle > _sideThreshold + 0.2)
        {
            _state = RobotState::TURNING_RIGHT;
        }
        else if (rightObstacle < _sideThreshold - 0.2)
        {
            _state = RobotState::TURNING_LEFT;
        }
        else
        {
            _state = RobotState::MOVING_STRAIGHT;
        }

        geometry_msgs::msg::Twist command;
        switch (_state)
        {
        case RobotState::MOVING_STRAIGHT:
            command.linear.x = _linearVel;
            command.angular.z = 0.0;
            break;
        case RobotState::TURNING_LEFT:
            command.linear.x = 0.1;
            command.angular.z = _angularVel;
            break;
        case RobotState::TURNING_RIGHT:
            command.linear.x = 0.1;
            command.angular.z = -_angularVel;
            break;
        case RobotState::OUT_OF_MAZE:
            command.linear.x = 0.0;
            command.angular.z = 0.0;
            break;
        }

        _publisher->publish(command);
    }

    float _frontThreshold = 1.0f;
    float _sideThreshold = 0.5f;
    float _angularVel = 0.3f;
    float _linearVel = 0.4f;
    RobotState _state;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_subscription;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeSolvingWithIMU>());
    rclcpp::shutdown();
    return 0;
}