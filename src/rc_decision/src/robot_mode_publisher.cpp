#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include <iostream>

class RobotModePublisher : public rclcpp::Node
{
public:
    RobotModePublisher() : Node("robot_mode_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("robot_mode", 10);
    }

    void publish_robot_mode(int64_t robot_mode)
    {
        std_msgs::msg::Int64 msg;
        msg.data = robot_mode;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto robot_mode_publisher = std::make_shared<RobotModePublisher>();

    int64_t robot_mode;
    std::cout << "请输入机器人模式: ";
    while (std::cin >> robot_mode)
    {
        robot_mode_publisher->publish_robot_mode(robot_mode);
        std::cout << "请输入机器人模式: ";
    }

    rclcpp::shutdown();
    return 0;
}