#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include <iostream>

class PositionIdPublisher : public rclcpp::Node
{
public:
    PositionIdPublisher() : Node("position_id_pub")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("position_id", 10);
    }

    void publish_goal_id(int64_t goal_id)
    {
        std_msgs::msg::Int64 msg;
        msg.data = goal_id;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto position_id_pub = std::make_shared<PositionIdPublisher>();

    int64_t goal_id;
    std::cout << "请输入目标点ID: ";
    while (std::cin >> goal_id)
    {
        position_id_pub->publish_goal_id(goal_id);
        std::cout << "请输入目标点ID: ";
    }

    rclcpp::shutdown();
    return 0;
}