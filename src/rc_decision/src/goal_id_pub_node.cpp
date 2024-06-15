#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include <iostream>

class GoalIdPublisher : public rclcpp::Node
{
public:
    GoalIdPublisher() : Node("goal_id_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("goal_pose_id", 10);
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
    auto goal_id_publisher = std::make_shared<GoalIdPublisher>();

    int64_t goal_id;
    std::cout << "请输入目标点ID: ";
    while (std::cin >> goal_id)
    {
        goal_id_publisher->publish_goal_id(goal_id);
        std::cout << "请输入目标点ID: ";
    }

    rclcpp::shutdown();
    return 0;
}