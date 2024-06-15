#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include <iostream>

class MoveModePubNode : public rclcpp::Node  // 修改了类名
{
public:
    MoveModePubNode() : Node("move_mode_pub_node")  // 修改了节点名称
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("move_mode", 10);  // 修改了发布的主题名称
    }

    void publish_move_mode(int64_t move_mode)  // 修改了函数名和参数名
    {
        std_msgs::msg::Int64 msg;
        msg.data = move_mode;  // 修改了赋值的变量名
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto move_mode_pub_node = std::make_shared<MoveModePubNode>();  // 修改了变量名和类名

    int64_t move_mode;  // 修改了变量名
    std::cout << "请输入移动模式: ";  // 修改了提示信息
    while (std::cin >> move_mode)  // 修改了输入的变量名
    {
        move_mode_pub_node->publish_move_mode(move_mode);  // 修改了变量名和函数名
        std::cout << "请输入移动模式: ";  // 修改了提示信息
    }

    rclcpp::shutdown();
    return 0;
}