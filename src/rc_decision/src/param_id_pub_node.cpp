#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include <iostream>

class ParamIdPublisher : public rclcpp::Node  // 修改了类名
{
public:
    ParamIdPublisher() : Node("param_id_publisher")  // 修改了节点名称
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("param_id", 10);  // 修改了发布的主题名称
    }

    void publish_param_id(int64_t param_id)  // 修改了函数名和参数名
    {
        std_msgs::msg::Int64 msg;
        msg.data = param_id;  // 修改了赋值的变量名
        publisher_->publish(msg);
    }
    
private:
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto param_id_publisher = std::make_shared<ParamIdPublisher>();  // 修改了变量名和类名

    int64_t param_id;  // 修改了变量名
    std::cout << "请输入参数ID: ";  // 修改了提示信息
    while (std::cin >> param_id)  // 修改了输入的变量名
    {
        param_id_publisher->publish_param_id(param_id);  // 修改了变量名和函数名
        std::cout << "请输入参数ID: ";  // 修改了提示信息
    }

    rclcpp::shutdown();
    return 0;
}