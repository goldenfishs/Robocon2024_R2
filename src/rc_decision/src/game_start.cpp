#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GameStartPublisher : public rclcpp::Node
{
public:
    GameStartPublisher()
    : Node("game_start_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("game", 10);
        publishGameStart();
    }

private:
    void publishGameStart()
    {
        std_msgs::msg::String msg;
        msg.data = "start";
        publisher_->publish(msg);//发布两次
        publisher_->publish(msg);//发布两次
        publisher_->publish(msg);//发布两次
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GameStartPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}