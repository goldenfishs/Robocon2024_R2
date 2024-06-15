#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

class ChoiceNode : public rclcpp::Node
{
public:
    ChoiceNode() : Node("send_choice"), last_mode_(0), choice_index_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("choice", 10);
        position_id_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/position_id", 10);  
        subscription_ = this->create_subscription<std_msgs::msg::Int64>(
            "robot_mode", 10, std::bind(&ChoiceNode::robot_mode_callback, this, std::placeholders::_1));
    }

private:
    void robot_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (last_mode_ == 2 && msg->data == 4) {
            publish_choice();
        } else if (msg->data == 6) {
            publish_position_id_zero();
        }
        last_mode_ = msg->data;
    }

    void publish_position_id_zero()
    {
        std_msgs::msg::Int64 message;
        message.data = 0;
        position_id_publisher_->publish(message);
    }

    void publish_choice()
    {
        std_msgs::msg::Int64 message;
        message.data = choices_[choice_index_];
        publisher_->publish(message);
        position_id_publisher_->publish(message);

        choice_index_ = (choice_index_ + 1) % choices_.size();
    }

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr position_id_publisher_;
    int64_t last_mode_;
    size_t choice_index_;
    std::vector<int64_t> choices_ = {1, 2, 3, 4, 5};
    // std::vector<int64_t> choices_ = {1, 2, 3,};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChoiceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}