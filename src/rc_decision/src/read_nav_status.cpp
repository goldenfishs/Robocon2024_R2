#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "std_msgs/msg/string.hpp"

class NavigationStatusMonitor : public rclcpp::Node
{
public:
    NavigationStatusMonitor()
    : Node("read_nav_status")
    {
        subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status",
            10,
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                if (!msg->status_list.empty()) {
                    const auto& status = msg->status_list.back();
                    std::string status_str;
                    std_msgs::msg::String status_msg;  // 将status_msg的定义移动到这里
                    switch (status.status) {
                        case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
                            status_str = "未知";
                            break;
                        case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
                            status_str = "被接受";
                            break;
                        case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                            status_str = "执行中";
                            break;
                        case action_msgs::msg::GoalStatus::STATUS_CANCELING:
                            status_str = "取消中";
                            break;
                        case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                            status_str = "成功";
                            // 发布成功状态
                            status_msg.data = "succeed";
                            publisher_->publish(status_msg);
                            break;
                        case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                            status_str = "已取消";
                            break;
                        case action_msgs::msg::GoalStatus::STATUS_ABORTED:
                            status_str = "被中止";
                            break;
                        default:
                            status_str = "无效状态";
                            break;
                    }
                    RCLCPP_INFO(this->get_logger(), "收到导航状态: %s", status_str.c_str());
                }
            }
        );

        publisher_ = this->create_publisher<std_msgs::msg::String>("nav_status", 10);
    }

private:
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationStatusMonitor>());
    rclcpp::shutdown();
    return 0;
}