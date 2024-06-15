#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int64.hpp"
#include <vector>

// GoalPublisher类，继承自rclcpp::Node
class GoalPublisher : public rclcpp::Node
{
public:
    // 构造函数
    GoalPublisher() : Node("pub_goal_pose") 
    {
        // 创建一个发布者，发布目标点
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        
        // 添加预设的目标点
        preset_goals_.push_back(createGoal(1.76, 0.71, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 三区中点
        preset_goals_.push_back(createGoal(1.76, 0.21, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 找框点
        preset_goals_.push_back(createGoal(1.96, 0.21, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 去取球点
        preset_goals_.push_back(createGoal(1.46, 0.21, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 去1号框
        preset_goals_.push_back(createGoal(1.86, 0.71, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 去2号框
        preset_goals_.push_back(createGoal(1.72, 0.21, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 去3号框
        preset_goals_.push_back(createGoal(1.76, 0.71, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 去4号框
        preset_goals_.push_back(createGoal(1.76, 0.71, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 去5号框
        preset_goals_.push_back(createGoal(1.76, 0.21, 0.0, 0.0, -sqrt(2) / 2, sqrt(2) / 2)); // 去6号框

        // 创建一个订阅者，订阅机器人模式
        robot_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "robot_mode", 
            10, 
            [this](const std_msgs::msg::Int64::SharedPtr msg) {
                this->robot_mode_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Robot mode: %ld", this->robot_mode_);
            }
        );

        // 创建一个订阅者，订阅选择
        choice_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "zhaokuang", 
            10, 
            [this](const std_msgs::msg::Int64::SharedPtr msg) {
                this->choice_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Choice: %ld", this->choice_);
                this->publish_goal_based_on_mode_and_choice();
            }
        );
    }

private:
    // 根据机器人模式和选择发布目标点
    void publish_goal_based_on_mode_and_choice()
    {
        if (robot_mode_ == 1)
        {
            publish_goal(preset_goals_[0]);
        }
        else if (robot_mode_ == 3)
        {
            publish_goal(preset_goals_[1]);
        }
        else if (robot_mode_ == 6)
        {
            publish_goal(preset_goals_[2]);
        }
        else if (robot_mode_ == 4)
        {
            if (choice_ == 1)
            {
                publish_goal(preset_goals_[3]);
            }
            else if (choice_ == 2)
            {
                publish_goal(preset_goals_[4]);
            }
            else if (choice_ == 3)
            {
                publish_goal(preset_goals_[5]);
            }
        }
    }

    // 创建一个目标点
    geometry_msgs::msg::PoseStamped createGoal(double px, double py, double ox, double oy, double oz, double ow)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.pose.position.x = px;
        goal.pose.position.y = py;
        goal.pose.orientation.x = ox;
        goal.pose.orientation.y = oy;
        goal.pose.orientation.z = oz;
        goal.pose.orientation.w = ow;
        return goal;
    }

    // 发布一个目标点
    void publish_goal(const geometry_msgs::msg::PoseStamped& goal)
    {
        auto goal_to_publish = goal;
        goal_to_publish.header.stamp = this->now();
        goal_to_publish.header.frame_id = "map";

        publisher_->publish(goal_to_publish);

        RCLCPP_INFO(this->get_logger(), "Publishing goal: Position: (%f, %f, %f)", goal_to_publish.pose.position.x, goal_to_publish.pose.position.y, goal_to_publish.pose.orientation.w);
    }

    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    // 订阅者
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr robot_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr choice_subscriber_;
    // 机器人模式
    int64_t robot_mode_ = 0;
    // 选择
    int64_t choice_ = 0;
    // 预设的目标点
    std::vector<geometry_msgs::msg::PoseStamped> preset_goals_;
};

// 主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}