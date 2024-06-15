#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"  // Add this line
#include "std_msgs/msg/int64.hpp"  // Add this line
#include <vector>

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher() : Node("pub_goal_pose") 
  {
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "NavigateToPose");

    // Define your goals
    auto goal1 = geometry_msgs::msg::PoseStamped();//三区中点
    goal1.pose.position.x = 1.55;
    goal1.pose.position.y = -0.36;
    goal1.pose.position.z = 0.0;
    goal1.pose.orientation.x = 0.0;
    goal1.pose.orientation.y = 0.0;
    goal1.pose.orientation.z = -0.70;
    goal1.pose.orientation.w = 0.70;

    goals_.push_back(goal1);

    auto goal2 = geometry_msgs::msg::PoseStamped();
    goal2.pose.position.x = -0.11;
    goal2.pose.position.y = 1.50;
    goal2.pose.orientation.w = 0.70;
    goal2.pose.orientation.z = -0.70;
    goals_.push_back(goal2);

    auto goal3 = geometry_msgs::msg::PoseStamped();
    goal3.pose.position.x = 0.72;
    goal3.pose.position.y = 1.50;
    goal3.pose.orientation.w = 0.70;
    goal3.pose.orientation.z = -0.70;
    goals_.push_back(goal3);

    auto goal4 = geometry_msgs::msg::PoseStamped();
    goal4.pose.position.x = 1.59;
    goal4.pose.position.y = 1.50;
    goal4.pose.orientation.w = 0.70;
    goal4.pose.orientation.z = -0.70;;
    goals_.push_back(goal4);

    auto goal5 = geometry_msgs::msg::PoseStamped();
    goal5.pose.position.x = 2.43;
    goal5.pose.position.y = 1.50;
    goal5.pose.orientation.w = 0.70;
    goal5.pose.orientation.z = -0.70;;
    goals_.push_back(goal5);
    // Define more goals as needed

    // 创建一个订阅器以接收目标点ID
    subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "goal_pose_id",
      10,
      [this](const std_msgs::msg::Int64::SharedPtr msg) {
        if (msg->data >= 0 && static_cast<size_t>(msg->data) < goals_.size())
        {
          send_goal(goals_[msg->data]);
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "未知的目标点ID: %ld", msg->data);
        }
      }
    );
  }

private:
  void send_goal(const geometry_msgs::msg::PoseStamped& goal)
  {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [](std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(rclcpp::get_logger("pub_goal_pose"), "Goal was rejected by server");
        } else {
          RCLCPP_INFO(rclcpp::get_logger("pub_goal_pose"), "Goal accepted by server, waiting for result");
        }
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisher>());
  rclcpp::shutdown();
  return 0;
}