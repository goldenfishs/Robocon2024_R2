#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/int64.hpp"
#include <vector>

class NavigateToPoseClient : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit NavigateToPoseClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("navigate_to_pose_client", options)
    {
        // 创建一个客户端以与/navigate_to_pose服务进行通信
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/navigate_to_pose");

        // 创建一个订阅器以接收/goal_pose_id话题
        this->goal_pose_id_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "/goal_pose_id", 10, std::bind(&NavigateToPoseClient::goal_pose_id_callback, this, std::placeholders::_1));
        
        // 创建一个订阅器以接收/robot_mode话题
        this->robot_mode_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "/robot_mode", 10, std::bind(&NavigateToPoseClient::robot_mode_callback, this, std::placeholders::_1));

        // 创建一个订阅器以接收/zhaokuang话题
        this->choice_sub_ = this->create_subscription<std_msgs::msg::Int64>(
            "/choice", 10, std::bind(&NavigateToPoseClient::choice_callback, this, std::placeholders::_1));

        // 添加目标点到数组
        add_goal_poses();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr goal_pose_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr robot_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr choice_sub_;

    std::vector<NavigateToPose::Goal> goal_poses_;
    // int case3_count = 0;  // 将case3_count作为类的一个私有成员变量
    // 添加目标点到数组
    void add_goal_poses()
    {
        this->goal_poses_.push_back(set_goal(1.50, -0.36, 0.0, 0.0, 0.0, -0.712, 0.712));//3区中点
        this->goal_poses_.push_back(set_goal(0.020, 1.52, 0.0, 0.0, 0.0, -0.712, 0.712));//1号框
        this->goal_poses_.push_back(set_goal(0.705, 1.53, 0.0, 0.0, 0.0, -0.712, 0.712));//2号框
        this->goal_poses_.push_back(set_goal(1.482, 1.53, 0.0, 0.0, 0.0, -0.712, 0.712));//3号框
        this->goal_poses_.push_back(set_goal(2.234, 1.53, 0.0, 0.0, 0.0, -0.712, 0.712));//4号框
        this->goal_poses_.push_back(set_goal(2.981, 1.53, 0.0, 0.0, 0.0, -0.712, 0.712));//5号框
        this->goal_poses_.push_back(set_goal(0.572, -0.05, 0.0, 0.0, 0.0, -0.712, 0.712));//决策点1
        this->goal_poses_.push_back(set_goal(2.307, 0.65, 0.0, 0.0, 0.0, -0.712, 0.712));//决策点2
        this->goal_poses_.push_back(set_goal(2.75, -2.50, 0.0, 0.0, 0.0, 1.0, 0.0));//取球点1
        this->goal_poses_.push_back(set_goal(0.333, -2.45, 0.0, 0.0, 0.0, 0.0, 1.0));//取球点2
        // this->goal_poses_.push_back(set_goal(0.50, -2.30, 0.0, 0.0, 0.0, 0.0, 1.0));//取球点2
        // 添加更多目标点...
    }

    // 当接收到新的/robot_mode消息时调用
    void robot_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到robot_mode: '%ld'", msg->data);

        // 根据接收到的robot_mode选择发送目标点
        switch (msg->data)
        {
            case 1:
                this->send_goal(this->goal_poses_[0]);
                break;
            // case 3:
            //     // case3_count++;
            //     // if (case3_count <= 3) {
            //     //     this->send_goal(this->goal_poses_[6]);
            //     // } else if (case3_count > 3 && case3_count <= 5) {
            //     //     this->send_goal(this->goal_poses_[7]);
            //     // }
            //     // if (case3_count == 5) {  // 如果case 3出现了5次，重置计数器
            //     //     case3_count = 0;
            //     // }
            //     this->send_goal(this->goal_poses_[6]);
            //     break;
            // case 6:
            //     this->send_goal(this->goal_poses_[9]);
            //     break;
            default:
                break;
        }
    }
    // 当接收到新的/zhaokuang消息时调用
    void choice_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到choice: '%ld'", msg->data);

        // 根据接收到的choice选择发送目标点
        if (msg->data >= 1 && static_cast<size_t>(msg->data) < this->goal_poses_.size())
        {
            // this->send_goal(this->goal_poses_[msg->data]);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无效的choice: '%ld'", msg->data);
        }
    }

    // 设置目标点
    NavigateToPose::Goal set_goal(double x, double y, double z, double ox, double oy, double oz, double ow)
    {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;
        goal.pose.pose.position.z = z;
        goal.pose.pose.orientation.x = ox;
        goal.pose.pose.orientation.y = oy;
        goal.pose.pose.orientation.z = oz;
        goal.pose.pose.orientation.w = ow;
        return goal;
    }

    // 发送目标点
    void send_goal(NavigateToPose::Goal goal)
    {
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        // 添加一个lambda函数来处理目标发送成功的情况
        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "目标发送失败");
                } else {
                    RCLCPP_INFO(this->get_logger(), "目标发送成功");
                }
            };

        this->client_ptr_->async_send_goal(goal, send_goal_options);
    }

    // 当接收到新的/goal_pose_id消息时调用:调试使用
    void goal_pose_id_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收到目标点id: '%ld'", msg->data);

        // 根据接收到的goal_pose_id从数组中选择目标点
        if (msg->data >= 0 && static_cast<size_t>(msg->data) < this->goal_poses_.size())
        {
            this->send_goal(this->goal_poses_[msg->data]);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无效的目标点id: '%ld'", msg->data);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigateToPoseClient>());
    rclcpp::shutdown();
    return 0;
}