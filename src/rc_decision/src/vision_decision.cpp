#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "geometry_msgs/msg/point.hpp"
// #include "nav_msgs/msg/odometry.hpp"

class VisionDecision : public rclcpp::Node
{
public:
    VisionDecision() : Node("vision_decision")
    {
        robot_mode_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
            "/robot_mode", 10, std::bind(&VisionDecision::robot_mode_callback, this, std::placeholders::_1));

        quqiu_subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/quqiu", 10, std::bind(&VisionDecision::quqiu_callback, this, std::placeholders::_1));

        zhaokuang_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
            "/zhaokuang", 10, std::bind(&VisionDecision::zhaokuang_callback, this, std::placeholders::_1));

        // odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odom", 10, std::bind(&VisionDecision::odom_callback, this, std::placeholders::_1));

        ball_position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/ball_position", 10);
        choice_publisher_ = this->create_publisher<std_msgs::msg::Int64>("/choice", 10);


        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), //每100ms检测一次
            std::bind(&VisionDecision::timer_callback, this));
    }

private:
    void robot_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
       robot_mode_ = msg->data;
    //    RCCLP_INFO(this->get_logger(), "Robot mode: %d", robot_mode_);
    }

    void quqiu_callback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        if (robot_mode_ == 2) {
            ball_position_publisher_->publish(*msg);
            received_quqiu_ = true;
        }
    }
    int last_robot_mode_ = -1;  // 添加一个新的变量来跟踪robot_mode_的上一次值
    void zhaokuang_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (robot_mode_ == 4 && last_robot_mode_ != 4) {
            choice_publisher_->publish(*msg);
        }
        last_robot_mode_ = robot_mode_;  // 更新last_robot_mode_的值
    }

    void timer_callback()
    {
        if (robot_mode_ == 2 && !received_quqiu_) {
            geometry_msgs::msg::Point msg;
            msg.x = 0;
            msg.y = 250;
            msg.z = 0;
            ball_position_publisher_->publish(msg);
        }
        received_quqiu_ = false;
    }
    // void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    // {
    //     // TODO: Handle odom message
    // }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr robot_mode_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr quqiu_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr zhaokuang_subscription_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_position_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr choice_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool received_quqiu_;
    int robot_mode_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionDecision>());
    rclcpp::shutdown();
    return 0;
}