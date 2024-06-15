#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>  
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}//初始化:kp ki kd,积分项,上一次误差

    double control(double setpoint, double pv)
    {
        double error = setpoint - pv;
        integral_ += error;
        double derivative = error - prev_error_;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_, integral_, prev_error_;
};

struct TargetPose {
    double x, y, yaw;
};

class PoseController : public rclcpp::Node
{
public:
    PoseController()
        : Node("pose_controller"),
            x_controller_(0.12, 0.0, 0.0),
            y_controller_(0.15, 0.0, 0.0),
            yaw_controller_(0.12, 0.0, 0.0),
            robot_mode_(0)  // 初始化机器人模式为1

    { 
        // Initialize target poses
        target_poses_ = {
            // {0.5, 0.5, -M_PI/2},//直取球
            {1.582, 0.00, 0.0},//侧取球
            {0.000, 1.60, -M_PI/2},
            {0.705, 1.60, -M_PI/2},
            {1.482, 1.60, -M_PI/2},
            {2.234, 1.60, -M_PI/2},
            {2.981, 1.60, -M_PI/2},
            // {1.582, 0.0, -M_PI/2},
            // Add more target poses here
        };
        target_ = target_poses_[0];

        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "filtered_pose", 10, std::bind(&PoseController::pose_callback, this, std::placeholders::_1));
        cmd_pid_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_pid", 10);
        position_id_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "/position_id", 10, std::bind(&PoseController::position_id_callback, this, std::placeholders::_1));
        pid_status_publisher_ = this->create_publisher<std_msgs::msg::String>("/pid_status", 10); 
            robot_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
        "robot_mode", 10, std::bind(&PoseController::robot_mode_callback, this, std::placeholders::_1)); 
        
    }

    void robot_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        robot_mode_ = msg->data;  // 更新 robot_mode_ 的值
        if (msg->data == 1) {
            target_ = target_poses_[0];
        }
    }

    void position_id_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        size_t id = static_cast<size_t>(msg->data);
        if (id < target_poses_.size()) {
            target_ = target_poses_[id];
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid position id: %ld", id);
        }
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;

        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.orientation, quat);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, yaw: %f", x, y, yaw);  // 修改这里
        TargetPose target_local = transformTargetPose(x, y, yaw);
        calculateControl(target_local);
    }

private:
    TargetPose transformTargetPose(double x, double y, double yaw)
    {
        TargetPose target_local;
        target_local.x = cos(yaw) * (target_.x - x) + sin(yaw) * (target_.y - y);
        target_local.y = -sin(yaw) * (target_.x - x) + cos(yaw) * (target_.y - y);
        target_local.yaw = target_.yaw - yaw;
        if (target_local.yaw > M_PI)
            target_local.yaw -= 2 * M_PI;
        else if (target_local.yaw < -M_PI)
            target_local.yaw += 2 * M_PI;
        return target_local;
    }

    void calculateControl(const TargetPose &target_local)
    {
        double error_threshold = 0.05;  // 设定阈值
        double control_x = 0.0;
        double control_y = 0.0;
        double control_yaw = 0.0;
        if (std::abs(target_local.x) > error_threshold || std::abs(target_local.y) > error_threshold || std::abs(target_local.yaw) > error_threshold) {
            control_x = x_controller_.control(0.0, target_local.x);
            control_y = y_controller_.control(0.0, target_local.y);
            control_yaw = yaw_controller_.control(0.0, target_local.yaw);
            // 加入限制
            if (control_x > 0.2)
                control_x = 0.2;
            if (control_y > 0.2)
                control_y = 0.2;
        } else {
            std_msgs::msg::String msg;
            if (robot_mode_ == 6) {  // 如果 robot_mode_ 为6，发布 "OKK"
                msg.data = "OKK";
            } else {
                msg.data = "OK";
            }
            pid_status_publisher_->publish(msg);  // 发布 "OK" 或 "OKK"
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = control_x;
        cmd.linear.y = -control_y;
        cmd.angular.z = control_yaw;
        cmd_pid_publisher_->publish(cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pid_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pid_status_publisher_; 
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr robot_mode_subscriber_;
    std::vector<TargetPose> target_poses_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr position_id_subscriber_;
    PIDController x_controller_, y_controller_, yaw_controller_;
    TargetPose target_;
    int robot_mode_; 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}