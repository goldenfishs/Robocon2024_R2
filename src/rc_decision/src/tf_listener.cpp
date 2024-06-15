#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter(double initial_value, double process_noise, double measurement_noise)
        : x_hat_(initial_value), Q_(process_noise), R_(measurement_noise), P_(1.0) {}

    double filter(double measurement)
    {
        // Prediction
        double x_hat_minus = x_hat_;
        double P_minus = P_ + Q_;

        // Update
        double K = P_minus / (P_minus + R_);
        x_hat_ = x_hat_minus + K * (measurement - x_hat_minus);
        P_ = (1 - K) * P_minus;

        return x_hat_;
    }

private:
    double x_hat_, Q_, R_, P_;
};

class TFListener : public rclcpp::Node
{
public:
    TFListener()
        : Node("tf_listener"),
            x_filter_(0.0, 0.02, 0.05),
            y_filter_(0.0, 0.02, 0.05),
            w_filter_(0.0, 0.02, 0.05),
            roll_filter_(0.0, 0.02, 0.05),
            pitch_filter_(0.0, 0.02, 0.05),
            yaw_filter_(0.0, 0.02, 0.05)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("filtered_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TFListener::listen_tf, this));
    }

    void listen_tf()
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
            return;
        }

        double x = x_filter_.filter(transform_stamped.transform.translation.x);
        double y = y_filter_.filter(transform_stamped.transform.translation.y);

        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        roll = roll_filter_.filter(roll);
        pitch = pitch_filter_.filter(pitch);
        yaw = yaw_filter_.filter(yaw);

        tf2::Quaternion q_filtered;
        q_filtered.setRPY(roll, pitch, yaw);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = round(x * 10000) / 10000;
        pose.pose.position.y = round(y * 10000) / 10000;
        pose.pose.orientation.x = round(q_filtered.x() * 10000) / 10000;
        pose.pose.orientation.y = round(q_filtered.y() * 10000) / 10000;
        pose.pose.orientation.z = round(q_filtered.z() * 10000) / 10000;
        pose.pose.orientation.w = round(q_filtered.w() * 10000) / 10000;

        pose_publisher_->publish(pose);
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    KalmanFilter x_filter_, y_filter_, w_filter_, roll_filter_, pitch_filter_, yaw_filter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}