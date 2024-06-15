#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

class ParamSetter : public rclcpp::Node
{
public:
    ParamSetter() : Node("param_setter")
    {
        client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/controller_server/set_parameters");
        local_costmap_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
        global_costmap_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/global_costmap/global_costmap/set_parameters");
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "param_id", 10, std::bind(&ParamSetter::param_id_callback, this, std::placeholders::_1));

        presets_ = {
            {1, {
                {"FollowPath.min_vel_x", -1.0},
                {"FollowPath.max_vel_x", 1.0},
                {"FollowPath.min_vel_y", -1.0},
                {"FollowPath.max_vel_y", 1.0},
                {"FollowPath.min_speed_xy", -1.0},
                {"FollowPath.max_speed_xy", 1.0},
                {"FollowPath.max_vel_theta", 0.0},
                {"FollowPath.acc_lim_x", 1.0},
                {"FollowPath.acc_lim_y", 1.0},
                {"FollowPath.acc_lim_theta", 0.0},
                {"FollowPath.decel_lim_x", -1.0},
                {"FollowPath.decel_lim_y", -1.0},
                {"FollowPath.decel_lim_theta", 0.0},
                {"FollowPath.xy_goal_tolerance",0.05},
                {"general_goal_checker.xy_goal_tolerance",0.05},
                {"general_goal_checker.yaw_goal_tolerance",3.14},
                {"global_costmap.inflation_layer.inflation_radius", 0.02}, 
                {"local_costmap.inflation_layer.inflation_radius", 0.02}, 
            }},
            {2, {
                {"FollowPath.min_vel_x", -1.5},
                {"FollowPath.max_vel_x", 1.5},
                {"FollowPath.min_vel_y", -1.5},
                {"FollowPath.max_vel_y", 1.5},
                {"FollowPath.min_speed_xy", -1.5},
                {"FollowPath.max_speed_xy", 1.5},
                {"FollowPath.max_vel_theta", 2.0},
                {"FollowPath.acc_lim_x", 2.0},
                {"FollowPath.acc_lim_y", 2.0},
                {"FollowPath.acc_lim_theta", 2.0},
                {"FollowPath.decel_lim_x", -1.5},
                {"FollowPath.decel_lim_y", -1.5},
                {"FollowPath.decel_lim_theta", -2.0},
                {"FollowPath.xy_goal_tolerance",0.25},
                {"general_goal_checker.xy_goal_tolerance",0.25},
                {"general_goal_checker.yaw_goal_tolerance",0.05},
                {"global_costmap.inflation_layer.inflation_radius", 0.02}, 
                {"local_costmap.inflation_layer.inflation_radius", 0.02}, 
            }},
            // 添加更多预设...
        };
    }

private:
    void param_id_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        auto preset = presets_.find(msg->data);
        if (preset != presets_.end()) {
            auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            auto local_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            auto global_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
            for (const auto& param : preset->second) {
                rcl_interfaces::msg::Parameter parameter;
                parameter.name = param.first;
                if (param.first.find("local_costmap.") == 0) {
                    parameter.name = param.first.substr(strlen("local_costmap."));
                    local_costmap_request->parameters.push_back(parameter);
                } else if (param.first.find("global_costmap.") == 0) {
                    parameter.name = param.first.substr(strlen("global_costmap."));
                    global_costmap_request->parameters.push_back(parameter);
                } else {
                    request->parameters.push_back(parameter);
                }
            }
            RCLCPP_INFO(this->get_logger(), "Tuning parameters...");
            auto result = client_->async_send_request(request);
            auto local_costmap_result = local_costmap_client_->async_send_request(local_costmap_request);
            auto global_costmap_result = global_costmap_client_->async_send_request(global_costmap_request);
            RCLCPP_INFO(this->get_logger(), "Tuning succeeded.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Preset not found: %ld", msg->data);
        }
    }

    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_costmap_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr global_costmap_client_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    std::map<int64_t, std::map<std::string, double>> presets_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamSetter>());
    rclcpp::shutdown();
    return 0;
}