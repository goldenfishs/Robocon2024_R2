#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

// 定义一个名为ParamAdjuster的类，该类继承自rclcpp::Node
class ParamAdjuster : public rclcpp::Node
{
public:
    // 构造函数
    ParamAdjuster() : Node("param_adjuster")
    {
        // 创建参数服务的客户端
        create_clients();
        // 创建订阅者
        create_subscriber();
        // 初始化预设参数集
        initialize_presets();
    }

private:
    // 创建参数服务的客户端
    void create_clients()
    {
        controller_server_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/controller_server/set_parameters");
        local_costmap_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/local_costmap/local_costmap/set_parameters");
        global_costmap_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/global_costmap/global_costmap/set_parameters");
        velocity_smoother_client_ = this->create_client<rcl_interfaces::srv::SetParameters>("/velocity_smoother/set_parameters");
    }

    // 创建订阅者
    void create_subscriber()
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "param_id", 10, std::bind(&ParamAdjuster::param_id_callback, this, std::placeholders::_1));
        robot_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int64>(
            "robot_mode", 10, std::bind(&ParamAdjuster::robot_mode_callback, this, std::placeholders::_1));
    }

    // 初始化预设参数集
    void initialize_presets()
    {
        presets_ = {
            {1, {
                {"controller_server.FollowPath.max_vel_x", {1.0}},
                {"controller_server.general_goal_checker.xy_goal_tolerance", {1.0}},
                {"controller_server.general_goal_checker.yaw_goal_tolerance", {1.0}},
                {"global_costmap/global_costmap.inflation_layer.inflation_radius", {0.02}},
                {"local_costmap/local_costmap.inflation_layer.inflation_radius", {0.02}},
                {"velocity_smoother.max_accel", {0.8, 0.8, 1.0}},
            }},//适合用于高速1区域到3区域。
            {2, {
                {"controller_server.FollowPath.max_vel_x", {1.5}},
                {"controller_server.general_goal_checker.xy_goal_tolerance", {1.0}},
                {"controller_server.general_goal_checker.yaw_goal_tolerance", {1.0}},
                {"global_costmap/global_costmap.inflation_layer.inflation_radius", {0.72}},
                {"local_costmap/local_costmap.inflation_layer.inflation_radius", {0.02}},
                {"velocity_smoother.max_accel", {1.8, 2.8, 1.0}},
            }},//适合用于在3区快速移动。
            {3, {
                {"controller_server.FollowPath.max_vel_x", {1.5}},
                {"controller_server.general_goal_checker.xy_goal_tolerance", {1.0}},
                {"controller_server.general_goal_checker.yaw_goal_tolerance", {1.0}},
                {"global_costmap/global_costmap.inflation_layer.inflation_radius", {0.72}},
                {"local_costmap/local_costmap.inflation_layer.inflation_radius", {0.02}},
                {"velocity_smoother.max_accel", {1.8, 2.8, 1.0}},
            }},//适合用于靠近框

            
        };
    }

    // 回调函数，当接收到robot_mode话题的消息时调用
    void robot_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (msg->data != current_robot_mode_) {
            current_robot_mode_ = msg->data;
            int64_t preset_id;
            if (current_robot_mode_ == 0) {
                preset_id = 1;  // 当r2启动时，修改参数为预设1
            } else if (current_robot_mode_ == 5) {
                preset_id = 2;  // 当robot_mode为5时，使用预设2
            } else if (current_robot_mode_ == 6) {
                preset_id = 3;  // 当robot_mode为6时，使用预设3
            } else {
                return;  // 对于其他的robot_mode值，不做任何操作
            }
            if (presets_.find(preset_id) != presets_.end()) {
                set_parameters(presets_[preset_id]);
            }
        }
    }

    //回调函数，当接收到param_id话题的消息时调用
    void param_id_callback(const std_msgs::msg::Int64::SharedPtr msg)
    {
        if (presets_.find(msg->data) != presets_.end()) {
            set_parameters(presets_[msg->data]);
        }
    }

    // 设置参数
    void set_parameters(const std::map<std::string, std::vector<double>>& params) {
        auto controller_server_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        auto local_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        auto global_costmap_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        auto velocity_smoother_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        std::map<std::string, rcl_interfaces::srv::SetParameters::Request::SharedPtr> requests = {
            {"controller_server.", controller_server_request},
            {"local_costmap/local_costmap.", local_costmap_request},
            {"global_costmap/global_costmap.", global_costmap_request},
            {"velocity_smoother.", velocity_smoother_request},
        };

        for (const auto& param : params) {
            for (const auto& request : requests) {
                if (param.first.find(request.first) == 0) {
                    rcl_interfaces::msg::Parameter parameter;
                    parameter.name = param.first.substr(strlen(request.first.c_str()));
                    parameter.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                    parameter.value.double_value = param.second[0];
                    request.second->parameters.push_back(parameter);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Tuning parameters...");
        auto controller_server_result_future = controller_server_client_->async_send_request(controller_server_request);
        auto local_costmap_result_future = local_costmap_client_->async_send_request(local_costmap_request);
        auto global_costmap_result_future = global_costmap_client_->async_send_request(global_costmap_request);
        auto velocity_smoother_result_future = velocity_smoother_client_->async_send_request(velocity_smoother_request);
        RCLCPP_INFO(this->get_logger(), "Tuning succeeded.");
    }

    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr controller_server_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_costmap_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr global_costmap_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr velocity_smoother_client_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr robot_mode_subscriber_;
    std::map<int64_t, std::map<std::string, std::vector<double>>> presets_;
    int64_t current_robot_mode_ = -1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamAdjuster>());
    rclcpp::shutdown();
    return 0;
}