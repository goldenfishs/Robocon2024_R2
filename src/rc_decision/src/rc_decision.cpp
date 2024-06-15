#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"
#include <cstdio>

// 初始化机器人的状态为0
int robot_mode = 0;

// 创建发布者
rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub;

// 游戏开始的回调函数
void gameStartCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "start") {
        robot_mode = 1;
        printf("Game started\n");
        std_msgs::msg::Int64 mode_msg;
        mode_msg.data = robot_mode;
        pub->publish(mode_msg);
    }
}

// 导航状态的回调函数
void navStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "succeed" && robot_mode == 1) {
        robot_mode = 2;
        printf("robot_mode = 2\n");
        std_msgs::msg::Int64 mode_msg;
        mode_msg.data = robot_mode;
        pub->publish(mode_msg);
    }
}

// pid的状态的回调函数
void pidStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "OK") {
        switch (robot_mode) {
            case 3:
                robot_mode = 4;
                printf("robot_mode = 4\n");
                break;
            case 4:
                robot_mode = 5;
                printf("robot_mode = 5\n");
                break;
            // case 6:
            //     robot_mode = 2;
            //     printf("robot_mode = 2\n");
            //     break;
            default:
                break;
            }
        std_msgs::msg::Int64 mode_msg;
        mode_msg.data = robot_mode;
        pub->publish(mode_msg);
    }else if (msg->data == "OKK") {
        if (robot_mode == 6) {
            robot_mode = 2;
            printf("robot_mode = 2\n");
            std_msgs::msg::Int64 mode_msg;
            mode_msg.data = robot_mode;
            pub->publish(mode_msg);
        }
    }
}

// 球的状态的回调函数
void ballStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "picked_up" && robot_mode == 2) {
        robot_mode = 3;
        printf("robot_mode = 3\n");
    } else if (msg->data == "put_down" && robot_mode == 5) {
        robot_mode = 6;
        printf("robot_mode = 6\n");
    }
    std_msgs::msg::Int64 mode_msg;
    mode_msg.data = robot_mode;
    pub->publish(mode_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_mode_publisher");

    pub = node->create_publisher<std_msgs::msg::Int64>("robot_mode", 10);

    // 发布初始状态
    std_msgs::msg::Int64 mode_msg;
    mode_msg.data = robot_mode;
    pub->publish(mode_msg);

    auto game_sub = node->create_subscription<std_msgs::msg::String>("game", 10, gameStartCallback);
    auto nav_status_sub = node->create_subscription<std_msgs::msg::String>("nav_status", 10, navStatusCallback);
    auto ball_status_sub = node->create_subscription<std_msgs::msg::String>("ball_status", 10, ballStatusCallback);
    auto pid_status_sub = node->create_subscription<std_msgs::msg::String>("pid_status", 10, pidStatusCallback);
    rclcpp::Rate loop_rate(10);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}