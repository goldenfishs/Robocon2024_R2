#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <serial/serial.h>

typedef union{
  uint8_t bytes[4];
  float value;
}u;

class pub_velocity : public rclcpp::Node
{
public:
  pub_velocity() : Node("pub_odom"), last_send_time(std::chrono::high_resolution_clock::now())  // 初始化last_send_time
  {
    try{  //打开串口
      ser.setPort(serial_port);
      ser.setBaudrate(baud_rate);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(500);
      ser.setTimeout(timeout);
      ser.open();
    }
    catch(serial::IOException &e){
      RCLCPP_INFO(this->get_logger(),"Unable to open port!!!");
      return;
    }
    if (ser.isOpen()){
      RCLCPP_INFO(this->get_logger(),"Serial Port initalized");
    }else{
      RCLCPP_INFO(this->get_logger(),"Serial Port ???");
      return;
    }

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 100, std::bind(&pub_velocity::callback, this, std::placeholders::_1)); //订阅话题
   
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto position_x = msg->pose.pose.position.x;
    auto position_y = msg->pose.pose.position.y;
    auto orientation_w = msg->pose.pose.orientation.w;

    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time);
    last_send_time = now;

    u position_x_u, position_y_u, orientation_w_u;
    position_x_u.value = position_x;
    position_y_u.value = position_y;
    orientation_w_u.value = orientation_w;

    send_data[0] = usart_start;  // 添加包头
    send_data[1] = usart_id;  // 添加id
    for (int i = 0; i < 4; i++){
      send_data[2+i] = position_x_u.bytes[i];     // x坐标
      send_data[6+i] = position_y_u.bytes[i];     // y坐标
      send_data[10+i] = orientation_w_u.bytes[i];  // w值
    }
    send_data[arr_size - 1] = usart_end;  // 添加包尾
    ser.write(send_data, arr_size);  //发送数据
    RCLCPP_INFO(this->get_logger(),"\nx:%f\ny:%f\nw:%f\nInterval: %ld ms\n",position_x,position_y,orientation_w, duration.count()); //打印接收到的数据和间隔
  }
private:
  std::chrono::high_resolution_clock::time_point last_send_time;  // 添加这一行来存储上一次发送数据的时间

public:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  serial::Serial ser;
  std::string serial_port = "/dev/ttyUSB0";   //串口名
  int baud_rate = 115200;                    //波特率
  uint8_t usart_start = 0xFF;                 //帧头
  uint8_t usart_id = 0x08;                 //帧头
  uint8_t usart_end = 0xFE;                   //帧尾
  const static int arr_size = sizeof(uint8_t)*3+sizeof(float)*3; //定义数组大小
  uint8_t send_data[arr_size];                //定义send_data数组
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pub_velocity>());
  rclcpp::shutdown();
  return 0;
}

