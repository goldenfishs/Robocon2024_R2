#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp" // 新增
#include <serial/serial.h>

typedef union{
  uint8_t bytes[4];
  float value;
}u;

class send_move_command : public rclcpp::Node
{
public:
  send_move_command() : Node("send_move_command")
  {
    init_serial();
    init_subscriptions();
  }

private:
  void init_serial()
  {
    try{  
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
      RCLCPP_INFO(this->get_logger(),"Serial~ Port initialized");
    }else{
      RCLCPP_INFO(this->get_logger(),"Serial Port failed to open");
      return;
    }
  }

  void init_subscriptions()
  {
    subscription_target_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/ball_position", 100, std::bind(&send_move_command::callback_target, this, std::placeholders::_1));
    
    subscription_robot_mode_ = this->create_subscription<std_msgs::msg::Int64>(
      "/robot_mode", 100, std::bind(&send_move_command::callback_robot_mode, this, std::placeholders::_1));
    
    // 使用callback_cmd_vel_pid替换callback_cmd_vel
    subscription_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav", 100, std::bind(&send_move_command::callback_cmd_vel_pid, this, std::placeholders::_1));

    subscription_cmd_vel_pid_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_pid", 100, std::bind(&send_move_command::callback_cmd_vel_pid, this, std::placeholders::_1));
  } 


  void callback_target(const geometry_msgs::msg::Point::SharedPtr msg) // 修改
  {
    // RCLCPP_INFO(this->get_logger(),"Received target callback with data: %f, %f, %f", msg->data[0], msg->data[1], msg->data[2]);
    if (robot_mode == 2 ) {
      prepare_and_send_data(-msg->z, -msg->x, msg->y, 0x06, 0x01); // 修改
    }
  }

  void callback_robot_mode(const std_msgs::msg::Int64::SharedPtr msg) //用于接收机器人模式
  {
    RCLCPP_INFO(this->get_logger(),"Received robot_mode callback with data: %ld", msg->data);
    robot_mode = msg->data;
  }

  // void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) //用于接收cmd_vel
  // {
  //   float reversed_linear_x = -msg->linear.x;
  //   float reversed_linear_y = msg->linear.y;

  //   RCLCPP_INFO(this->get_logger(),"Received cmd_vel callback with data: %f, %f, %f", msg->linear.x, msg->linear.y, msg->angular.z);
  //   if (robot_mode == 1 ) {
  //     prepare_and_send_data(reversed_linear_x, reversed_linear_y, msg->angular.z, 0x09, 0x01);}
  // }

  void callback_cmd_vel_pid(const geometry_msgs::msg::Twist::SharedPtr msg) //用于接收cmd_vel_pid
  {
    RCLCPP_INFO(this->get_logger(),"Received cmd_vel_pid callback with data: %f, %f, %f", msg->linear.x, msg->linear.y, msg->angular.z);
    if (robot_mode == 1 || robot_mode == 6 ) {
      prepare_and_send_data(msg->linear.x, msg->linear.y, msg->angular.z, 0x09, 0x01);
    }else if (robot_mode == 3 || robot_mode == 4 || robot_mode == 5){
      prepare_and_send_data(msg->linear.x, msg->linear.y, msg->angular.z, 0x09, 0x00);
    }
  }


  void prepare_and_send_data(float x, float y, float z, uint8_t id, uint8_t pose)
  {
    u data_x, data_y, data_z;
    uint8_t send_data[arr_size]; 
    send_data[0] = usart_start;//0x11：底盘地址
    send_data[1] = id;//0x06:visoin, 0x09:nav,0x0a:锁陀螺仪
    send_data[2] = pose;//0x00:相机朝前, 0x01:相机朝后
    send_data[15] =usart_end;//0xFE,结束位
    data_x.value = x;
    data_y.value = y;
    data_z.value = z;

    add_float_to_send_data(send_data, 3, data_x);
    add_float_to_send_data(send_data, 7, data_y);
    add_float_to_send_data(send_data, 11, data_z);

    RCLCPP_INFO(this->get_logger(),"\nx:%f\ny:%f\nz:%f\n",x,y,z);
    send_data_over_serial(send_data);
  }

  void add_float_to_send_data(uint8_t* send_data, int start_index, u data)
  {
    for (int i = 0; i < 4; i++){
      send_data[start_index+i] = data.bytes[i];
    }
  }

  void send_data_over_serial(uint8_t* send_data)
  {
    if (ser.isOpen()){
      ser.write(send_data,sizeof(uint8_t)*arr_size);
    }
    else
      RCLCPP_INFO(this->get_logger(),"Serial Port is not open");
  }

public:
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_target_; // 修改
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_robot_mode_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_pid_;
  serial::Serial ser;
  std::string serial_port = "/dev/ttyUSB_chassis";   
  int baud_rate = 115200;                    
  uint8_t usart_start = 0xFF;                 
  uint8_t usart_id = 0x06; 
  uint8_t usart_end = 0xFE;
  const static int arr_size = sizeof(uint8_t)*4+sizeof(float)*3; 
  int robot_mode = 0; // 0: stop, 1: 上3区域, 2: 去取球, 3: 去找框, 4: 去放球 ,5: 放球, 6: 去取球点
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<send_move_command>());
  rclcpp::shutdown();
  return 0;
}