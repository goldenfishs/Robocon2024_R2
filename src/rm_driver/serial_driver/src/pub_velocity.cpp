#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <serial/serial.h>
#include <iomanip>

typedef union{
  uint8_t bytes[4];
  float value;
}u;

class pub_velocity : public rclcpp::Node
{
public:
  pub_velocity() : Node("pub_velocity")
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

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_pid", 100, std::bind(&pub_velocity::callback, this, std::placeholders::_1));
    
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    u line_vel_x,line_vel_y,angle_vel_z;
    uint8_t send_data[arr_size]; // 发送数据的数组
    send_data[0] = usart_start;
    send_data[1] = usart_id;
    send_data[2] = usart_pose;
    send_data[arr_size - 1] = usart_end;
    // send_data[arr_size - 1] = usart_end;
    line_vel_x.value = std::round(msg->linear.x * 10000) / 10000;
    line_vel_y.value = std::round(msg->linear.y * 10000) / 10000;
    angle_vel_z.value = std::round(msg->angular.z * 10000) / 10000;
    // line_vel_x.value *= 1;
    // line_vel_y.value *= -1;
    // angle_vel_z.value *= -1;
    for (int i = 0; i < 4; i++){
      send_data[3+i] = line_vel_x.bytes[i];     // x轴线速度
      send_data[7+i] = line_vel_y.bytes[i];     // y轴线速度
      send_data[11+i] = angle_vel_z.bytes[i];    // z轴角速度
    }

    // RCLCPP_INFO(this->get_logger(),"\nx:%f\ny:%f\nz:%f\n",line_vel_x.value,line_vel_y.value,angle_vel_z.value);
    // RCLCPP_INFO(this->get_logger(),"\nx:%f\ny:%f\nz:%f\n",msg->linear.x,msg->linear.y,msg->angular.z);
    RCLCPP_INFO(this->get_logger(),"\nx:%f\ny:%f\nz:%f\n",line_vel_x.value,line_vel_y.value,angle_vel_z.value);
    //while(1){
      if (ser.isOpen()){
        if (line_vel_x.value == 0 && line_vel_y.value == 0 && angle_vel_z.value == 0){
          // 发送三次
          for (int i = 0; i < 6; i++){
            ser.write(send_data,sizeof(uint8_t)*arr_size);
          }
        }
        else{
          ser.write(send_data,sizeof(uint8_t)*arr_size);
        }
          
      }
        // ser.write(send_data,sizeof(uint8_t)*arr_size);

      else
        RCLCPP_INFO(this->get_logger(),"Serial Port is not open");
    //}
  }

public:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  serial::Serial ser;
  std::string serial_port = "/dev/ttyUSB_chassis";   //串口名
  int baud_rate = 115200;                    //波特率
  uint8_t usart_start = 0xFF;                 //帧头
  uint8_t usart_id = 0x09;                    //ID
  uint8_t usart_pose = 0x01;                  //0x00:相机朝前, 0x01:相机朝后
  uint8_t usart_end = 0xFE;                   //帧尾
  const static int arr_size = sizeof(uint8_t)*4+sizeof(float)*3; //line_vel_x,line_vel_y,angle_vel_z
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pub_velocity>());
  rclcpp::shutdown();
  return 0;
}

