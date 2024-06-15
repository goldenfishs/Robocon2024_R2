#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <serial/serial.h>

typedef union{
    uint8_t bytes[4];
    float value;
}u;

class pub_target : public rclcpp::Node
{
public:
    pub_target() : Node("pub_target")
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

        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/target_ball", 100, std::bind(&pub_target::callback, this, std::placeholders::_1));
        
    }

private:
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        u vel_x,pitch,deepth;
        uint8_t send_data[arr_size]; // 发送数据的数组
        send_data[0] = usart_start;
        send_data[1] = usart_id;
        // send_data[arr_size - 1] = usart_end;
        vel_x.value = msg->data[0];
        pitch.value = msg->data[1];
        deepth.value = msg->data[2];

        for (int i = 0; i < 4; i++){
            send_data[2+i] = vel_x.bytes[i];     // x轴线速度
            send_data[6+i] = pitch.bytes[i];     // pitch速度
            send_data[10+i] = deepth.bytes[i];    // 距离
        }

        RCLCPP_INFO(this->get_logger(),"\nvel_x:%f\npitch:%f\ndeepth:%f\n",msg->data[0],msg->data[1],msg->data[2]);
        if (ser.isOpen()){
            ser.write(send_data,sizeof(uint8_t)*arr_size);
            }
            else
                RCLCPP_INFO(this->get_logger(),"Serial Port is not open");
    }

public:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    serial::Serial ser;
    std::string serial_port = "/dev/ttyUSB0";   //串口名
    int baud_rate = 115200;                    //波特率
    uint8_t usart_start = 0x11;                 //地址
    uint8_t usart_id = 0x06;                    //功能码
    // uint8_t usart_end = 0x05;                   //帧尾
    const static int arr_size = sizeof(uint8_t)*2+sizeof(float)*3; //vel_x,pitch,deepth
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pub_target>());
    rclcpp::shutdown();
    return 0;
}