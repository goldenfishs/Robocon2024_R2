#include <iostream>
#include <string>
#include <serial/serial.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class SubSerialPubStatus : public rclcpp::Node
{
public:
    SubSerialPubStatus() : Node("sub_serial_pub_status"), ser(), last_publish_time_(0)
    {
        init_serial();
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("ball_take_status", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&SubSerialPubStatus::timer_callback, this));
    }

private:
    // 初始化串口
    void init_serial()
    {
        try{  
            ser.setPort("/dev/ttyUSB1");
            ser.setBaudrate(115200);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(500);  // 创建一个Timeout对象
            ser.setTimeout(timeout);  // 传递Timeout对象
            ser.open();
        }
        catch(serial::IOException &e){
            RCLCPP_INFO(this->get_logger(),"Unable to open port!!!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), ser.isOpen() ? "Serial Port initialized" : "Serial Port failed to open");
    }

    // 定时器回调函数，读取串口数据并发布消息
    void timer_callback()
    {
        auto message = std_msgs::msg::Int32();
        message.data = 0;  // 默认值为0

        if (ser.available()){
            std::string result = ser.read(ser.available());
            if (result.size() >= 4 && 
                static_cast<unsigned char>(result[0]) == 0xBD &&  // 将char转换为unsigned char
                static_cast<unsigned char>(result[1]) == 0xBD && 
                static_cast<unsigned char>(result[2]) == 0x00 && 
                static_cast<unsigned char>(result[3]) == 0xFE) {
                message.data = 1;  // 如果接收到特定字节序列，值设为1
                last_publish_time_ = this->now();  // 更新最后发布时间
            }
        }

        else if(ser.available()){
            std::string result = ser.read(ser.available());
            if (result.size() >= 4 && 
                static_cast<unsigned char>(result[0]) == 0xBD &&  // 将char转换为unsigned char
                static_cast<unsigned char>(result[1]) == 0xAC && 
                static_cast<unsigned char>(result[2]) == 0x00 && 
                static_cast<unsigned char>(result[3]) == 0x55) {
                message.data = 2;  // 如果接收到特定字节序列，值设为2
                last_publish_time_ = this->now();  // 更新最后发布时间
            }
        }

        // 如果最后发布时间距现在超过2秒，值设为0
        // if (this->now() - last_publish_time_ > rclcpp::Duration(2, 0)) {
        //     message.data = 0;
        // }

        publisher_->publish(message);  // 发布消息
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    serial::Serial ser;
    rclcpp::Time last_publish_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubSerialPubStatus>());
    rclcpp::shutdown();
    return 0;
}