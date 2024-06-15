#include <iostream>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int64.hpp>

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("serial_node"), ser()
  {
    init_serial();
    publisher_ = this->create_publisher<std_msgs::msg::String>("ball_status", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "robot_mode", 10, std::bind(&SerialNode::robot_mode_callback, this, std::placeholders::_1));
  }

  void spin()
  {
    while (rclcpp::ok()) {
      read_serial();
      rclcpp::spin_some(shared_from_this());
    }
  }

private:
  void init_serial()
  {
    try {
      ser.setPort("/dev/ttyUSB_mechanism");
      ser.setBaudrate(115200);
      serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(timeout);
      ser.open();
    }
    catch (serial::IOException& e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open port.");
      rclcpp::shutdown();
    }

    if (ser.isOpen()){
      RCLCPP_INFO(this->get_logger(), "Successfully opened port.");
    }
    else {
      rclcpp::shutdown();
    }
  }

  void read_serial()
  {
      if (ser.available()) {
          std::string line = ser.readline();
          if (line.size() > 4) {  // 如果数据长度大于4
              line = line.substr(line.size() - 4);  // 只保留最后四个字节
          }
          print_received_data(line);
          process_data(line);
      }
  }

  void process_data(const std::string& data)
  {
    int dataType = get_data_type(data);
    if (dataType == 1) {
      publish_message("picked_up");
    } else if (dataType == 2) {
      publish_message("put_down");
    }
  }

  void print_received_data(const std::string& data)
  {
    std::ostringstream oss;
    oss << "Received: ";
    for (unsigned char c : data) {
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
    }
    RCLCPP_INFO(this->get_logger(), oss.str().c_str());
  }

  int get_data_type(const std::string& data)
  {
    if (data.size() < 4 || data[0] != '\xbd') {
      return 0;  // 无效数据
    }
    unsigned char checksum = calculate_checksum(data.substr(0, 3));
    if (checksum != static_cast<unsigned char>(data[3])) {
      return 0;  // 校验失败
    }
    if (data[1] == '\xbd' && data[2] == '\x00') {
      return 1;  // picked up
    }
    if (data[1] == '\xac' && data[2] == '\x00') {
      return 2;  // put down
    }
    return 0;  // 未知类型
  }

  unsigned char calculate_checksum(const std::string& data)
  {
    unsigned char checksum = 0;
    for (char c : data) {
      checksum += static_cast<unsigned char>(c);
    }
    return checksum;
  }

  void publish_message(const std::string& data)
  {
    auto message = std_msgs::msg::String();
    message.data = data;
    publisher_->publish(message);
  }

  void robot_mode_callback(const std_msgs::msg::Int64::SharedPtr msg)
  {
    uint8_t data[4] = {0xcc, 0x00, 0x00, 0x00};
    if (msg->data == 2) {
      data[1] = 0x0d;
    } else if (msg->data == 5) {
      data[1] = 0xef;
    } else {
      return;
    }
    data[3] = data[0] + data[1] + data[2];
    ser.write(data, sizeof(data));

    std::ostringstream oss;
    oss << "Sent: ";
    for (unsigned char c : data) {
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(c) << " ";
    }
    RCLCPP_INFO(this->get_logger(), oss.str().c_str());
  }

  serial::Serial ser;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialNode>();
  node->spin();
  rclcpp::shutdown();
  return 0;
}