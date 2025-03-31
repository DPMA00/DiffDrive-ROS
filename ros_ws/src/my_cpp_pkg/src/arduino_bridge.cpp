#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <serial/serial.h> 

class ArduinoBridge : public rclcpp::Node {
public:
  ArduinoBridge() : Node("arduino_bridge") {
    // Publishers and Subscribers
    sensor_pub_ = this->create_publisher<std_msgs::msg::Int32>("arduino/sensor", 10);
    led_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "arduino/led", 10, std::bind(&ArduinoBridge::led_callback, this, std::placeholders::_1));

    // Setup Serial Port
    try {
      serial_.setPort("/dev/ttyACM0");
      serial_.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(100);
      serial_.setTimeout(to);
      serial_.open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    }

    // Timer to poll Arduino serial
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ArduinoBridge::read_serial, this));
  }

private:
  void read_serial() {
    if (serial_.available()) {
      std::string line = serial_.readline(100, "\n");
      try {
        int sensor_val = std::stoi(line);
        std_msgs::msg::Int32 msg;
        msg.data = sensor_val;
        sensor_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Received: %d", sensor_val);
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Invalid serial data: '%s'", line.c_str());
      }
    }
  }

  void led_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    char cmd = msg->data ? '1' : '0';
    try {
        serial_.write(reinterpret_cast<const uint8_t*>(&cmd), 1);       
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial");
    }
  }

  serial::Serial serial_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sensor_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr led_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArduinoBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
