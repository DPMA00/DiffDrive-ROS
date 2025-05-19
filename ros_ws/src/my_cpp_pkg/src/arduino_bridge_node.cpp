#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <my_robot_interfaces/msg/cmd_drive_vel.hpp>
#include <my_robot_interfaces/msg/motor_info.hpp>
#include <my_robot_interfaces/msg/motor_odom_info.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <sstream>
#include <iomanip>

using namespace std::placeholders;


class ArduinoBridge : public rclcpp::Node{
    public: 
    
    ArduinoBridge() : Node("arduino_bridge_node")
    {
        motor_info_publisher = this->create_publisher<my_robot_interfaces::msg::MotorInfo>("arduino/motor_info",10);
        motor_odom_info_publisher = this->create_publisher<my_robot_interfaces::msg::MotorOdomInfo>("arduino/motor_odom_info",10);
        robot_target_vel_subscriber = this->create_subscription<my_robot_interfaces::msg::CmdDriveVel>("arduino/cmd_vel", 10, std::bind(&ArduinoBridge::velocity_callback, this, std::placeholders::_1));
        encoder_reset_server = this->create_service<example_interfaces::srv::Trigger>("encoder_reset_server", 
                                std::bind(&ArduinoBridge::callbackResetEncoder, this, _1, _2));

        try {
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(to);
            serial_.open();
            reset_encoders_on_startup();
            set_zero_vel();
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
          }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ArduinoBridge::read_serial, this));
    }

    void set_zero_vel()
        {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(3) << 0.00 << " " << 0.00 << "\n";
            std::string command = ss.str();
            serial_.write(reinterpret_cast<const uint8_t *>(command.c_str()), command.length());
        }

    private:

        void callbackResetEncoder(const example_interfaces::srv::Trigger::Request::SharedPtr request, const example_interfaces::srv::Trigger::Response::SharedPtr response)
        {
            
            response->message = "Sending reset command...";
            std::string reset_cmd = "reset";
            serial_.write(reinterpret_cast<const uint8_t*>(reset_cmd.c_str()), reset_cmd.length());

        }

        void reset_encoders_on_startup()
        {
            std::string reset_cmd = "reset";
            serial_.write(reinterpret_cast<const uint8_t*>(reset_cmd.c_str()), reset_cmd.length());
        }


        void read_serial()
        {
            if (serial_.available())
            {
                std::string line;
                try
                {
                    line = serial_.readline(1000, "\n");
                    //RCLCPP_INFO(this->get_logger(), line.c_str());
                } 
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Serial read error: %s", e.what());
                    return;
                }

                if (line.empty() || line[0] != '$')
                {
                    return;
                }
                line.erase(0,1);
                
                std::istringstream iss(line);

                long val1, val2;
                float val3, val4, val5, val6, val7;

                if (iss >> val1 >> val2 >> val3 >> val4 >> val5 >> val6 >> val7)
                {
                    my_robot_interfaces::msg::MotorOdomInfo motor_odom_info;
                    motor_odom_info.left_encoder = val2; //reversed
                    motor_odom_info.right_encoder = val1;
                    motor_odom_info.left_rpm= val4; //reversed
                    motor_odom_info.right_rpm = val3;

                    my_robot_interfaces::msg::MotorInfo motor_info;
                    motor_info.left_current = val5;
                    motor_info.right_current = val6;
                    motor_info.battery_power = val7;
                    
                    motor_odom_info_publisher->publish(motor_odom_info);
                    motor_info_publisher->publish(motor_info);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Malformed serial input: '%s'", line.c_str());
                }
            }
        }


        void velocity_callback(const my_robot_interfaces::msg::CmdDriveVel::SharedPtr data)
        {
            float cmd_vx = data->vx;
            float cmd_omega = data->omega;

            try
            {
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(3) << cmd_vx << " " << cmd_omega << "\n";

                std::string command = ss.str();
                serial_.write(reinterpret_cast<const uint8_t *>(command.c_str()), command.length());

                //RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", command.c_str());
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to serial: %s", e.what());
            }
        }


    serial::Serial serial_;
    rclcpp::Publisher<my_robot_interfaces::msg::MotorInfo>::SharedPtr motor_info_publisher;
    rclcpp::Publisher<my_robot_interfaces::msg::MotorOdomInfo>::SharedPtr motor_odom_info_publisher;
    rclcpp::Subscription<my_robot_interfaces::msg::CmdDriveVel>::SharedPtr robot_target_vel_subscriber;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr encoder_reset_server;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ArduinoBridge>();
    rclcpp::spin(node);
    node->set_zero_vel();
    rclcpp::shutdown;
    return 0;
}