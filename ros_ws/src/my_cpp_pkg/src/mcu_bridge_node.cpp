#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <my_robot_interfaces/msg/robot_info.hpp>
#include <my_robot_interfaces/msg/motor_odom_info.hpp>
#include <example_interfaces/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sstream>
#include <iomanip>

using namespace std::placeholders;


class MCUBridge : public rclcpp::Node{
    public: 
    
    MCUBridge() : Node("mcu_bridge_node")
    {
        robot_info_publisher = this->create_publisher<my_robot_interfaces::msg::RobotInfo>("robot_info",10);
        motor_odom_info_publisher = this->create_publisher<my_robot_interfaces::msg::MotorOdomInfo>("motor_odom_info",10);
        imu_info_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu/data",10);
        robot_target_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&MCUBridge::velocity_callback, this, std::placeholders::_1));
        encoder_reset_server = this->create_service<example_interfaces::srv::Trigger>("encoder_reset_server", 
                                std::bind(&MCUBridge::callbackResetEncoder, this, _1, _2));
        encoder_reset_client = this->create_client<example_interfaces::srv::Trigger>("encoder_reset_server");

        try {
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(to);
            serial_.open();
            set_zero_vel();
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
          }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MCUBridge::read_serial, this));
    }

    void set_zero_vel()
        {
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(3) << 0.00 << " " << 0.00 << "\n";
            std::string command = ss.str();
            serial_.write(reinterpret_cast<const uint8_t *>(command.c_str()), command.length());
        }
    
    void reset_encoders_on_startup()
    {
        while(!encoder_reset_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for server");
        }
        auto request = std::make_shared<example_interfaces::srv::Trigger::Request>();
        
        encoder_reset_client->async_send_request(
            request, std::bind(&MCUBridge::callBackRequestEncoderReset, this, _1));
    }

    private:
        void callBackRequestEncoderReset(rclcpp::Client<example_interfaces::srv::Trigger>::SharedFuture future)
        {
            auto response= future.get();
        }

        void callbackResetEncoder(const example_interfaces::srv::Trigger::Request::SharedPtr request, const example_interfaces::srv::Trigger::Response::SharedPtr response)
        {
            
            response->message = "Sending reset command...";
            std::string reset_cmd = "reset";
            serial_.write(reinterpret_cast<const uint8_t*>(reset_cmd.c_str()), reset_cmd.length());

        }

        void decode(std::string& line)
        {
            line.erase(0,1);
            std::istringstream iss(line);

            long ENC_R,ENC_L;
            float RPM_R,RPM_L,CUR_R,CUR_L,VOLT,POW;
            float ACC_X,ACC_Y,ACC_Z,OMEG_X,OMEG_Y,OMEG_Z,TEMP;

            if (iss 
                >> ENC_R >> ENC_L >> RPM_R >> RPM_L 
                >> CUR_R >> CUR_L >> VOLT >> POW 
                >> ACC_X >> ACC_Y >> ACC_Z 
                >> OMEG_X >> OMEG_Y >> OMEG_Z >> TEMP)
            {
                my_robot_interfaces::msg::MotorOdomInfo odom;
                odom.left_encoder = ENC_L;
                odom.right_encoder = ENC_R;
                odom.left_rpm = RPM_L;
                odom.right_rpm = RPM_R;

                my_robot_interfaces::msg::RobotInfo robot_info;
                robot_info.motor_left_current = CUR_L;
                robot_info.motor_right_current = CUR_R;
                robot_info.battery_voltage = VOLT;
                robot_info.battery_power = POW;
                robot_info.env_temperature = TEMP;

                sensor_msgs::msg::Imu imu;
                imu.header.stamp = this->get_clock()->now();
                imu.header.frame_id = "imu_link";
                
                imu.linear_acceleration.x = ACC_X;
                imu.linear_acceleration.y = ACC_Y;
                imu.linear_acceleration.z = ACC_Z;
                imu.angular_velocity.x = OMEG_X;
                imu.angular_velocity.y = OMEG_Y;
                imu.angular_velocity.z = OMEG_Z;

                imu.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};
                imu.angular_velocity_covariance = {5.809e-6, 0,0,0,8.7848e-6,0,0,0,3.401e-6};
                
                motor_odom_info_publisher->publish(odom);
                robot_info_publisher->publish(robot_info);
                imu_info_publisher->publish(imu);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Malformed serial input: '%s'", line.c_str());
            }
        }

        void read_serial()
        {
            if (serial_.available())
            {
                std::string line;
                try
                {
                    line = serial_.readline(128, "\n");
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
                
                decode(line);
                
            }
        }


        void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr data)
        {
            float cmd_vx = data->linear.x;
            float cmd_omega = data->angular.z;

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
    rclcpp::Publisher<my_robot_interfaces::msg::RobotInfo>::SharedPtr robot_info_publisher;
    rclcpp::Publisher<my_robot_interfaces::msg::MotorOdomInfo>::SharedPtr motor_odom_info_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_info_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr robot_target_vel_subscriber;
    rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr encoder_reset_server;
    rclcpp::Client<example_interfaces::srv::Trigger>::SharedPtr encoder_reset_client;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr reset_timer_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MCUBridge>();
    node->reset_encoders_on_startup();
    rclcpp::spin(node);
    rclcpp::shutdown;
    return 0;
}