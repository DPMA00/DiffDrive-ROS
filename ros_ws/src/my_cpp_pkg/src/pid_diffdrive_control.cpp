#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/msg/motor_odom_info.hpp>
#include <my_robot_interfaces/msg/cmd_drive_vel.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

constexpr double PI = 3.14159265358979323846;
constexpr double MAX_VELOCITY = 0.6;  // 1.05 m/s
constexpr double MAX_ANGULAR_VELOCITY = 5;  // 13.3 rad/s


class DiffDriveControllerNode : public rclcpp::Node
{
public:
    DiffDriveControllerNode() : Node("pid_diffdrivedrive_control")
    {
        this->declare_parameter("pose", std::vector<double>{2, 4});

        odom_info_subscription = this->create_subscription<my_robot_interfaces::msg::MotorOdomInfo>(
            "/arduino/motor_odom_info", 10, std::bind(&DiffDriveControllerNode::MotorOdomInfoCallback, this, std::placeholders::_1));

        velocity_publisher = this->create_publisher<my_robot_interfaces::msg::CmdDriveVel>("arduino/cmd_vel", 10);

        odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("diffdrive/odometry", 10);

        timer_ = this->create_wall_timer(30ms, std::bind(&DiffDriveControllerNode::controller, this));

    }

private:

    double pos_x {0};
    double pos_y {0};
    double ori_z {0};
    double pos_error = {0};
    double heading_error = {0};
    double Kp_pos {0.6};
    double Kp_head {0.8};
    
    double wheel_radius {0.05};
    double wheel_base {0.245};

    float control_v {0};
    float control_w {0};

    long prev_L_enc_pos {0};
    long prev_R_enc_pos {0};

    long L_enc_pos {0};
    long R_enc_pos {0};

    float prev_L_rpm {0};
    float prev_R_rpm {0};
    float L_rpm {0};
    float R_rpm {0};

    double rad2deg(double angle)
    {
        return angle*180/PI;
    }

    double clamp(double val, double min_val, double max_val) {
        return std::max(min_val, std::min(val, max_val));
      }      

    void calcOdometry()
    {
        int delta_L_encoder;
        int delta_R_encoder;
        
        delta_L_encoder = -(L_enc_pos-prev_L_enc_pos);
        delta_R_encoder = -(R_enc_pos-prev_R_enc_pos);

        prev_L_enc_pos = L_enc_pos;
        prev_R_enc_pos = R_enc_pos;
        RCLCPP_INFO(this->get_logger(), "L_enc: %ld | R_enc: %ld", L_enc_pos, R_enc_pos);

        double d_left = wheel_radius*delta_L_encoder * (2*PI/360);
        double d_right = wheel_radius*delta_R_encoder * (2*PI/360);


        double d_avg = (d_left + d_right)/2;
        double delta_theta = (d_right-d_left)/wheel_base;

        RCLCPP_INFO(this->get_logger(), "delta_theta: %f", delta_theta);
        
        ori_z += delta_theta;

        double delta_x = d_avg * cos(ori_z);
        double delta_y = d_avg * sin(ori_z);

        pos_x += delta_x;
        pos_y += delta_y;
        
        ori_z = atan2(sin(ori_z), cos(ori_z));

    }

    void publishVelocity()
    {
        auto msg = my_robot_interfaces::msg::CmdDriveVel();
        msg.vx = control_v;
        msg.omega = control_w;
        velocity_publisher->publish(msg);
    }

    void controller()
    {
        std::vector<double> setpoint;
        this->get_parameter("pose", setpoint);
        double x_target = setpoint[0];
        double y_target = setpoint[1];



        pos_error = sqrt(pow((x_target-pos_x),2) + pow((y_target-pos_y),2));
        double heading = atan2((y_target-pos_y), (x_target-pos_x));
        heading_error = heading - ori_z;
        RCLCPP_INFO(this->get_logger(), "heading: %f | heading_error: %f", rad2deg(heading), rad2deg(heading_error));
        if (pos_error >= 0.1)
        {
            heading_error = atan2(sin(heading_error), cos(heading_error));


            control_v = clamp(Kp_pos * pos_error, -MAX_VELOCITY, MAX_VELOCITY);
            control_w = clamp(-Kp_head * heading_error, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        }

        else
        {
            control_v = 0;
            control_w = 0;
        }

        publishVelocity();

    }
    void MotorOdomInfoCallback(const my_robot_interfaces::msg::MotorOdomInfo::SharedPtr odom)
    {
        L_enc_pos = odom->left_encoder;
        R_enc_pos = odom->right_encoder;
        L_rpm = odom->left_rpm;
        R_rpm = odom->right_rpm;


        calcOdometry();
        OdometryPublisher();
    }

    void OdometryPublisher()
    {
        auto data = nav_msgs::msg::Odometry();
        data.pose.pose.position.x = pos_x;
        data.pose.pose.position.y = pos_y;

        RCLCPP_INFO(this->get_logger(), "x: %.2f, y: %.2f, theta: %.2f", pos_x, pos_y, rad2deg(ori_z));

        odometry_publisher->publish(data);
        
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Subscription<my_robot_interfaces::msg::MotorOdomInfo>::SharedPtr odom_info_subscription;
    rclcpp::Publisher<my_robot_interfaces::msg::CmdDriveVel>::SharedPtr velocity_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffDriveControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


