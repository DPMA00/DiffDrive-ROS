#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/msg/motor_odom_info.hpp>
#include <my_robot_interfaces/msg/cmd_drive_vel.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

constexpr double PI = 3.14159265358979323846;
constexpr double MAX_VELOCITY = 0.6;  // 1.05 m/s
constexpr double MAX_ANGULAR_VELOCITY = 13;  // 13.3 rad/s


class DiffDriveControllerNode : public rclcpp::Node
{
public:
    DiffDriveControllerNode() : Node("pid_diffdrivedrive_control")
    {
        odom_info_subscription = this->create_subscription<my_robot_interfaces::msg::MotorOdomInfo>(
            "arduino/motor_odom_info", 10, std::bind(&DiffDriveControllerNode::MotorOdomInfoCallback, this, std::placeholders::_1));

        velocity_publisher = this->create_publisher<my_robot_interfaces::msg::CmdDriveVel>("arduino/cmd_vel", 10);

        odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("diffdrive/odometry", 10);

        timer_ = this->create_wall_timer(30ms, std::bind(&DiffDriveControllerNode::controller, this));

        target_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
            "diffdrive/target_pos",10, std::bind(&DiffDriveControllerNode::TargetPosCallback, this, std::placeholders::_1));

    }

private:

    double pos_x {0};
    double pos_y {0};
    double ori_z {0};
    double pos_error = {0};
    double heading_error = {0};
    double Kp_pos {0.6};
    double Kp_head {0.9};
    
    double wheel_radius {0.05};
    double wheel_base {0.3};

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

    double x_target {0};
    double y_target {0};

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

        double d_left = wheel_radius*delta_L_encoder * (2*PI/360);
        double d_right = wheel_radius*delta_R_encoder * (2*PI/360);


        double d_avg = (d_left + d_right)/2;
        double delta_theta = (d_right-d_left)/wheel_base;

        
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
        pos_error = sqrt(pow((x_target-pos_x),2) + pow((y_target-pos_y),2));
        double heading = atan2((y_target-pos_y), (x_target-pos_x));
        heading_error = heading - ori_z;
        //RCLCPP_INFO(this->get_logger(), "heading: %f | heading_error: %f", rad2deg(heading), rad2deg(heading_error));
        if (pos_error >= 0.05)
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

        data.pose.pose.orientation.w = rad2deg(ori_z);
        RCLCPP_INFO(this->get_logger(), "x: %f | y: %f | theta: %f", pos_x, pos_y, rad2deg(ori_z));

        data.twist.twist.linear.x = control_v;
        data.twist.twist.angular.z = control_w;

        odometry_publisher->publish(data);
        
    }

    void TargetPosCallback(const geometry_msgs::msg::Vector3::SharedPtr target)
    {
        x_target = target->x;
        y_target = target->y;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Subscription<my_robot_interfaces::msg::MotorOdomInfo>::SharedPtr odom_info_subscription;
    rclcpp::Publisher<my_robot_interfaces::msg::CmdDriveVel>::SharedPtr velocity_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_subscriber;
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


