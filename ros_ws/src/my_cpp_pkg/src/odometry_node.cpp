#include <rclcpp/rclcpp.hpp>
#include <my_robot_interfaces/msg/motor_odom_info.hpp>
#include <my_robot_interfaces/msg/cmd_drive_vel.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"

#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

constexpr double PI = 3.14159265358979323846;


class OdometryNode : public rclcpp::Node
{
public: OdometryNode() : Node("odometry_node")
    {
        odom_info_subscriber= this->create_subscription<my_robot_interfaces::msg::MotorOdomInfo>(
            "arduino/motor_odom_info", 10, std::bind(&OdometryNode::MotorOdomInfoCallback, this, std::placeholders::_1));

        odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(50ms, std::bind(&OdometryNode::UpdateOdom,this));

    }

private:
    double pos_x {0};
    double pos_y {0};
    double ori_z {0};
    
    double wheel_radius {0.05};
    double wheel_base {0.29};


    long prev_L_enc_pos {0};
    long prev_R_enc_pos {0};

    long L_enc_pos {0};
    long R_enc_pos {0};

    rclcpp::Time prev_time =this->get_clock()->now();

    float prev_L_rpm {0};
    float prev_R_rpm {0};
    float L_rpm {0};
    float R_rpm {0};

    double linear_velocity {0};
    double angular_velocity {0};

    double rad2deg(double angle)
    {
        return angle*180/PI;
    }

    double clamp(double val, double min_val, double max_val) 
    {
        return std::max(min_val, std::min(val, max_val));
    }      

    void calcOdometry()
    {
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now-prev_time).seconds();
        prev_time = now;

        if (dt <= 1e-5) return;

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

        linear_velocity = d_avg/dt;
        angular_velocity = delta_theta/dt;

        double delta_x = d_avg * cos(ori_z);
        double delta_y = d_avg * sin(ori_z);

        pos_x += delta_x;
        pos_y += delta_y;
        
        ori_z = atan2(sin(ori_z), cos(ori_z));

    }


    void MotorOdomInfoCallback(const my_robot_interfaces::msg::MotorOdomInfo::SharedPtr odom)
    {
        L_enc_pos = odom->left_encoder;
        R_enc_pos = odom->right_encoder;
        L_rpm = odom->left_rpm;
        R_rpm = odom->right_rpm;
    }


    void OdometryPublisher()
    {
        auto data = nav_msgs::msg::Odometry();
        data.header.stamp = this->get_clock()->now();
        data.header.frame_id = "odom";
        data.child_frame_id = "base_footprint";
        data.pose.pose.position.x = pos_x;
        data.pose.pose.position.y = pos_y;
        data.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0,0,ori_z);

        data.pose.pose.orientation.x = q.x();
        data.pose.pose.orientation.y = q.y();
        data.pose.pose.orientation.z = q.z();
        data.pose.pose.orientation.w = q.w();

        data.twist.twist.linear.x = linear_velocity;
        data.twist.twist.angular.z = angular_velocity;

        
        odometry_publisher->publish(data);
        
    }


    void PublishTF()
    {
        auto tf = geometry_msgs::msg::TransformStamped();
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "odom";          
        tf.child_frame_id = "base_footprint";
        
        tf.transform.translation.x = pos_x;
        tf.transform.translation.y = pos_y;
        tf.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0,0,ori_z);

        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        this->tf_broadcaster_->sendTransform(tf);
    }

    void UpdateOdom()
    {
        calcOdometry();
        OdometryPublisher();
        PublishTF();
    }


    rclcpp::Subscription<my_robot_interfaces::msg::MotorOdomInfo>::SharedPtr odom_info_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


