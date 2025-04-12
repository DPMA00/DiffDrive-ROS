/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string port = "/dev/ttyUSB0";
  node->declare_parameter("port", port);
  node->get_parameter("port", port);
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

  std::string ignore_array = "";
  node->declare_parameter("ignore_array", ignore_array);
  node->get_parameter("ignore_array", ignore_array);
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  // Integer parameters
  int baudrate = 230400;
  node->declare_parameter("baudrate", baudrate);
  node->get_parameter("baudrate", baudrate);
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  int lidar_type = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type", lidar_type);
  node->get_parameter("lidar_type", lidar_type);
  laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  int device_type = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter("device_type", device_type);
  node->get_parameter("device_type", device_type);
  laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

  int sample_rate = 9;
  node->declare_parameter("sample_rate", sample_rate);
  node->get_parameter("sample_rate", sample_rate);
  laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

  int abnormal_check_count = 4;
  node->declare_parameter("abnormal_check_count", abnormal_check_count);
  node->get_parameter("abnormal_check_count", abnormal_check_count);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count, sizeof(int));

  int intensity_bit = 8;
  node->declare_parameter("intensity_bit", intensity_bit);
  node->get_parameter("intensity_bit", intensity_bit);
  laser.setlidaropt(LidarPropIntenstiyBit, &intensity_bit, sizeof(int));

  // Boolean parameters
  bool fixed_resolution = false;
  node->declare_parameter("fixed_resolution", fixed_resolution);
  node->get_parameter("fixed_resolution", fixed_resolution);
  laser.setlidaropt(LidarPropFixedResolution, &fixed_resolution, sizeof(bool));

  bool reversion = true;
  node->declare_parameter("reversion", reversion);
  node->get_parameter("reversion", reversion);
  laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));

  bool inverted = true;
  node->declare_parameter("inverted", inverted);
  node->get_parameter("inverted", inverted);
  laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));

  bool auto_reconnect = true;
  node->declare_parameter("auto_reconnect", auto_reconnect);
  node->get_parameter("auto_reconnect", auto_reconnect);
  laser.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));

  bool isSingleChannel = false;
  node->declare_parameter("isSingleChannel", isSingleChannel);
  node->get_parameter("isSingleChannel", isSingleChannel);
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));

  bool intensity = false;
  node->declare_parameter("intensity", intensity);
  node->get_parameter("intensity", intensity);
  laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));

  bool support_motor_dtr = false;
  node->declare_parameter("support_motor_dtr", support_motor_dtr);
  node->get_parameter("support_motor_dtr", support_motor_dtr);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr, sizeof(bool));

  // Float parameters
  float angle_max = 180.0;
  node->declare_parameter("angle_max", angle_max);
  node->get_parameter("angle_max", angle_max);
  laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));

  float angle_min = -180.0;
  node->declare_parameter("angle_min", angle_min);
  node->get_parameter("angle_min", angle_min);
  laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));

  float range_max = 64.0;
  node->declare_parameter("range_max", range_max);
  node->get_parameter("range_max", range_max);
  laser.setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));

  float range_min = 0.1;
  node->declare_parameter("range_min", range_min);
  node->get_parameter("range_min", range_min);
  laser.setlidaropt(LidarPropMinRange, &range_min, sizeof(float));

  float frequency = 10.0;
  node->declare_parameter("frequency", frequency);
  node->get_parameter("frequency", frequency);
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);


  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }
  
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;//

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);
      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(*scan_msg);


    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
