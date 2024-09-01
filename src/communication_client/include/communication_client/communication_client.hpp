#ifndef COMMUNICATION_CLIENT_HPP
#define COMMUNICATION_CLIENT_HPP

#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace communication_client
{
class CommunicationClientNode : public rclcpp::Node
{
public:
  CommunicationClientNode(const rclcpp::NodeOptions & options);

private:
  int robot_id;

  void LivoxScanCallBack(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_scan_msg);
  void LivoxImuCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr livox_imu_msg);
  void CmdVelCallBack(const geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel_msg);

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr livox_imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr livox_imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};
} // namespace communication_client

#endif