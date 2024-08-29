#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>

#include "communication_client/communication_client.hpp"

namespace communication_client
{
CommunicationClientNode::CommunicationClientNode(const rclcpp::NodeOptions & options)
  : Node("communication_client", options)
{
  this->declare_parameter<int>("robot_id", 0);

  this->get_parameter("robot_id", robot_id);
  livox_scan_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    "/robot_" + std::to_string(robot_id) + "/livox/lidar",
    5,
    std::bind(&CommunicationClientNode::LivoxScanCallBack, this, std::placeholders::_1));
  livox_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/robot_" + std::to_string(robot_id) + "/livox/imu",
    5,
    std::bind(&CommunicationClientNode::LivoxImuCallBack, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel",
    5,
    std::bind(&CommunicationClientNode::CmdVelCallBack, this, std::placeholders::_1));

  livox_scan_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 5);
  livox_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 5);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/robot_" + std::to_string(robot_id) + "/cmd_vel", 5);
}

void CommunicationClientNode::LivoxScanCallBack(
  const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_scan_msg)
{
  livox_scan_pub_->publish(*livox_scan_msg);
}

void CommunicationClientNode::LivoxImuCallBack(
  const sensor_msgs::msg::Imu::ConstSharedPtr livox_imu_msg)
{
  livox_imu_pub_->publish(*livox_imu_msg);
}

void CommunicationClientNode::CmdVelCallBack(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd_vel_msg)
{
  geometry_msgs::msg::Twist twist = cmd_vel_msg->twist;
  cmd_vel_pub_->publish(twist);
}

} // namespace communication_client

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(communication_client::CommunicationClientNode)