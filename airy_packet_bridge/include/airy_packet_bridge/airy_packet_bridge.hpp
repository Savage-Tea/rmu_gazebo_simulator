// Copyright 2026 SavageTea
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AIRY_PACKET_BRIDGE__AIRY_PACKET_BRIDGE_HPP_
#define AIRY_PACKET_BRIDGE__AIRY_PACKET_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rslidar_msg/msg/rslidar_packet.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace airy_packet_bridge
{

// ===========================================================================
// Airy MSOP packet constants (from rs_driver decoder_RSAIRY)
// ===========================================================================
struct MsopConstants
{
  static constexpr int kBlocksPerPkt = 8;
  static constexpr int kChPerBlock = 48;
  static constexpr int kTotalLines = 96;
  static constexpr int kPktSize = 1248;
  static constexpr int kHeaderSize = 42;
  static constexpr int kBlockSize = 148;       // 2B magic + 2B azimuth + 48×3B channel
  static constexpr int kChannelSize = 3;        // 2B distance + 1B intensity
  static constexpr int kPktsPerRevolution = 225;
  static constexpr double kDistanceRes = 0.005;
  static constexpr uint8_t kLidarMode96 = 0x02;

  // MSOP magic bytes
  static constexpr uint8_t kMsopMagic[4] = {0x55, 0xAA, 0x05, 0x5A};
  static constexpr uint8_t kBlockMagic[2] = {0xFF, 0xEE};
};

// ===========================================================================
// One azimuth direction worth of 96 points
// ===========================================================================
struct AzimuthSample
{
  double azimuth_deg = 0.0;
  double timestamp_sec = 0.0;
  float x[96]{};
  float y[96]{};
  float z[96]{};
  bool valid[96]{};
};

// ===========================================================================
// Main bridge node: ROS2 PointCloud2 → Airy MSOP packets
// ===========================================================================
class AiryPacketBridge : public rclcpp::Node
{
public:
  explicit AiryPacketBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // =========================================================================
  // ROS 2 PointCloud2 callback
  // =========================================================================
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // =========================================================================
  // MSOP packet construction (unchanged)
  // =========================================================================
  void buildMsopPacket(
    const AzimuthSample & az0,
    const AzimuthSample & az1,
    const AzimuthSample & az2,
    const AzimuthSample & az3,
    double pkt_ts_sec,
    uint32_t pkt_seq,
    rslidar_msg::msg::RslidarPacket & pkt);

  void encodeBlock(
    uint8_t * buf,
    uint16_t azimuth_raw,
    int base_ring,
    const AzimuthSample & sample);

  void encodeChannel(
    uint8_t * buf,
    int ring_id,
    const AzimuthSample & sample);

  // =========================================================================
  // Utilities
  // =========================================================================
  static uint16_t toRawAzimuth(double deg)
  {
    double az = std::fmod(deg + 360.0, 360.0);
    return static_cast<uint16_t>(az * 100.0);
  }

  static uint16_t toRawDistance(float dist)
  {
    return static_cast<uint16_t>(dist / MsopConstants::kDistanceRes) & 0x3FFF;
  }

  void onTimer();

  // =========================================================================
  // Members
  // =========================================================================
  // ROS 2
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr packet_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub_;
  rclcpp::TimerBase::SharedPtr pkt_timer_;

  // Ring buffer (4 azimuth samples → 1 MSOP packet)
  AzimuthSample ring_[4];
  int ring_idx_{0};

  // Azimuth tracking for drop detection (Fix #7)
  double last_azimuth_deg_{0.0};
  bool last_azimuth_valid_{false};

  // Packet sequencing
  uint32_t pkt_seq_{0};

  // Cloud buffer for timer-driven column extraction
  std::vector<float> cloud_xs_, cloud_ys_, cloud_zs_;
  uint32_t cloud_w_{0}, cloud_h_{0};
  double cloud_ts_{0.0};
  uint32_t cloud_col_{0};
  bool cloud_fresh_{false};

  // GT odometry tracking for synthetic acceleration
  rclcpp::Time last_gt_time_{0, 0, RCL_ROS_TIME};
  double last_gt_vx_{0}, last_gt_vy_{0}, last_gt_vz_{0};
  sensor_msgs::msg::Imu last_imu_;

  // Parameters
  std::string robot_name_;
  std::string imu_topic_;
  std::string cloud_topic_;
};

}  // namespace airy_packet_bridge

#endif  // AIRY_PACKET_BRIDGE__AIRY_PACKET_BRIDGE_HPP_
