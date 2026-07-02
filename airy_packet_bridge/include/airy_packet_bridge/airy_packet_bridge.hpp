// Copyright 2026 SavageTea
//
// Licensed under the Apache License, Version 2.0 (the "License");

#ifndef AIRY_PACKET_BRIDGE__AIRY_PACKET_BRIDGE_HPP_
#define AIRY_PACKET_BRIDGE__AIRY_PACKET_BRIDGE_HPP_

#include <ignition/msgs/point_cloud_packed.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rslidar_msg/msg/rslidar_packet.hpp>
#include <sensor_msgs/msg/imu.hpp>

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
  static constexpr int kBlockSize = 148;      // 2B magic + 2B azimuth + 48×3B channel
  static constexpr double kDistanceRes = 0.005;
  static constexpr int kPktsPerRevolution = 225;  // 100ms / 444.44μs
  static constexpr uint8_t kLidarMode96 = 0x02;

  // MSOP magic bytes
  static constexpr uint8_t kMsopMagic[4] = {0x55, 0xAA, 0x05, 0x5A};
  static constexpr uint8_t kBlockMagic[2] = {0xFF, 0xEE};

  // Airy Side mode firing time offsets (μs), 12 groups × 8 channels
  static constexpr double kChanTssUs[12] = {
    0.0, 11.424, 22.848, 34.272, 45.696, 57.120,
    68.544, 79.968, 91.392, 99.008, 110.432, 119.856
  };
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
// Main bridge node
// ===========================================================================
class AiryPacketBridge : public rclcpp::Node
{
public:
  explicit AiryPacketBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // =========================================================================
  // Gazebo Transport callback
  // =========================================================================
  void onGpuLidarBlock(const ignition::msgs::PointCloudPacked & msg);

  // =========================================================================
  // MSOP packet construction
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

  // =========================================================================
  // Members
  // =========================================================================
  std::shared_ptr<ignition::transport::Node> ign_node_;

  // ROS 2
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr packet_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Ring buffer (4 azimuth samples → 1 MSOP packet)
  AzimuthSample ring_[4];
  int ring_idx_{0};

  // Packet sequencing
  uint32_t pkt_seq_{0};

  // Parameters
  std::string robot_name_;
  std::string imu_topic_;
};

}  // namespace airy_packet_bridge

#endif  // AIRY_PACKET_BRIDGE__AIRY_PACKET_BRIDGE_HPP_
