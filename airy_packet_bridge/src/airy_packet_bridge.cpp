// Copyright 2026 SavageTea
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "airy_packet_bridge/airy_packet_bridge.hpp"

#include <ignition/transport/Node.hh>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace airy_packet_bridge
{

AiryPacketBridge::AiryPacketBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("airy_packet_bridge", options)
{
  // =========================================================================
  // Parameters
  // =========================================================================
  robot_name_ = this->declare_parameter("robot_name", "red_standard_robot1");
  lidar_link_ = this->declare_parameter("lidar_link", "front_mid360");
  imu_topic_ = this->declare_parameter("imu_topic", "livox/imu");

  // =========================================================================
  // ROS 2 publishers
  // =========================================================================
  packet_pub_ = this->create_publisher<rslidar_msg::msg::RslidarPacket>(
    "/lidar/packets", rclcpp::SensorDataQoS());

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/lidar/imu", rclcpp::SensorDataQoS());

  // =========================================================================
  // IMU subscription (passthrough from Gazebo)
  // =========================================================================
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      imu_pub_->publish(*msg);
    });

  // =========================================================================
  // Gazebo Transport subscription (Fix #5: uses lidar_link_ parameter)
  // =========================================================================
  ign_node_ = std::make_shared<ignition::transport::Node>();
  std::string gz_topic = "/world/default/model/" + robot_name_ +
    "/link/" + lidar_link_ + "/sensor/airy_lidar/scan/points";

  if (!ign_node_->Subscribe(gz_topic, &AiryPacketBridge::onGpuLidarBlock, this)) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to subscribe to Gazebo topic: %s", gz_topic.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(),
      "Subscribed to Gazebo topic: %s", gz_topic.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
    "AiryPacketBridge ready. robot=%s, link=%s", robot_name_.c_str(), lidar_link_.c_str());
}

// ===========================================================================
// Gazebo Transport callback — called ~2250 times/sec
// ===========================================================================
void AiryPacketBridge::onGpuLidarBlock(
  const ignition::msgs::PointCloudPacked & msg)
{
  // Fix #3: validate message payload size to prevent out-of-bounds read
  if (msg.field() != "xyz" ||
      msg.width() != MsopConstants::kTotalLines ||
      static_cast<int>(msg.data().size()) < MsopConstants::kExpectedDataSize) {
    return;
  }

  const float * data = reinterpret_cast<const float *>(msg.data().data());

  // =========================================================================
  // Parse into AzimuthSample
  // =========================================================================
  AzimuthSample sample;
  sample.timestamp_sec = static_cast<double>(msg.header().stamp().sec()) +
                         static_cast<double>(msg.header().stamp().nsec()) * 1e-9;

  // Compute azimuth from first valid point's x,y
  bool azimuth_set = false;
  for (int ring = 0; ring < MsopConstants::kTotalLines; ++ring) {
    const float * p = data + ring * 3;
    if (std::isfinite(p[0]) && std::isfinite(p[1])) {
      sample.x[ring] = p[0];
      sample.y[ring] = p[1];
      sample.z[ring] = p[2];
      sample.valid[ring] = true;

      if (!azimuth_set) {
        sample.azimuth_deg = std::atan2(-p[1], p[0]) * 180.0 / M_PI;
        azimuth_set = true;
      }
    } else {
      sample.valid[ring] = false;
    }
  }

  if (!azimuth_set) return;  // all NAN — skip

  // =========================================================================
  // Fix #7: azimuth continuity check — detect dropped messages
  // =========================================================================
  if (last_azimuth_valid_) {
    double expected_azimuth = last_azimuth_deg_ + 0.4;  // kAzimuthPerBlock ~0.4°
    if (expected_azimuth >= 180.0) expected_azimuth -= 360.0;
    double gap = std::abs(sample.azimuth_deg - expected_azimuth);
    // Allow small tolerance for floating-point rounding
    if (gap > 0.8) {  // more than 2 blocks' worth of gap
      RCLCPP_WARN(this->get_logger(),
        "Azimuth discontinuity detected (last=%.2f, cur=%.2f, gap=%.2f). Resetting ring buffer.",
        last_azimuth_deg_, sample.azimuth_deg, gap);
      ring_idx_ = 0;
    }
  }
  last_azimuth_deg_ = sample.azimuth_deg;
  last_azimuth_valid_ = true;

  // =========================================================================
  // Insert into ring buffer
  // =========================================================================
  ring_[ring_idx_ % 4] = sample;
  ring_idx_++;

  // =========================================================================
  // Every 4 azimuth samples → publish one MSOP packet
  // =========================================================================
  if (ring_idx_ % 4 == 0) {
    rslidar_msg::msg::RslidarPacket pkt;
    double pkt_ts = ring_[0].timestamp_sec;

    buildMsopPacket(
      ring_[0], ring_[1], ring_[2], ring_[3],
      pkt_ts, pkt_seq_++, pkt);

    packet_pub_->publish(pkt);
  }
}

// ===========================================================================
// Build a complete Airy MSOP packet (1248 bytes)
// ===========================================================================
void AiryPacketBridge::buildMsopPacket(
  const AzimuthSample & az0,
  const AzimuthSample & az1,
  const AzimuthSample & az2,
  const AzimuthSample & az3,
  double pkt_ts_sec,
  uint32_t pkt_seq,
  rslidar_msg::msg::RslidarPacket & pkt)
{
  pkt.data.resize(MsopConstants::kPktSize);
  uint8_t * buf = pkt.data.data();
  std::memset(buf, 0, MsopConstants::kPktSize);

  // =========================================================================
  // MSOP Header (42 bytes)
  // =========================================================================
  std::memcpy(buf + 0, MsopConstants::kMsopMagic, 4);  // offset 0: magic
  // offset 4-15: reserved + packet counters (leave zero)
  buf[16] = 0x00;  // data_type[0] = 0 (point cloud data)
  buf[17] = 0x03;  // data_type[1] = 3 (dual return mode)

  // Fix #6: correct Airy timestamp encoding — separate usec (4B) and sec (6B)
  {
    uint64_t usec_total = static_cast<uint64_t>(pkt_ts_sec * 1'000'000.0);
    uint32_t usec = static_cast<uint32_t>(usec_total % 1'000'000ULL);
    uint64_t sec  = usec_total / 1'000'000ULL;

    // offset 18-21: uint32 microseconds within current second (little-endian)
    buf[18] = (usec >> 0)  & 0xFF;
    buf[19] = (usec >> 8)  & 0xFF;
    buf[20] = (usec >> 16) & 0xFF;
    buf[21] = (usec >> 24) & 0xFF;

    // offset 22-27: uint48 seconds since epoch (little-endian)
    buf[22] = (sec >> 0)  & 0xFF;
    buf[23] = (sec >> 8)  & 0xFF;
    buf[24] = (sec >> 16) & 0xFF;
    buf[25] = (sec >> 24) & 0xFF;
    buf[26] = (sec >> 32) & 0xFF;
    buf[27] = (sec >> 40) & 0xFF;
  }

  buf[28] = 0x0A;  // lidar_type (RSAIRY)
  buf[29] = MsopConstants::kLidarMode96;  // lidar_mode (96-line)
  // offset 30-41: reserved + temperature (leave zero)

  // =========================================================================
  // 8 Blocks (148 bytes each)
  // 96-line mode: even blocks → rings 0-47, odd blocks → rings 48-95
  // 2 consecutive blocks share the same azimuth
  // =========================================================================
  const AzimuthSample * az_samples[4] = {&az0, &az1, &az2, &az3};

  for (int blk = 0; blk < MsopConstants::kBlocksPerPkt; ++blk) {
    uint8_t * blk_buf = buf + MsopConstants::kHeaderSize +
                        blk * MsopConstants::kBlockSize;

    int az_idx = blk / 2;                              // 2 blocks per azimuth
    int base_ring = (blk % 2 == 0) ? 0 : 48;           // even: 0-47, odd: 48-95
    const auto & sample = *az_samples[az_idx];

    encodeBlock(blk_buf, toRawAzimuth(sample.azimuth_deg), base_ring, sample);
  }

  // =========================================================================
  // Tail (6 bytes) + reserved (16 bytes) — already zeroed
  // =========================================================================

  // =========================================================================
  // ROS 2 message metadata
  // =========================================================================
  pkt.header.stamp.sec = static_cast<int32_t>(std::floor(pkt_ts_sec));
  pkt.header.stamp.nanosec = static_cast<uint32_t>(
    (pkt_ts_sec - std::floor(pkt_ts_sec)) * 1'000'000'000.0);
  pkt.header.frame_id = robot_name_;
  pkt.is_difop = 0;
  pkt.is_frame_begin = (pkt_seq % MsopConstants::kPktsPerRevolution == 0) ? 1 : 0;
}

// ===========================================================================
// Encode a single MSOP block
// ===========================================================================
void AiryPacketBridge::encodeBlock(
  uint8_t * buf,
  uint16_t azimuth_raw,
  int base_ring,
  const AzimuthSample & sample)
{
  // Block magic
  std::memcpy(buf, MsopConstants::kBlockMagic, 2);

  // Azimuth (little-endian uint16, 0.01° units)
  buf[2] = azimuth_raw & 0xFF;
  buf[3] = (azimuth_raw >> 8) & 0xFF;

  // 48 channels × 3 bytes each
  for (int ch = 0; ch < MsopConstants::kChPerBlock; ++ch) {
    int ring = base_ring + ch;
    encodeChannel(buf + 4 + ch * MsopConstants::kChannelSize, ring, sample);
  }
}

// ===========================================================================
// Encode a single channel (3 bytes: 2B distance + 1B intensity)
// ===========================================================================
void AiryPacketBridge::encodeChannel(
  uint8_t * buf,
  int ring_id,
  const AzimuthSample & sample)
{
  if (sample.valid[ring_id]) {
    float dist = std::sqrt(
      sample.x[ring_id] * sample.x[ring_id] +
      sample.y[ring_id] * sample.y[ring_id] +
      sample.z[ring_id] * sample.z[ring_id]);

    uint16_t dist_raw = toRawDistance(dist);
    uint8_t intensity = 0;  // GPU LiDAR does not produce reflectance

    buf[0] = dist_raw & 0xFF;            // distance low byte
    buf[1] = ((dist_raw >> 8) & 0x3F);   // distance high 6 bits (feature bits = 0)
    buf[2] = intensity;                   // reflectance
  } else {
    buf[0] = 0x00;
    buf[1] = 0x00;
    buf[2] = 0x00;  // NAN point (zero distance)
  }
}

}  // namespace airy_packet_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(airy_packet_bridge::AiryPacketBridge)
