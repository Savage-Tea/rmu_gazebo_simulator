// Copyright 2026 SavageTea
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "airy_packet_bridge/airy_packet_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace airy_packet_bridge
{

AiryPacketBridge::AiryPacketBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("airy_packet_bridge", options)
{
  robot_name_ = this->declare_parameter("robot_name", "red_standard_robot1");
  cloud_topic_ = this->declare_parameter("cloud_topic", "airy/lidar");
  imu_topic_ = this->declare_parameter("imu_topic", "livox/imu");

  packet_pub_ = this->create_publisher<rslidar_msg::msg::RslidarPacket>(
    "/lidar/packets", rclcpp::SensorDataQoS());
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/lidar/imu", rclcpp::SensorDataQoS());

  // Subscribe to real Gazebo IMU (orientation + angular velocity)
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
      last_imu_ = *msg;
    });

  // Subscribe to GT odometry: compute a=dv/dt and inject into synthetic IMU
  gt_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "chassis_odometry_gt", rclcpp::SensorDataQoS(),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      auto now = this->now();
      double dt = (now - last_gt_time_).seconds();
      if (last_gt_time_.nanoseconds() > 0 && dt > 0.0001 && dt < 1.0) {
        // a = dv/dt from GT odometry velocity change
        double ax = (msg->twist.twist.linear.x - last_gt_vx_) / dt;
        double ay = (msg->twist.twist.linear.y - last_gt_vy_) / dt;
        double az = (msg->twist.twist.linear.z - last_gt_vz_) / dt;
        // Only emit when non-zero acceleration detected
        if (std::abs(ax) > 0.01 || std::abs(ay) > 0.01 || std::abs(az) > 0.01) {
          sensor_msgs::msg::Imu synth = last_imu_;
          synth.header.stamp = now;
          synth.linear_acceleration.x += ax;
          synth.linear_acceleration.y += ay;
          synth.linear_acceleration.z += az;
          imu_pub_->publish(synth);
        }
      }
      // Always publish IMU with gravity (for continuous state prediction)
      sensor_msgs::msg::Imu grav = last_imu_;
      grav.header.stamp = now;
      imu_pub_->publish(grav);
      last_gt_time_ = now;
      last_gt_vx_ = msg->twist.twist.linear.x;
      last_gt_vy_ = msg->twist.twist.linear.y;
      last_gt_vz_ = msg->twist.twist.linear.z;
    });

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AiryPacketBridge::onPointCloud, this, std::placeholders::_1));

  // Timer: ~2250 Hz = 444us per tick → 1 azimuth block per tick
  pkt_timer_ = this->create_wall_timer(
    std::chrono::microseconds(444),
    std::bind(&AiryPacketBridge::onTimer, this));

  RCLCPP_INFO(this->get_logger(),
    "AiryPacketBridge ready. robot=%s, cloud=%s, imu=%s, pkt_timer=444us",
    robot_name_.c_str(), cloud_topic_.c_str(), imu_topic_.c_str());
}

// ===========================================================================
// PointCloud2 callback — just buffer the latest frame
// ===========================================================================
void AiryPacketBridge::onPointCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Drop frame if still processing previous one
  if (cloud_fresh_) return;

  uint32_t w = msg->width;
  uint32_t h = msg->height;
  const size_t total_pts = w * h;

  cloud_xs_.resize(total_pts);
  cloud_ys_.resize(total_pts);
  cloud_zs_.resize(total_pts);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (size_t i = 0; i < total_pts; ++i, ++iter_x, ++iter_y, ++iter_z) {
    cloud_xs_[i] = *iter_x;
    cloud_ys_[i] = *iter_y;
    cloud_zs_[i] = *iter_z;
  }

  cloud_w_ = w;
  cloud_h_ = h;
  cloud_ts_ = static_cast<double>(msg->header.stamp.sec) +
              static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
  cloud_col_ = 0;
  cloud_fresh_ = true;
}

// ===========================================================================
// Timer callback — extract one 4-azimuth block, build one MSOP packet
// ===========================================================================
void AiryPacketBridge::onTimer()
{
  if (!cloud_fresh_ || cloud_w_ == 0) return;
  uint32_t w = cloud_w_;
  uint32_t h = cloud_h_;
  uint32_t col = cloud_col_;

  AzimuthSample samples[4];
  bool all_valid = true;
  for (int a = 0; a < 4; ++a) {
    uint32_t c = col + a;
    if (c >= w) { all_valid = false; break; }
    AzimuthSample & s = samples[a];
    // Per-column timestamp: each column advances 0.4° = 111μs
    s.timestamp_sec = cloud_ts_ + c * 0.00011108;
    double sum_az = 0.0; int az_count = 0;

    for (uint32_t ring = 0; ring < h && ring < 96; ++ring) {
      size_t idx = ring * w + c;
      float px = cloud_xs_[idx];
      float py = cloud_ys_[idx];
      float pz = cloud_zs_[idx];
      if (std::isfinite(px) && std::isfinite(py) && std::isfinite(pz)) {
        s.x[ring] = px; s.y[ring] = py; s.z[ring] = pz;
        s.valid[ring] = true;
        sum_az += std::atan2(-py, px) * 180.0 / M_PI;
        az_count++;
      }
    }
    if (az_count == 0) { all_valid = false; break; }
    s.azimuth_deg = sum_az / az_count;
  }

  if (all_valid) {
    if (last_azimuth_valid_) {
      double gap = std::abs(samples[0].azimuth_deg - last_azimuth_deg_);
      if (gap > 5.0 && gap < 355.0) { ring_idx_ = 0; }
    }
    last_azimuth_deg_ = samples[0].azimuth_deg;
    last_azimuth_valid_ = true;

    for (int a = 0; a < 4; ++a) {
      ring_[ring_idx_ % 4] = samples[a];
      ring_idx_++;
    }

    if (ring_idx_ % 4 == 0 && ring_idx_ > 0) {
      rslidar_msg::msg::RslidarPacket pkt;
      double pkt_ts = ring_[0].timestamp_sec;  // use first sample's ts
      buildMsopPacket(ring_[0], ring_[1], ring_[2], ring_[3],
                      pkt_ts, pkt_seq_++, pkt);
      packet_pub_->publish(pkt);
    }
  }

  cloud_col_ += 4;
  if (cloud_col_ >= w) cloud_fresh_ = false;  // wait for next frame
}

// ===========================================================================
// buildMsopPacket → UNCHANGED
// ===========================================================================
void AiryPacketBridge::buildMsopPacket(
  const AzimuthSample & az0, const AzimuthSample & az1,
  const AzimuthSample & az2, const AzimuthSample & az3,
  double pkt_ts_sec, uint32_t pkt_seq,
  rslidar_msg::msg::RslidarPacket & pkt)
{
  pkt.data.resize(MsopConstants::kPktSize);
  uint8_t * buf = pkt.data.data();
  std::memset(buf, 0, MsopConstants::kPktSize);

  std::memcpy(buf + 0, MsopConstants::kMsopMagic, 4);
  buf[16] = 0x00; buf[17] = 0x03;

  {
    uint64_t usec_total = static_cast<uint64_t>(pkt_ts_sec * 1'000'000.0);
    uint32_t usec = static_cast<uint32_t>(usec_total % 1'000'000ULL);
    uint64_t sec  = usec_total / 1'000'000ULL;
    buf[18] = (usec >> 0)  & 0xFF; buf[19] = (usec >> 8)  & 0xFF;
    buf[20] = (usec >> 16) & 0xFF; buf[21] = (usec >> 24) & 0xFF;
    buf[22] = (sec >> 0)  & 0xFF;  buf[23] = (sec >> 8)  & 0xFF;
    buf[24] = (sec >> 16) & 0xFF;  buf[25] = (sec >> 24) & 0xFF;
    buf[26] = (sec >> 32) & 0xFF;  buf[27] = (sec >> 40) & 0xFF;
  }

  buf[28] = 0x0A; buf[29] = MsopConstants::kLidarMode96;

  const AzimuthSample * az_samples[4] = {&az0, &az1, &az2, &az3};
  for (int blk = 0; blk < MsopConstants::kBlocksPerPkt; ++blk) {
    uint8_t * blk_buf = buf + MsopConstants::kHeaderSize +
                        blk * MsopConstants::kBlockSize;
    int az_idx = blk / 2;
    int base_ring = (blk % 2 == 0) ? 0 : 48;
    encodeBlock(blk_buf, toRawAzimuth(az_samples[az_idx]->azimuth_deg),
                base_ring, *az_samples[az_idx]);
  }

  pkt.header.stamp.sec = static_cast<int32_t>(std::floor(pkt_ts_sec));
  pkt.header.stamp.nanosec = static_cast<uint32_t>(
    (pkt_ts_sec - std::floor(pkt_ts_sec)) * 1'000'000'000.0);
  pkt.header.frame_id = robot_name_;
  pkt.is_difop = 0;
  pkt.is_frame_begin = (pkt_seq % MsopConstants::kPktsPerRevolution == 0) ? 1 : 0;
}

void AiryPacketBridge::encodeBlock(
  uint8_t * buf, uint16_t azimuth_raw, int base_ring,
  const AzimuthSample & sample)
{
  std::memcpy(buf, MsopConstants::kBlockMagic, 2);
  buf[2] = azimuth_raw & 0xFF;
  buf[3] = (azimuth_raw >> 8) & 0xFF;
  for (int ch = 0; ch < MsopConstants::kChPerBlock; ++ch) {
    encodeChannel(buf + 4 + ch * MsopConstants::kChannelSize, base_ring + ch, sample);
  }
}

void AiryPacketBridge::encodeChannel(
  uint8_t * buf, int ring_id, const AzimuthSample & sample)
{
  if (sample.valid[ring_id]) {
    float dist = std::sqrt(
      sample.x[ring_id] * sample.x[ring_id] +
      sample.y[ring_id] * sample.y[ring_id] +
      sample.z[ring_id] * sample.z[ring_id]);
    uint16_t dist_raw = toRawDistance(dist);
    buf[0] = dist_raw & 0xFF;
    buf[1] = ((dist_raw >> 8) & 0x3F);
    buf[2] = 0;
  } else {
    buf[0] = 0x00; buf[1] = 0x00; buf[2] = 0x00;
  }
}

}  // namespace airy_packet_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(airy_packet_bridge::AiryPacketBridge)
