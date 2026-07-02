// Copyright 2026 SavageTea
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AIRY_GAZEBO_PLUGIN__AIRY_GPU_LIDAR_HPP_
#define AIRY_GAZEBO_PLUGIN__AIRY_GPU_LIDAR_HPP_

#include <ignition/gazebo/System.hh>
#include <ignition/rendering/GpuRays.hh>
#include <ignition/transport/Node.hh>

#include <memory>
#include <string>

namespace airy_gazebo_plugin
{

/// @brief Airy 96-line GPU LiDAR Gazebo System Plugin.
///
/// Renders 96 vertical rays at a single horizontal angle per simulation step,
/// simulating the Airy mechanical LiDAR's block-level scan pattern at ~2250 blocks/sec.
///
/// Output: ignition::msgs::PointCloudPacked on Gazebo Transport
///   /world/default/model/{robot}/link/{link}/sensor/airy_lidar/scan/points
class AiryGpuLidarPlugin : public ignition::gazebo::System,
                           public ignition::gazebo::ISystemConfigure,
                           public ignition::gazebo::ISystemPostUpdate
{
public:
  AiryGpuLidarPlugin() = default;
  ~AiryGpuLidarPlugin() override = default;

  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager & eventMgr) override;

  void PostUpdate(
    const ignition::gazebo::UpdateInfo & info,
    const ignition::gazebo::EntityComponentManager & ecm) override;

private:
  /// @brief Render a single block (96 rays at one azimuth) and publish.
  void renderBlock(
    double azimuth_rad, const ignition::math::Pose3d & lidar_pose);

  /// @brief Convert spherical (az, el, range) to Cartesian (x,y,z).
  static void sphericalToCartesian(
    double azimuth_rad, const double * vertical_rad, const double * ranges,
    int num_rays, double min_range, double max_range,
    float * xyz_out);

  // ===========================================================================
  // Airy 96-line constants
  // ===========================================================================
  static constexpr int kNumLines = 96;
  static constexpr double kRpm = 600.0;
  static constexpr double kDegPerSec = kRpm / 60.0 * 360.0;  // 3600°/s
  static constexpr double kBlockDurationUs = 111.080;         // μs
  static constexpr double kAzimuthPerBlock =
    kRpm / 60.0 * 360.0 * kBlockDurationUs * 1e-6;           // ~0.4°

  // ===========================================================================
  // State
  // ===========================================================================
  ignition::gazebo::Entity lidar_entity_{ignition::gazebo::kNullEntity};
  ignition::rendering::GpuRaysPtr gpu_rays_;
  ignition::transport::Node::Publisher pub_;

  std::string robot_name_;
  std::string lidar_link_;
  double max_range_{60.0};
  double min_range_{0.1};

  double azimuth_accumulator_{0.0};
  double current_azimuth_deg_{-180.0};
  double vertical_angles_rad_[kNumLines];

  bool initialized_{false};
};

}  // namespace airy_gazebo_plugin

#endif  // AIRY_GAZEBO_PLUGIN__AIRY_GPU_LIDAR_HPP_
