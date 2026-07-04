// Copyright 2026 SavageTea
//
// Licensed under the Apache License, Version 2.0 (the "License");

#include "airy_gazebo_plugin/airy_gpu_lidar.hpp"

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include <cmath>
#include <cstring>

using namespace ignition::gazebo;

namespace airy_gazebo_plugin
{

void AiryGpuLidarPlugin::Configure(
  const Entity & /*entity*/,
  const std::shared_ptr<const sdf::Element> & sdf,
  EntityComponentManager & ecm,
  EventManager & /*eventMgr*/)
{
  // =========================================================================
  // Read SDF parameters
  // =========================================================================
  robot_name_ = sdf->Get<std::string>("robot_name", "red_standard_robot1").first;
  lidar_link_ = sdf->Get<std::string>("lidar_link", "front_mid360").first;
  max_range_ = sdf->Get<double>("max_range", 60.0).first;
  min_range_ = sdf->Get<double>("min_range", 0.1).first;

  // =========================================================================
  // Compute 96 vertical angles (-45° to +45°, evenly spaced)
  // Ring 0 = bottom (-45°), Ring 95 = top (+45°)
  // =========================================================================
  for (int i = 0; i < kNumLines; ++i) {
    double deg = -45.0 + static_cast<double>(i) * 90.0 / static_cast<double>(kNumLines - 1);
    vertical_angles_rad_[i] = deg * M_PI / 180.0;
  }

  // =========================================================================
  // Get Ogre2 rendering engine and create GpuRays sensor
  // =========================================================================
  auto *engine = ignition::rendering::engine("ogre2");
  if (!engine) {
    for (unsigned int i = 0; i < ignition::rendering::engineCount(); ++i) {
      engine = ignition::rendering::engine(i);
      if (engine) break;
    }
  }

  if (engine) {
    // Scene name is "scene" when created by the Sensors system
    auto scene = engine->SceneByName("scene");
    if (scene) {
      gpu_rays_ = scene->CreateGpuRays("airy_lidar_sensor");
      gpu_rays_->SetVerticalRayCount(kNumLines);
      gpu_rays_->SetVerticalAngles(vertical_angles_rad_);
      gpu_rays_->SetHorizontalRayCount(1);
      gpu_rays_->SetMinRange(min_range_);
      gpu_rays_->SetMaxRange(max_range_);
      rendering_ready_ = true;
    }
  }

  // =========================================================================
  // Gazebo Transport publisher (Fix #2: persistent Node prevents dangling handle)
  // =========================================================================
  ign_node_ = std::make_shared<ignition::transport::Node>();
  std::string topic = "/world/default/model/" + robot_name_ +
    "/link/" + lidar_link_ + "/sensor/airy_lidar/scan/points";
  pub_ = ign_node_->Advertise<ignition::msgs::PointCloudPacked>(topic);

  // =========================================================================
  // Store entity name for deferred lookup in PostUpdate (Fix #1)
  // The robot is spawned dynamically after Configure, so the entity may not
  // exist yet.  We search for it at the start of every PostUpdate until found.
  // =========================================================================
  full_entity_name_ = robot_name_ + "::" + lidar_link_;
  rendering_ready_ = (gpu_rays_ != nullptr);
}

void AiryGpuLidarPlugin::PostUpdate(
  const UpdateInfo & info,
  const EntityComponentManager & ecm)
{
  if (!rendering_ready_) return;

  // =========================================================================
  // Fix #1: Deferred entity lookup — robot may be spawned after Configure
  // =========================================================================
  if (!lidar_entity_found_) {
    ecm.Each<components::Name, components::ParentEntity>(
      [&](const Entity & ent, const components::Name * name,
          const components::ParentEntity *) {
        if (name->Data() == full_entity_name_) {
          lidar_entity_ = ent;
          lidar_entity_found_ = true;
          return false;  // stop iteration
        }
        return true;
      });
    if (!lidar_entity_found_) return;  // not spawned yet, try again next step
  }

  // =========================================================================
  // Read LiDAR world pose from ECS
  // =========================================================================
  auto * pose_comp = ecm.Component<components::WorldPose>(lidar_entity_);
  if (!pose_comp) return;

  // =========================================================================
  // Accumulate azimuth rotation
  // =========================================================================
  double dt = std::chrono::duration<double>(info.dt).count();
  azimuth_accumulator_ += kDegPerSec * dt;

  // =========================================================================
  // Render all blocks whose azimuth boundaries have been crossed
  // =========================================================================
  while (azimuth_accumulator_ >= kAzimuthPerBlock) {
    azimuth_accumulator_ -= kAzimuthPerBlock;
    current_azimuth_deg_ += kAzimuthPerBlock;
    if (current_azimuth_deg_ >= 180.0) current_azimuth_deg_ -= 360.0;

    renderBlock(current_azimuth_deg_ * M_PI / 180.0, pose_comp->Data());
  }
}

void AiryGpuLidarPlugin::renderBlock(
  double azimuth_rad, const ignition::math::Pose3d & lidar_pose)
{
  // Set sensor pose and horizontal direction
  gpu_rays_->SetWorldPose(lidar_pose);
  gpu_rays_->SetHorizontalAngle(azimuth_rad);

  // Trigger GPU ray casting (blocking for the small 96-ray batch)
  gpu_rays_->Update();

  // Retrieve ranges (96 floats)
  const auto & ranges = gpu_rays_->Ranges();

  // Fix #8: ensure Ranges() returned the expected number of elements
  if (ranges.size() < static_cast<size_t>(kNumLines)) {
    return;  // sensor not fully configured yet
  }

  // Convert spherical to Cartesian
  float xyz[kNumLines * 3];
  sphericalToCartesian(
    azimuth_rad, vertical_angles_rad_, ranges.data(),
    kNumLines, min_range_, max_range_, xyz);

  // Build PointCloudPacked message
  ignition::msgs::PointCloudPacked msg;
  msg.mutable_header()->mutable_stamp()->set_sec(
    static_cast<int64_t>(gpu_rays_->SimTime().count()) / 1'000'000'000);
  msg.mutable_header()->mutable_stamp()->set_nsec(
    static_cast<int32_t>(gpu_rays_->SimTime().count() % 1'000'000'000));
  msg.set_field("xyz");
  msg.set_data(xyz, kNumLines * 3 * sizeof(float));
  msg.set_width(kNumLines);
  msg.set_height(1);
  msg.set_is_dense(false);

  pub_.Publish(msg);
}

void AiryGpuLidarPlugin::sphericalToCartesian(
  double azimuth_rad, const double * vertical_rad, const double * ranges,
  int num_rays, double min_range, double max_range,
  float * xyz_out)
{
  for (int i = 0; i < num_rays; ++i) {
    float * p = xyz_out + i * 3;
    double range = ranges[i];

    if (range < min_range || range > max_range || !std::isfinite(range)) {
      p[0] = NAN; p[1] = NAN; p[2] = NAN;
    } else {
      double cos_v = std::cos(vertical_rad[i]);
      p[0] = static_cast<float>(range * cos_v * std::cos(azimuth_rad));
      p[1] = static_cast<float>(-range * cos_v * std::sin(azimuth_rad));
      p[2] = static_cast<float>(range * std::sin(vertical_rad[i]));
    }
  }
}

}  // namespace airy_gazebo_plugin

IGNITION_ADD_PLUGIN(
  airy_gazebo_plugin::AiryGpuLidarPlugin,
  ignition::gazebo::System,
  airy_gazebo_plugin::AiryGpuLidarPlugin::ISystemConfigure,
  airy_gazebo_plugin::AiryGpuLidarPlugin::ISystemPostUpdate)
