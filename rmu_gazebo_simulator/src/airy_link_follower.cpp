// Copyright 2026
// Minimal Gazebo System plugin: copies world pose from target link to follower link each step.
// This keeps the standalone Airy LiDAR sensor model locked to the robot.

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/gazebo/components/Pose.hh>

namespace airy
{

class AiryLinkFollower
  : public ignition::gazebo::System
  , public ignition::gazebo::ISystemConfigure
  , public ignition::gazebo::ISystemPostUpdate
{
public:
  AiryLinkFollower() = default;

  void Configure(
    const ignition::gazebo::Entity &,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager &) override
  {
    target_model_ = sdf->Get<std::string>("target_model", "red_standard_robot1").first;
    target_link_  = sdf->Get<std::string>("target_link", "front_mid360").first;
    follower_link_ = sdf->Get<std::string>("follower_link", "airy_lidar::airy_link").first;

    // Search ECS for follower entity (deferred — model may not be spawned yet)
    follower_found_ = false;
  }

  void PostUpdate(
    const ignition::gazebo::UpdateInfo &,
    const ignition::gazebo::EntityComponentManager & ecm) override
  {
    // Deferred lookup of follower entity
    if (!follower_found_) {
      ecm.Each<ignition::gazebo::components::Name,
               ignition::gazebo::components::ParentEntity>(
        [&](const ignition::gazebo::Entity & ent,
            const ignition::gazebo::components::Name * name,
            const ignition::gazebo::components::ParentEntity *) {
          if (name->Data() == follower_link_) {
            follower_entity_ = ent;
            follower_found_ = true;
            return false;
          }
          return true;
        });
      if (!follower_found_) return;
    }

    // Read target pose
    ignition::math::Pose3d target_pose;
    bool target_found = false;
    ecm.Each<ignition::gazebo::components::Name,
             ignition::gazebo::components::WorldPose>(
      [&](const ignition::gazebo::Entity &,
          const ignition::gazebo::components::Name * name,
          const ignition::gazebo::components::WorldPose * pose) {
        if (name->Data() == target_link_) {
          target_pose = pose->Data();
          target_found = true;
          return false;
        }
        return true;
      });

    if (!target_found) return;

    // Write follower pose via mutable ECS (use const_cast for read-only PostUpdate)
    auto * follower_pose =
      const_cast<ignition::gazebo::EntityComponentManager &>(ecm)
        .Component<ignition::gazebo::components::WorldPose>(follower_entity_);
    if (follower_pose) {
      *follower_pose = ignition::gazebo::components::WorldPose(target_pose);
    }
  }

private:
  std::string target_model_;
  std::string target_link_;
  std::string follower_link_;
  ignition::gazebo::Entity follower_entity_{ignition::gazebo::kNullEntity};
  bool follower_found_{false};
};

}  // namespace airy

IGN_COMMON_BEGIN_ADDING_PLUGINS
IGN_COMMON_ADD_PLUGIN(airy::AiryLinkFollower, ignition::gazebo::ISystemConfigure)
IGN_COMMON_ADD_PLUGIN(airy::AiryLinkFollower, ignition::gazebo::ISystemPostUpdate)
IGN_COMMON_FINISH_ADDING_PLUGINS
