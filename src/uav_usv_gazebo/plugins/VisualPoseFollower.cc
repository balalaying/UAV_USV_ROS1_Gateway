#include <memory>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>

#include <sdf/Element.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
class VisualPoseFollower
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &/*_ecm*/,
                         EventManager &/*_eventMgr*/) override
  {
    this->visualEntity = _entity;
    if (_sdf->HasElement("target_entity"))
      this->targetName = _sdf->Get<std::string>("target_entity");
    if (_sdf->HasElement("pose_offset"))
      this->poseOffset = _sdf->Get<math::Pose3d>("pose_offset");
  }

  public: void PreUpdate(const UpdateInfo &/*_info*/,
                         EntityComponentManager &_ecm) override
  {
    if (this->targetEntity == kNullEntity)
    {
      if (auto entity = _ecm.EntityByName(this->targetName))
        this->targetEntity = *entity;
    }

    if (this->targetEntity == kNullEntity ||
        this->visualEntity == kNullEntity)
      return;

    const Model targetModel(this->targetEntity);
    const Entity targetLink = targetModel.CanonicalLink(_ecm);
    if (targetLink == kNullEntity)
      return;

    const auto linkPose = _ecm.Component<components::Pose>(targetLink);
    if (!linkPose)
      return;

    // Recover the model origin from the moving canonical link. PX4 models may
    // keep the model Pose component at its spawn value while physics updates
    // the canonical link.
    const math::Pose3d targetPose = worldPose(targetLink, _ecm) *
        linkPose->Data().Inverse() * this->poseOffset;
    // The shell is a static, visual-only top-level model. Updating its Pose
    // component keeps the server scene graph and Gazebo GUI renderer in sync;
    // WorldPoseCmd alone can update pose/info while leaving a static visual at
    // its spawn position on some Gazebo Sim 8 GUI backends.
    if (_ecm.Component<components::Pose>(this->visualEntity))
    {
      _ecm.SetComponentData<components::Pose>(
          this->visualEntity, targetPose);
    }
    else
    {
      _ecm.CreateComponent(
          this->visualEntity, components::Pose(targetPose));
    }

    if (_ecm.Component<components::WorldPoseCmd>(this->visualEntity))
    {
      _ecm.SetComponentData<components::WorldPoseCmd>(
          this->visualEntity, targetPose);
    }
    else
    {
      _ecm.CreateComponent(
          this->visualEntity, components::WorldPoseCmd(targetPose));
    }
  }

  private: Entity visualEntity{kNullEntity};
  private: Entity targetEntity{kNullEntity};
  private: std::string targetName{"uav_01"};
  private: math::Pose3d poseOffset{0, 0, 0, 0, 0, 0};
};
}
}
}
}

GZ_ADD_PLUGIN(gz::sim::systems::VisualPoseFollower,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::VisualPoseFollower,
                    "gz::sim::systems::VisualPoseFollower")
