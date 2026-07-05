#include <chrono>
#include <atomic>
#include <algorithm>
#include <memory>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
class DroneDeckFollower
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  public: void Configure(const Entity &/*_entity*/,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &/*_ecm*/,
                         EventManager &/*_eventMgr*/) override
  {
    if (_sdf->HasElement("boat_name"))
      this->boatName = _sdf->Get<std::string>("boat_name");
    if (_sdf->HasElement("drone_name"))
      this->droneName = _sdf->Get<std::string>("drone_name");
    if (_sdf->HasElement("deck_offset"))
      this->deckOffset = _sdf->Get<math::Pose3d>("deck_offset");
    if (_sdf->HasElement("release_vertical_speed"))
      this->releaseVerticalSpeed = _sdf->Get<double>("release_vertical_speed");
    if (_sdf->HasElement("release_after"))
      this->releaseAfter = _sdf->Get<double>("release_after");
    if (_sdf->HasElement("release_topic"))
      this->releaseTopic = _sdf->Get<std::string>("release_topic");
    if (_sdf->HasElement("relock_duration"))
      this->relockDuration = _sdf->Get<double>("relock_duration");
    if (_sdf->HasElement("initially_released"))
      this->released = _sdf->Get<bool>("initially_released");

    this->node.Subscribe(this->releaseTopic, &DroneDeckFollower::OnRelease,
        this);
  }

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    if (this->releaseRequested.load())
    {
      this->released = true;
      this->lockRequested.store(false);
      this->relocking = false;
      return;
    }

    if (this->boatEntity == kNullEntity)
    {
      if (auto entity = _ecm.EntityByName(this->boatName))
        this->boatEntity = *entity;
    }

    if (this->droneEntity == kNullEntity)
    {
      if (auto entity = _ecm.EntityByName(this->droneName))
        this->droneEntity = *entity;
    }

    if (this->boatEntity == kNullEntity || this->droneEntity == kNullEntity)
      return;

    const double simTime =
        std::chrono::duration<double>(_info.simTime).count();
    if (this->released)
    {
      if (!this->lockRequested.load())
        return;

      auto dronePoseComp = _ecm.Component<components::Pose>(this->droneEntity);
      if (dronePoseComp)
        this->relockStartPose = dronePoseComp->Data();
      this->relockStartTime = simTime;
      this->relocking = true;
      this->released = false;
      this->lockRequested.store(false);
    }

    if (simTime > this->releaseAfter)
    {
      this->released = true;
      return;
    }

    auto droneVel = _ecm.Component<components::WorldLinearVelocity>(
        this->droneEntity);
    if (droneVel && droneVel->Data().Z() > this->releaseVerticalSpeed)
    {
      this->released = true;
      return;
    }

    auto boatPoseComp = _ecm.Component<components::Pose>(this->boatEntity);
    if (!boatPoseComp)
      return;

    const auto boatPose = boatPoseComp->Data();
    math::Pose3d target;
    target.Pos() = boatPose.Pos() +
        boatPose.Rot().RotateVector(this->deckOffset.Pos());
    target.Rot() = boatPose.Rot() * this->deckOffset.Rot();

    if (this->relocking)
    {
      const double elapsed = simTime - this->relockStartTime;
      const double alpha = this->relockDuration <= 0.0 ? 1.0 :
          std::min(1.0, std::max(0.0, elapsed / this->relockDuration));
      target.Pos() = this->relockStartPose.Pos() * (1.0 - alpha) +
          target.Pos() * alpha;

      if (alpha >= 1.0)
        this->relocking = false;
    }

    if (auto cmd = _ecm.Component<components::WorldPoseCmd>(
        this->droneEntity))
    {
      _ecm.SetComponentData<components::WorldPoseCmd>(this->droneEntity,
          target);
    }
    else
    {
      _ecm.CreateComponent(this->droneEntity,
          components::WorldPoseCmd(target));
    }
  }

  private: void OnRelease(const gz::msgs::Boolean &_msg)
  {
    if (_msg.data())
    {
      this->releaseRequested.store(true);
      this->lockRequested.store(false);
    }
    else
    {
      this->releaseRequested.store(false);
      this->lockRequested.store(true);
    }
  }

  private: std::string boatName{"landing_boat"};
  private: std::string droneName{"x500_0"};
  private: std::string releaseTopic{"/model/x500_0/release_from_deck"};
  private: math::Pose3d deckOffset{-0.92, 0, 0.43, 0, 0, 0};
  private: double releaseVerticalSpeed{0.9};
  private: double releaseAfter{20.0};
  private: double relockDuration{2.5};
  private: Entity boatEntity{kNullEntity};
  private: Entity droneEntity{kNullEntity};
  private: gz::transport::Node node;
  private: std::atomic_bool releaseRequested{false};
  private: std::atomic_bool lockRequested{false};
  private: math::Pose3d relockStartPose{0, 0, 0, 0, 0, 0};
  private: double relockStartTime{0.0};
  private: bool relocking{false};
  private: bool released{false};
};
}
}
}
}

GZ_ADD_PLUGIN(gz::sim::systems::DroneDeckFollower,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::DroneDeckFollower,
                    "gz::sim::systems::DroneDeckFollower")
