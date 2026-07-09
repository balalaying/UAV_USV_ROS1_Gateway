#include <chrono>
#include <cmath>
#include <memory>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
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
class BoatWaveFollower
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventMgr*/) override
  {
    this->entity = _entity;

    if (auto poseComp = _ecm.Component<components::Pose>(this->entity))
    {
      this->basePose = poseComp->Data();
    }
    else
    {
      _ecm.CreateComponent(this->entity, components::Pose(this->basePose));
    }

    this->meanZ = this->basePose.Pos().Z();

    if (_sdf->HasElement("mean_z"))
      this->meanZ = _sdf->Get<double>("mean_z");
    if (_sdf->HasElement("heave_amplitude"))
      this->heaveAmplitude = _sdf->Get<double>("heave_amplitude");
    if (_sdf->HasElement("roll_amplitude"))
      this->rollAmplitude = _sdf->Get<double>("roll_amplitude");
    if (_sdf->HasElement("pitch_amplitude"))
      this->pitchAmplitude = _sdf->Get<double>("pitch_amplitude");
    if (_sdf->HasElement("primary_frequency"))
      this->primaryFrequency = _sdf->Get<double>("primary_frequency");
    if (_sdf->HasElement("secondary_frequency"))
      this->secondaryFrequency = _sdf->Get<double>("secondary_frequency");
    if (_sdf->HasElement("wave_number_x"))
      this->waveNumberX = _sdf->Get<double>("wave_number_x");
    if (_sdf->HasElement("wave_number_y"))
      this->waveNumberY = _sdf->Get<double>("wave_number_y");
  }

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    auto poseComp = _ecm.Component<components::Pose>(this->entity);
    if (!poseComp)
      return;

    auto pose = poseComp->Data();
    const double t =
        std::chrono::duration<double>(_info.simTime).count();
    const double x = pose.Pos().X();
    const double y = pose.Pos().Y();
    const double yaw = pose.Rot().Yaw();

    const double phaseA =
        this->primaryFrequency * t + this->waveNumberX * x +
        0.35 * this->waveNumberY * y;
    const double phaseB =
        this->secondaryFrequency * t - 0.55 * this->waveNumberX * x +
        this->waveNumberY * y + 1.2;

    const double heave =
        this->heaveAmplitude *
        (0.7 * std::sin(phaseA) + 0.3 * std::sin(phaseB));
    const double roll =
        this->rollAmplitude *
        (0.65 * std::sin(phaseB) + 0.35 * std::sin(phaseA + 0.8));
    const double pitch =
        this->pitchAmplitude *
        (0.7 * std::cos(phaseA) + 0.3 * std::sin(phaseB - 0.4));

    pose.Pos().Z(this->meanZ + heave);
    pose.Rot() = math::Quaterniond(roll, pitch, yaw);

    auto cmdComp = _ecm.Component<components::WorldPoseCmd>(this->entity);
    if (cmdComp)
    {
      _ecm.SetComponentData<components::WorldPoseCmd>(this->entity, pose);
    }
    else
    {
      _ecm.CreateComponent(this->entity, components::WorldPoseCmd(pose));
    }
  }

  private: Entity entity{kNullEntity};
  private: math::Pose3d basePose{0, 0, 0, 0, 0, 0};
  private: double meanZ{0.42};
  private: double heaveAmplitude{0.1};
  private: double rollAmplitude{0.055};
  private: double pitchAmplitude{0.045};
  private: double primaryFrequency{1.1};
  private: double secondaryFrequency{1.7};
  private: double waveNumberX{0.35};
  private: double waveNumberY{0.22};
};
}
}
}
}

GZ_ADD_PLUGIN(gz::sim::systems::BoatWaveFollower,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::BoatWaveFollower,
                    "gz::sim::systems::BoatWaveFollower")
