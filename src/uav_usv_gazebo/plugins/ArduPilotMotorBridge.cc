#include <algorithm>
#include <array>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace uav_usv_gazebo
{
class ArduPilotMotorBridge final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    this->outputTopic = _sdf->Get<std::string>("output_topic");
    this->maxRotVelocity =
      _sdf->Get<double>("max_rot_velocity", 1000.0).first;
    this->commandScale =
      _sdf->Get<double>("command_scale", 1.0).first;
    this->publisher = this->node.Advertise<gz::msgs::Actuators>(
      this->outputTopic);
    for (std::size_t index = 0; index < this->commands.size(); ++index)
    {
      const auto key = "motor" + std::to_string(index) + "_topic";
      const auto topic = _sdf->Get<std::string>(key);
      this->node.Subscribe<gz::msgs::Double>(topic,
        std::function<void(const gz::msgs::Double &)>(
        [this, index](const gz::msgs::Double &_message)
        {
          std::lock_guard<std::mutex> guard(this->mutex);
          this->commands[index] = _message.data();
        }));
    }
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &) override
  {
    gz::msgs::Actuators message;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      // ArduPilotPlugin applies the control multiplier before publishing the
      // COMMAND topic, so the value is already rotor speed in rad/s.
      for (const double command : this->commands)
      {
        message.add_velocity(
          std::clamp(command * this->commandScale,
            0.0, this->maxRotVelocity));
      }
    }
    this->publisher.Publish(message);
  }

private:
  gz::transport::Node node;
  gz::transport::Node::Publisher publisher;
  std::string outputTopic;
  double maxRotVelocity{1000.0};
  double commandScale{1.0};
  std::array<double, 4> commands{{0.0, 0.0, 0.0, 0.0}};
  std::mutex mutex;
};
}

GZ_ADD_PLUGIN(
  uav_usv_gazebo::ArduPilotMotorBridge,
  gz::sim::System,
  uav_usv_gazebo::ArduPilotMotorBridge::ISystemConfigure,
  uav_usv_gazebo::ArduPilotMotorBridge::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  uav_usv_gazebo::ArduPilotMotorBridge,
  "uav_usv_gazebo::ArduPilotMotorBridge")
