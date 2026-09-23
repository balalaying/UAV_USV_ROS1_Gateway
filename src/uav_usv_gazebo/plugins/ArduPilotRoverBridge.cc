#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace uav_usv_gazebo
{

class ArduPilotRoverBridge final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &) override
  {
    this->maxSpeed =
      _sdf->Get<double>("max_speed", 3.0).first;

    this->maxYawRate =
      _sdf->Get<double>("max_yaw_rate", 0.8).first;

    this->maxLinearAccel =
      _sdf->Get<double>("max_linear_accel", 1.0).first;

    this->maxYawAccel =
      _sdf->Get<double>("max_yaw_accel", 0.8).first;

    const auto driveMode =
      _sdf->Get<std::string>("drive_mode", "velocity").first;

    this->forceMode = (driveMode == "force");

    const auto leftTopic =
      _sdf->Get<std::string>("left_topic");

    const auto rightTopic =
      _sdf->Get<std::string>("right_topic");

    const auto outputTopic =
      _sdf->Get<std::string>("output_topic");

    if (this->forceMode)
    {
      this->maxThrustPerSide =
        _sdf->Get<double>("max_thrust_per_side", 75.0).first;

      this->thrusterY =
        _sdf->Get<double>("thruster_y", 1.2).first;

      this->surgeDrag =
        _sdf->Get<double>("surge_drag", 50.0).first;

      this->swayDrag =
        _sdf->Get<double>("sway_drag", 100.0).first;

      this->yawDrag =
        _sdf->Get<double>("yaw_drag", 80.0).first;

      const auto linkName =
        _sdf->Get<std::string>("link_name", "hull").first;

      gz::sim::Model model(_entity);

      // LinkByName only searches immediate child links. The physical
      // boat hull lives inside an included / nested model, so search
      // recursively through the model tree.
      std::function<gz::sim::Entity(const gz::sim::Model &)>
        findLinkRecursive;

      findLinkRecursive =
        [&](const gz::sim::Model &_model) -> gz::sim::Entity
        {
          const auto directLink =
            _model.LinkByName(_ecm, linkName);

          if (directLink != gz::sim::kNullEntity)
          {
            return directLink;
          }

          for (const auto nestedEntity : _model.Models(_ecm))
          {
            gz::sim::Model nestedModel(nestedEntity);

            const auto nestedLink =
              findLinkRecursive(nestedModel);

            if (nestedLink != gz::sim::kNullEntity)
            {
              return nestedLink;
            }
          }

          return gz::sim::kNullEntity;
        };

      const auto linkEntity =
        findLinkRecursive(model);

      if (linkEntity == gz::sim::kNullEntity)
      {
        throw std::runtime_error(
          "ArduPilotRoverBridge: physical link not found: " +
          linkName);
      }

      this->link.ResetEntity(linkEntity);

      // Needed for drag terms based on actual physics velocity.
      this->link.EnableVelocityChecks(_ecm, true);
    }
    else
    {
      this->publisher =
        this->node.Advertise<gz::msgs::Twist>(outputTopic);
    }

    this->node.Subscribe<gz::msgs::Double>(
      leftTopic,
      std::function<void(const gz::msgs::Double &)>(
        [this](const gz::msgs::Double &_message)
        {
          std::lock_guard<std::mutex> guard(this->mutex);

          this->left =
            2.0 * _message.data() - 1.0;
        }));

    this->node.Subscribe<gz::msgs::Double>(
      rightTopic,
      std::function<void(const gz::msgs::Double &)>(
        [this](const gz::msgs::Double &_message)
        {
          std::lock_guard<std::mutex> guard(this->mutex);

          this->right =
            2.0 * _message.data() - 1.0;
        }));
  }


  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
    {
      return;
    }

    double leftCommand;
    double rightCommand;

    {
      std::lock_guard<std::mutex> guard(this->mutex);

      leftCommand =
        std::clamp(this->left, -1.0, 1.0);

      rightCommand =
        std::clamp(this->right, -1.0, 1.0);
    }

    // -------------------------------------------------------
    // Legacy mode: preserve old behaviour for usv_02/usv_03.
    // -------------------------------------------------------
    if (!this->forceMode)
    {
      const double targetLinear =
        0.5 *
        (leftCommand + rightCommand) *
        this->maxSpeed;

      const double targetYaw =
        0.5 *
        (rightCommand - leftCommand) *
        this->maxYawRate;

      const double dt =
        std::chrono::duration<double>(_info.dt).count();

      if (dt > 0.0)
      {
        const double maxLinearStep =
          this->maxLinearAccel * dt;

        const double maxYawStep =
          this->maxYawAccel * dt;

        this->commandLinear +=
          std::clamp(
            targetLinear - this->commandLinear,
            -maxLinearStep,
            maxLinearStep);

        this->commandYaw +=
          std::clamp(
            targetYaw - this->commandYaw,
            -maxYawStep,
            maxYawStep);
      }

      gz::msgs::Twist message;

      message.mutable_linear()->set_x(
        this->commandLinear);

      message.mutable_angular()->set_z(
        this->commandYaw);

      this->publisher.Publish(message);

      return;
    }

    // -------------------------------------------------------
    // Force mode: physical differential thrust on hull.
    // -------------------------------------------------------

    const auto pose =
      this->link.WorldPose(_ecm);

    if (!pose)
    {
      return;
    }

    const auto rotation = pose->Rot();

    // Vehicle body +X is forward.
    const auto forwardWorld =
      rotation.RotateVector(
        gz::math::Vector3d(1.0, 0.0, 0.0));

    const double leftThrust =
      leftCommand * this->maxThrustPerSide;

    const double rightThrust =
      rightCommand * this->maxThrustPerSide;

    // A stronger left thruster turns the boat clockwise (negative Gazebo ENU
    // yaw), while a stronger right thruster turns it counter-clockwise.
    // Keep this consistent with the velocity-mode mapping above.
    this->link.AddWorldForce(
      _ecm,
      forwardWorld * leftThrust,
      gz::math::Vector3d(
        0.0,
        this->thrusterY,
        0.0));

    this->link.AddWorldForce(
      _ecm,
      forwardWorld * rightThrust,
      gz::math::Vector3d(
        0.0,
        -this->thrusterY,
        0.0));

    // -------------------------------------------------------
    // Simple passive water-like damping.
    // Values are configurable from SDF.
    // -------------------------------------------------------

    const auto worldVelocity =
      this->link.WorldLinearVelocity(_ecm);

    if (worldVelocity)
    {
      const auto bodyVelocity =
        rotation.RotateVectorReverse(*worldVelocity);

      const gz::math::Vector3d dragBody(
        -this->surgeDrag * bodyVelocity.X(),
        -this->swayDrag * bodyVelocity.Y(),
        0.0);

      const auto dragWorld =
        rotation.RotateVector(dragBody);

      this->link.AddWorldForce(
        _ecm,
        dragWorld);
    }

    const auto worldAngularVelocity =
      this->link.WorldAngularVelocity(_ecm);

    if (worldAngularVelocity)
    {
      const auto bodyAngularVelocity =
        rotation.RotateVectorReverse(
          *worldAngularVelocity);

      const gz::math::Vector3d torqueBody(
        0.0,
        0.0,
        -this->yawDrag *
        bodyAngularVelocity.Z());

      const auto torqueWorld =
        rotation.RotateVector(torqueBody);

      this->link.AddWorldWrench(
        _ecm,
        gz::math::Vector3d(0.0, 0.0, 0.0),
        torqueWorld);
    }
  }


private:

  gz::transport::Node node;

  gz::transport::Node::Publisher publisher;

  gz::sim::Link link;

  bool forceMode{false};

  double maxSpeed{3.0};

  double maxYawRate{0.8};

  // Slew limits for the legacy VelocityControl command.
  double maxLinearAccel{1.0};
  double maxYawAccel{0.8};

  double commandLinear{0.0};
  double commandYaw{0.0};

  double maxThrustPerSide{75.0};

  double thrusterY{1.2};

  double surgeDrag{50.0};

  double swayDrag{100.0};

  double yawDrag{80.0};

  double left{0.0};

  double right{0.0};

  std::mutex mutex;
};

}

GZ_ADD_PLUGIN(
  uav_usv_gazebo::ArduPilotRoverBridge,
  gz::sim::System,
  uav_usv_gazebo::ArduPilotRoverBridge::ISystemConfigure,
  uav_usv_gazebo::ArduPilotRoverBridge::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  uav_usv_gazebo::ArduPilotRoverBridge,
  "uav_usv_gazebo::ArduPilotRoverBridge")
