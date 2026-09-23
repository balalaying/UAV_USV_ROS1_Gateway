#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
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
class RandomVesselMotion
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
{
  public: void Configure(
      const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm,
      EventManager & /*_eventMgr*/) override
  {
    this->entity = _entity;
    auto name = _ecm.Component<components::Name>(_entity);
    const std::string modelName = name ? name->Data() : "enemy_ship";

    this->ReadParameter(_sdf, "min_speed", this->minSpeed);
    this->ReadParameter(_sdf, "max_speed", this->maxSpeed);
    this->ReadParameter(_sdf, "min_cruise_seconds", this->minCruiseSeconds);
    this->ReadParameter(_sdf, "max_cruise_seconds", this->maxCruiseSeconds);
    this->ReadParameter(_sdf, "min_stop_seconds", this->minStopSeconds);
    this->ReadParameter(_sdf, "max_stop_seconds", this->maxStopSeconds);
    this->ReadParameter(_sdf, "stop_probability", this->stopProbability);
    this->ReadParameter(_sdf, "max_heading_change", this->maxHeadingChange);
    this->ReadParameter(_sdf, "turn_rate", this->turnRate);
    this->ReadParameter(_sdf, "operating_center_x", this->operatingCenterX);
    this->ReadParameter(_sdf, "operating_center_y", this->operatingCenterY);
    this->ReadParameter(_sdf, "operating_radius", this->operatingRadius);
    this->ReadParameter(_sdf, "boundary_margin", this->boundaryMargin);
    this->ReadParameter(_sdf, "patrol_min_x", this->patrolMinX);
    this->ReadParameter(_sdf, "patrol_max_x", this->patrolMaxX);
    this->ReadParameter(_sdf, "patrol_min_y", this->patrolMinY);
    this->ReadParameter(_sdf, "patrol_max_y", this->patrolMaxY);
    this->ReadParameter(_sdf, "patrol_lookahead_seconds",
        this->patrolLookaheadSeconds);
    this->ReadParameter(_sdf, "patrol_boundary_margin",
        this->patrolBoundaryMargin);
    this->ReadParameter(_sdf, "route_speed", this->routeSpeed);
    this->ReadParameter(_sdf, "waypoint_radius", this->waypointRadius);
    this->ReadParameter(_sdf, "heading_gain", this->headingGain);
    this->ReadParameter(_sdf, "avoid_radius", this->avoidRadius);
    this->ReadParameter(_sdf, "avoid_heading_gain", this->avoidHeadingGain);

    if (_sdf->HasElement("waypoint") || _sdf->HasElement("avoid_model"))
    {
      auto element = _sdf->GetFirstElement();
      while (element)
      {
        if (element->GetName() == "waypoint" &&
            element->HasElement("x") && element->HasElement("y"))
        {
          this->waypoints.emplace_back(
              element->Get<double>("x"), element->Get<double>("y"));
        }
        else if (element->GetName() == "avoid_model")
        {
          this->avoidModels.push_back(element->Get<std::string>());
        }
        element = element->GetNextElement();
      }
    }

    std::uint32_t seed = 332U;
    if (_sdf->HasElement("seed"))
      seed = _sdf->Get<std::uint32_t>("seed");
    this->random.seed(seed);

    std::string topic = "cmd_vel";
    if (_sdf->HasElement("command_topic"))
      topic = _sdf->Get<std::string>("command_topic");
    if (!topic.empty() && topic.front() != '/')
      topic = "/model/" + modelName + "/" + topic;
    this->publisher = this->node.Advertise<gz::msgs::Twist>(topic);

    this->SelectTurn(0.0);
  }

  public: void PreUpdate(
      const UpdateInfo &_info,
      EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;

    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (this->waypoints.empty() && now + 1e-6 >= this->stateDeadline)
      this->AdvanceState(now);

    double speed = this->commandedSpeed;
    double yawRate = this->commandedYawRate;
    const auto pose = _ecm.Component<components::Pose>(this->entity);
    if (pose && !this->waypoints.empty())
    {
      const double x = pose->Data().Pos().X();
      const double y = pose->Data().Pos().Y();
      auto target = this->waypoints[this->waypointIndex];
      double distance = std::hypot(target.first - x, target.second - y);
      if (distance <= this->waypointRadius)
      {
        this->waypointIndex = (this->waypointIndex + 1) % this->waypoints.size();
        target = this->waypoints[this->waypointIndex];
        distance = std::hypot(target.first - x, target.second - y);
      }

      const double desiredYaw = std::atan2(target.second - y, target.first - x);
      const double rawError = desiredYaw - pose->Data().Rot().Yaw();
      const double error = std::atan2(std::sin(rawError), std::cos(rawError));
      yawRate = std::clamp(
          this->headingGain * error, -this->turnRate, this->turnRate);

      // Keep forward motion visible during turns, while slowing enough to
      // draw a smooth repeatable S-shaped route instead of cutting corners.
      const double alignment = std::max(0.25, std::cos(error));
      speed = std::clamp(
          this->routeSpeed * alignment, this->minSpeed, this->maxSpeed);
    }
    else if (pose && this->operatingRadius > 0.0)
    {
      const double x = pose->Data().Pos().X();
      const double y = pose->Data().Pos().Y();
      const double relativeX = x - this->operatingCenterX;
      const double relativeY = y - this->operatingCenterY;
      const double distance = std::hypot(relativeX, relativeY);
      if (distance > this->operatingRadius - this->boundaryMargin)
      {
        const double desiredYaw = std::atan2(-relativeY, -relativeX);
        const double rawError = desiredYaw - pose->Data().Rot().Yaw();
        const double error = std::atan2(
            std::sin(rawError), std::cos(rawError));
        speed = std::max(this->minSpeed, 0.55 * this->maxSpeed);
        yawRate = std::clamp(
            0.8 * error, -this->turnRate, this->turnRate);
      }

      // A velocity-controlled vessel can tunnel through a thin collision at
      // high speed.  Keep it in a configured water corridor proactively,
      // using a future position so the turn begins before reaching shore.
      // The physical island collision remains a secondary safety barrier.
      if (this->patrolMinX < this->patrolMaxX &&
          this->patrolMinY < this->patrolMaxY)
      {
        const double yaw = pose->Data().Rot().Yaw();
        const double lookahead = std::max(0.0, this->patrolLookaheadSeconds);
        const double projectedSpeed = std::max(speed, this->minSpeed);
        const double futureX = x + projectedSpeed * lookahead * std::cos(yaw);
        const double futureY = y + projectedSpeed * lookahead * std::sin(yaw);
        const double margin = std::max(0.0, this->patrolBoundaryMargin);
        const bool approachingBoundary =
            futureX < this->patrolMinX + margin ||
            futureX > this->patrolMaxX - margin ||
            futureY < this->patrolMinY + margin ||
            futureY > this->patrolMaxY - margin;
        if (approachingBoundary)
        {
          const double targetX = 0.5 * (this->patrolMinX + this->patrolMaxX);
          const double targetY = 0.5 * (this->patrolMinY + this->patrolMaxY);
          const double desiredYaw = std::atan2(targetY - y, targetX - x);
          const double rawError = desiredYaw - yaw;
          const double error = std::atan2(
              std::sin(rawError), std::cos(rawError));
          speed = std::max(this->minSpeed, 0.55 * this->maxSpeed);
          yawRate = std::clamp(
              1.2 * error, -this->turnRate, this->turnRate);
        }
      }
    }

    // Reuse the reference algorithms' distance-based repulsion for the
    // independently moving target. This prevents a random leg from steering
    // the enemy vessel into the protected ship or one of the pursuing USVs.
    if (pose && this->avoidRadius > 0.0 && !this->avoidModels.empty())
    {
      const double x = pose->Data().Pos().X();
      const double y = pose->Data().Pos().Y();
      double repulsionX = 0.0;
      double repulsionY = 0.0;
      double nearestDistance = this->avoidRadius;
      for (const auto &modelName : this->avoidModels)
      {
        const Entity otherEntity = _ecm.EntityByComponents(
            components::Name(modelName));
        if (otherEntity == kNullEntity || otherEntity == this->entity)
          continue;
        const auto otherPose = _ecm.Component<components::Pose>(otherEntity);
        if (!otherPose)
          continue;
        const double differenceX = x - otherPose->Data().Pos().X();
        const double differenceY = y - otherPose->Data().Pos().Y();
        const double distance = std::hypot(differenceX, differenceY);
        if (distance <= 1e-6 || distance >= this->avoidRadius)
          continue;
        nearestDistance = std::min(nearestDistance, distance);
        const double magnitude = 1.0 / distance - 1.0 / this->avoidRadius;
        repulsionX += magnitude * differenceX / distance;
        repulsionY += magnitude * differenceY / distance;
      }
      if (std::hypot(repulsionX, repulsionY) > 1e-9)
      {
        const double desiredYaw = std::atan2(repulsionY, repulsionX);
        const double rawError = desiredYaw - pose->Data().Rot().Yaw();
        const double error = std::atan2(std::sin(rawError), std::cos(rawError));
        const double urgency = std::clamp(
            (this->avoidRadius - nearestDistance) / this->avoidRadius,
            0.0, 1.0);
        yawRate = std::clamp(
            yawRate + this->avoidHeadingGain * urgency * error,
            -this->turnRate, this->turnRate);
        speed = std::max(speed, this->minSpeed +
            urgency * (this->maxSpeed - this->minSpeed));
      }
    }

    if (now - this->lastPublishTime < 0.1)
      return;
    this->lastPublishTime = now;
    gz::msgs::Twist command;
    command.mutable_linear()->set_x(speed);
    command.mutable_angular()->set_z(yawRate);
    this->publisher.Publish(command);
  }

  private: enum class MotionState {Stop, Turn, Cruise};

  private: template<typename T>
  void ReadParameter(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      T &_value)
  {
    if (_sdf->HasElement(_name))
      _value = _sdf->Get<T>(_name);
  }

  private: double Uniform(double _minimum, double _maximum)
  {
    std::uniform_real_distribution<double> distribution(_minimum, _maximum);
    return distribution(this->random);
  }

  private: void SelectTurn(double _now)
  {
    this->state = MotionState::Turn;
    const double delta = this->Uniform(
        -this->maxHeadingChange, this->maxHeadingChange);
    const double direction = delta < 0.0 ? -1.0 : 1.0;
    // Keep the vessel visibly underway while it turns.  The old 0.45 scale
    // could drop a configured 0.4 m/s patrol below the world-model moving
    // threshold even though no stop state had been selected.
    this->commandedSpeed = std::clamp(
        0.75 * this->Uniform(this->minSpeed, this->maxSpeed),
        this->minSpeed, this->maxSpeed);
    this->commandedYawRate = direction * this->turnRate;
    this->stateDeadline = _now + std::max(
        0.35, std::abs(delta) / std::max(0.01, this->turnRate));
  }

  private: void AdvanceState(double _now)
  {
    if (this->state == MotionState::Turn)
    {
      this->state = MotionState::Cruise;
      this->commandedSpeed = this->Uniform(this->minSpeed, this->maxSpeed);
      this->commandedYawRate = this->Uniform(-0.025, 0.025);
      this->stateDeadline = _now + this->Uniform(
          this->minCruiseSeconds, this->maxCruiseSeconds);
      return;
    }

    if (this->state == MotionState::Cruise &&
        this->Uniform(0.0, 1.0) < this->stopProbability)
    {
      this->state = MotionState::Stop;
      this->commandedSpeed = 0.0;
      this->commandedYawRate = 0.0;
      this->stateDeadline = _now + this->Uniform(
          this->minStopSeconds, this->maxStopSeconds);
      return;
    }
    this->SelectTurn(_now);
  }

  private: Entity entity{kNullEntity};
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: std::mt19937 random;
  private: MotionState state{MotionState::Turn};
  private: double minSpeed{0.35};
  private: double maxSpeed{1.15};
  private: double minCruiseSeconds{5.0};
  private: double maxCruiseSeconds{14.0};
  private: double minStopSeconds{2.0};
  private: double maxStopSeconds{6.0};
  private: double stopProbability{0.28};
  private: double maxHeadingChange{1.75};
  private: double turnRate{0.22};
  private: double operatingCenterX{0.0};
  private: double operatingCenterY{0.0};
  private: double operatingRadius{155.0};
  private: double boundaryMargin{18.0};
  private: double patrolMinX{0.0};
  private: double patrolMaxX{0.0};
  private: double patrolMinY{0.0};
  private: double patrolMaxY{0.0};
  private: double patrolLookaheadSeconds{4.0};
  private: double patrolBoundaryMargin{12.0};
  private: std::vector<std::pair<double, double>> waypoints;
  private: std::size_t waypointIndex{0U};
  private: double routeSpeed{0.32};
  private: double waypointRadius{5.0};
  private: double headingGain{1.2};
  private: double avoidRadius{0.0};
  private: double avoidHeadingGain{1.5};
  private: std::vector<std::string> avoidModels;
  private: double commandedSpeed{0.0};
  private: double commandedYawRate{0.0};
  private: double stateDeadline{0.0};
  private: double lastPublishTime{-1.0};
};
}
}
}
}

GZ_ADD_PLUGIN(
    gz::sim::systems::RandomVesselMotion,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::RandomVesselMotion,
    "gz::sim::systems::RandomVesselMotion")
