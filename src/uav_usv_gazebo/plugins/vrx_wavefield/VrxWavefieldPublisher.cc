#include <chrono>
#include <memory>

#include <gz/msgs/param.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "Wavefield.hh"

namespace vrx
{
class VrxWavefieldPublisher
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    this->wavefield.Load(_sdf);
    this->publishPeriod = 2.0;
    if (_sdf->HasElement("publish_period"))
      this->publishPeriod = _sdf->Get<double>("publish_period");
    this->publisher = this->node.Advertise<gz::msgs::Param>(
        this->wavefield.Topic());
    if (!this->publisher.Valid())
      gzerr << "Unable to advertise VRX wavefield topic ["
            << this->wavefield.Topic() << "]" << std::endl;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &) override
  {
    if (_info.paused || !this->publisher.Valid() ||
        !this->wavefield.Active())
      return;

    const double now =
        std::chrono::duration<double>(_info.simTime).count();
    if (now + 1e-9 < this->nextPublish)
      return;

    this->publisher.Publish(this->wavefield.Parameters());
    this->nextPublish = now + this->publishPeriod;
  }

  private: vrx::Wavefield wavefield;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: double publishPeriod{2.0};
  private: double nextPublish{0.0};
};
}

GZ_ADD_PLUGIN(vrx::VrxWavefieldPublisher,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(vrx::VrxWavefieldPublisher,
                    "vrx::VrxWavefieldPublisher")
