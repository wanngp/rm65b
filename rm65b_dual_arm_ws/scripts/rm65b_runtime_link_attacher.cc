#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace rm65b
{

class RuntimeLinkAttacher
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  struct Attachment
  {
    std::string name;
    std::string parentLink;
    std::string childLink;
    std::string attachTopic;
    std::string detachTopic;
    std::string outputTopic;
    gz::sim::Entity jointEntity{gz::sim::kNullEntity};
    std::atomic<bool> attachRequested{false};
    std::atomic<bool> detachRequested{false};
    bool attached{false};
    gz::transport::Node::Publisher pub;
  };

  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    auto elem = _sdf->FindElement("attachment");
    while (elem)
    {
      auto item = std::make_shared<Attachment>();
      item->name = elem->Get<std::string>("name", "attachment").first;
      item->parentLink = elem->Get<std::string>("parent_link", "").first;
      item->childLink = elem->Get<std::string>("child_link", "").first;
      item->attachTopic = elem->Get<std::string>(
          "attach_topic", "/rm65b/physical/" + item->name + "/attach").first;
      item->detachTopic = elem->Get<std::string>(
          "detach_topic", "/rm65b/physical/" + item->name + "/detach").first;
      item->outputTopic = elem->Get<std::string>(
          "output_topic", "/rm65b/physical/" + item->name + "/state").first;

      if (item->parentLink.empty() || item->childLink.empty())
      {
        gzerr << "rm65b RuntimeLinkAttacher attachment [" << item->name
              << "] is missing parent_link or child_link" << std::endl;
        elem = elem->GetNextElement("attachment");
        continue;
      }

      this->attachments.push_back(item);

      this->node.Subscribe(
          item->attachTopic,
          std::function<void(const gz::msgs::Empty &)>(
          [item](const gz::msgs::Empty &)
          {
            gzmsg << "rm65b RuntimeLinkAttacher attach requested ["
                  << item->name << "]" << std::endl;
            item->attachRequested = true;
          }));
      this->node.Subscribe(
          item->detachTopic,
          std::function<void(const gz::msgs::Empty &)>(
          [item](const gz::msgs::Empty &)
          {
            gzmsg << "rm65b RuntimeLinkAttacher detach requested ["
                  << item->name << "]" << std::endl;
            item->detachRequested = true;
          }));
      item->pub = this->node.Advertise<gz::msgs::StringMsg>(item->outputTopic);
      gzmsg << "rm65b RuntimeLinkAttacher registered [" << item->name
            << "] parent=" << item->parentLink
            << " child=" << item->childLink << std::endl;

      elem = elem->GetNextElement("attachment");
    }
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &_ecm) override
  {
    for (auto &itemPtr : this->attachments)
    {
      auto &item = *itemPtr;
      if (item.detachRequested.exchange(false) && item.attached)
      {
        if (item.jointEntity != gz::sim::kNullEntity)
        {
          _ecm.RequestRemoveEntity(item.jointEntity);
        }
        item.jointEntity = gz::sim::kNullEntity;
        item.attached = false;
        this->Publish(item, "detached");
      }

      if (item.attachRequested.exchange(false) && !item.attached)
      {
        const auto parent = this->ResolveLink(_ecm, item.parentLink);
        const auto child = this->ResolveLink(_ecm, item.childLink);
        if (parent == gz::sim::kNullEntity || child == gz::sim::kNullEntity)
        {
          gzerr << "rm65b RuntimeLinkAttacher attach failed [" << item.name
                << "] parent_entity=" << parent
                << " child_entity=" << child << std::endl;
          this->Publish(item, "attach_failed_missing_link");
          continue;
        }

        item.jointEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(
            item.jointEntity,
            gz::sim::components::DetachableJoint({parent, child, "fixed"}));
        item.attached = true;
        gzmsg << "rm65b RuntimeLinkAttacher attached [" << item.name
              << "] joint_entity=" << item.jointEntity
              << " parent_entity=" << parent
              << " child_entity=" << child << std::endl;
        this->Publish(item, "attached");
      }
    }
  }

  private: gz::sim::Entity ResolveLink(
      gz::sim::EntityComponentManager &_ecm,
      const std::string &_scopedName) const
  {
    auto candidates = gz::sim::entitiesFromScopedName(_scopedName, _ecm);
    for (const auto entity : candidates)
    {
      if (_ecm.EntityHasComponentType(
              entity, gz::sim::components::Link::typeId))
      {
        return entity;
      }
    }
    return gz::sim::kNullEntity;
  }

  private: void Publish(Attachment &_item, const std::string &_state)
  {
    if (!_item.pub)
    {
      return;
    }
    gz::msgs::StringMsg msg;
    msg.set_data(_item.name + ":" + _state);
    _item.pub.Publish(msg);
  }

  private: gz::transport::Node node;
  private: std::vector<std::shared_ptr<Attachment>> attachments;
};

}  // namespace rm65b

GZ_ADD_PLUGIN(
    rm65b::RuntimeLinkAttacher,
    gz::sim::System,
    rm65b::RuntimeLinkAttacher::ISystemConfigure,
    rm65b::RuntimeLinkAttacher::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    rm65b::RuntimeLinkAttacher,
    "rm65b::RuntimeLinkAttacher")
