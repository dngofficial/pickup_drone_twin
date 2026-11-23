#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace urc {
class FlightSystem final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  using Clock = std::chrono::steady_clock;
  using Entity = gz::sim::Entity;
  using Ecm = gz::sim::EntityComponentManager;
  using Vector = gz::math::Vector3d;
  struct Target {
    Entity link;
    std::string name;
    std::string kind;
    double mass;
    std::string token;
    double settled{0};
    bool counted{false};
  };

 public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &,
                 Ecm &ecm, gz::sim::EventManager &) override {
    body = gz::sim::Model(entity).LinkByName(ecm, "base_link");
    if (body == gz::sim::kNullEntity) return;
    gz::sim::Link(body).EnableVelocityChecks(ecm);
    node.Subscribe("/gz/urc/cmd_vel", &FlightSystem::OnTwist, this);
    node.Subscribe("/gz/urc/estop", &FlightSystem::OnStop, this);
    node.Subscribe("/gz/urc/gripper", &FlightSystem::OnGrip, this);
    statePublisher = node.Advertise<gz::msgs::StringMsg>("/gz/urc/state");
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, Ecm &ecm) override {
    if (info.paused || body == gz::sim::kNullEntity) return;
    const auto t = std::chrono::duration<double>(info.simTime).count();
    const auto dt = std::chrono::duration<double>(info.dt).count();
    if (!loaded) {
      ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const Entity &entity, const auto *, const auto *name) {
            const std::string id = name->Data();
            const bool bottle = id.rfind("bottle_", 0) == 0;
            if (bottle || id.rfind("hammer_", 0) == 0) {
              auto target = gz::sim::Model(entity).LinkByName(ecm, "object");
              if (target != gz::sim::kNullEntity) {
                targets.push_back({target,id,bottle?"bottle":"hammer",bottle?.15:.55,""});
                gz::sim::Link(target).EnableVelocityChecks(ecm);
              }
            }
            return true;
          });
      loaded = true;
    }
    Vector requested;
    double yawRate = 0;
    std::string command;
    {
      std::lock_guard<std::mutex> guard(mutex);
      if (!estop && Clock::now()-lastVelocity < std::chrono::milliseconds(500)) {
        requested = desired;
        yawRate = yaw;
      }
      if (!estop) command.swap(gripInput);
      else gripInput.clear();
    }
    const auto pose = gz::sim::worldPose(body, ecm);
    const auto vehicle = gz::sim::Link(body);
    if (!command.empty()) ApplyGrip(command, pose.Pos(), vehicle, ecm);
    const double limit = .8/(1.+1.5*(held ? held->mass : 0.));
    const double length=std::hypot(requested.X(),requested.Y());
    if (length>limit) { requested.X(requested.X()*limit/length); requested.Y(requested.Y()*limit/length); }
    requested.Z(std::clamp(requested.Z(),-.6,.6));
    const auto delta = requested - applied;
    applied += delta * std::min(1.,1.8*dt/std::max(delta.Length(),1e-7));
    vehicle.SetLinearVelocity(ecm, applied);
    vehicle.SetAngularVelocity(ecm, Vector(0,0,std::clamp(yawRate,-1.2,1.2)));
    CheckBins(ecm,dt);
    if (t-lastReport>=.049) {
      Report(pose, yawRate, t, ecm);
      lastReport=t;
    }
  }

 private:
  void OnTwist(const gz::msgs::Twist &msg) {
    Vector candidate(msg.linear().x(),msg.linear().y(),msg.linear().z());
    if (!candidate.IsFinite() || !std::isfinite(msg.angular().z())) return;
    std::lock_guard<std::mutex> guard(mutex);
    desired=candidate; yaw=msg.angular().z(); lastVelocity=Clock::now();
  }
  void OnStop(const gz::msgs::Boolean &msg) {
    std::lock_guard<std::mutex> guard(mutex);
    estop=msg.data(); desired=Vector::Zero; yaw=0; lastVelocity=Clock::time_point{};
  }
  void OnGrip(const gz::msgs::StringMsg &msg) {
    std::lock_guard<std::mutex> guard(mutex);
    gripInput=msg.data();
  }
  void ApplyGrip(const std::string &raw, const Vector &center, const gz::sim::Link &vehicle, Ecm &ecm) {
    const auto separator=raw.find(' ');
    if (separator==std::string::npos) return;
    const auto id=raw.substr(0,separator);
    const auto command=raw.substr(separator+1);
    if (id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_")!=std::string::npos || !handled.insert(id).second) return;
    request=id;
    if (command=="open") {
      closed=false;
      if (held) {
        ecm.RequestRemoveEntity(joint);
        held=nullptr;
      }
      return;
    }
    if (command!="close") return;
    closed=true;
    if (held || applied.Length()>.08) return;
    auto grip=center-Vector(0,0,.55);
    double best=.10;
    const auto velocity=vehicle.WorldLinearVelocity(ecm).value_or(Vector::Zero);
    for (auto &item: targets) {
      const auto p=gz::sim::worldPose(item.link,ecm).Pos();
      const auto v=gz::sim::Link(item.link).WorldLinearVelocity(ecm).value_or(Vector::Zero);
      const double distance=(p-grip).Length();
      if (distance<best && (v-velocity).Length()<.08) { best=distance; held=&item; }
    }
    if (held) {
      joint=ecm.CreateEntity();
      ecm.CreateComponent(joint,gz::sim::components::DetachableJoint({body,held->link,"fixed"}));
      payloadToken="payload-"+std::to_string(++serial);
      held->token=payloadToken; held->counted=false; held->settled=0;
    }
  }
  void CheckBins(const Ecm &ecm,double dt) {
    for (auto &item:targets) {
      if (item.token.empty() || item.counted || &item==held) continue;
      const auto p=gz::sim::worldPose(item.link,ecm).Pos();
      const auto v=gz::sim::Link(item.link).WorldLinearVelocity(ecm).value_or(Vector::Zero);
      const bool inside=std::abs(p.X()+2.)<.54 && std::abs(std::abs(p.Y())-2.5)<.54 && p.Z()>.025 && p.Z()<.61;
      item.settled=inside && v.Length()<.09 ? item.settled+dt : 0;
      if (item.settled>.35) {
        item.counted=true;
        events.push_back({item.token,p.Y()>0?"bin_a":"bin_b"});
      }
    }
  }
  void Report(const gz::math::Pose3d &pose,double omega,double t,const Ecm &ecm) {
    std::ostringstream out;
    out << std::setprecision(8) << "{\"t\":" << t << ",\"pose\":[" << pose.Pos().X() << ',' << pose.Pos().Y() << ','
        << pose.Pos().Z() << ',' << pose.Rot().Yaw() << "],\"flow\":[" << applied.X() << ',' << applied.Y() << "],\"gyro\":" << omega
        << ",\"gripper\":{\"request_id\":\"" << request << "\",\"payload_token\":\"" << payloadToken << "\",\"closed\":" << (closed?"true":"false")
        << ",\"attached\":" << (held?"true":"false") << ",\"payload_mass_kg\":" << (held?held->mass:0.) << "},\"events\":[";
    for (std::size_t i=0;i<events.size();++i) {
      if(i) out << ',';
      out << "[\"" << events[i].first << "\",\"" << events[i].second << "\"]";
    }
    events.clear();
    out << "],\"targets\":[";
    bool comma=false;
    for (const auto &item:targets) {
      if (&item==held || item.counted) continue;
      const auto xyz=gz::sim::worldPose(item.link,ecm).Pos();
      if(comma) out << ',';
      comma=true;
      out << "[\"" << item.kind << "\"," << xyz.X() << ',' << xyz.Y() << ',' << xyz.Z() << ']';
    }
    out << "]}";
    gz::msgs::StringMsg message;
    message.set_data(out.str()); statePublisher.Publish(message);
  }
  gz::transport::Node node;
  gz::transport::Node::Publisher statePublisher;
  std::mutex mutex;
  Clock::time_point lastVelocity{};
  Entity body{gz::sim::kNullEntity},joint{gz::sim::kNullEntity};
  Vector desired{},applied{};
  double yaw{0},lastReport{0};
  bool estop{false},loaded{false},closed{false};
  std::string gripInput,request,payloadToken;
  unsigned serial{0};
  Target *held{nullptr};
  std::vector<Target> targets;
  std::set<std::string> handled;
  std::vector<std::pair<std::string,std::string>> events;
};
}
GZ_ADD_PLUGIN(urc::FlightSystem,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(urc::FlightSystem,"urc::FlightSystem")
