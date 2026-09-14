#include <chrono>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <gz/transport/Node.hh>
#include <gz/msgs/uint32_v.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "slam_eval_ground_truth/core.hpp"

using namespace std::chrono_literals;
namespace se = slam_eval_ground_truth;
class GroundTruthNode : public rclcpp::Node {
public:
  GroundTruthNode() : Node("slam_eval_ground_truth"),
    world_(declare_parameter("world_name", "default")),
    model_(declare_parameter("model_name", "burger")),
    tracked_(declare_parameter("tracked_frame", "base_footprint")),
    output_frame_(declare_parameter("output_frame", "gazebo_world")),
    core_(model_, tracked_) {
    if (world_.empty() || model_.empty() || tracked_.empty() || output_frame_.empty())
      throw std::runtime_error("Frame, model and world names must be nonempty");
    const auto output = declare_parameter("output_topic", "/ground_truth/pose");
    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(output, rclcpp::QoS(100));
    topic_ = "/world/" + world_ + "/dynamic_pose/info";
    deletion_topic_ = "/world/" + world_ + "/scene/deletion";
    if (!transport_.Subscribe(topic_, &GroundTruthNode::receive, this) ||
        !transport_.Subscribe(deletion_topic_, &GroundTruthNode::deleted, this))
      throw std::runtime_error("Failed to subscribe to Gazebo transport");
    timer_ = create_wall_timer(500ms, [this] { discover(); });
    RCLCPP_INFO(get_logger(), "Gazebo %s -> %s; tracked=%s, reference=%s (world %s)",
                topic_.c_str(), output.c_str(), tracked_.c_str(), output_frame_.c_str(), world_.c_str());
  }
  ~GroundTruthNode() override {
    timer_.reset();
    transport_.Unsubscribe(topic_);
    transport_.Unsubscribe(deletion_topic_);
    std::lock_guard<std::mutex> lock(mutex_);
    const auto &c = core_.counters;
    std::cout << "Ground-truth counters: received=" << c.received
              << " published=" << c.published << " missing_entity_messages=" << c.missing
              << " time_resets=" << c.resets << " entity_id_changes=" << c.id_changes
              << " invalid_messages=" << c.invalid << " duplicate_stamps=" << c.duplicates
              << std::endl;
  }
private:
  void discover() {
    uint64_t generation;
    { std::lock_guard<std::mutex> lock(mutex_); generation = core_.generation; }
    gz::msgs::Scene scene;
    bool ok = false;
    const bool answered = transport_.Request("/world/" + world_ + "/scene/info", 200u, scene, ok);
    auto ids = answered && ok ? se::resolve(scene, model_, tracked_) : std::nullopt;
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != core_.generation) return; // Reset/deletion during request.
    if (!ids) {
      if (core_.ids) core_.invalidate();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Awaiting unambiguous model/link in Gazebo scene");
      return;
    }
    if (core_.set_ids(*ids))
      RCLCPP_INFO(get_logger(), "Resolved world=%s model=%s id=%llu tracked=%s id=%llu",
        world_.c_str(), model_.c_str(), static_cast<unsigned long long>(ids->first),
        tracked_.c_str(), static_cast<unsigned long long>(ids->second));
  }
  void deleted(const gz::msgs::UInt32_V &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!core_.ids) return;
    for (auto id : msg.data()) {
      if (id == core_.ids->first || id == core_.ids->second) {
        core_.invalidate();
        RCLCPP_WARN(get_logger(), "Tracked entity deleted; awaiting rediscovery");
        break;
      }
    }
  }
  void receive(const gz::msgs::Pose_V &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto before = core_.counters.resets;
    auto result = core_.process(msg);
    if (core_.counters.resets != before)
      RCLCPP_WARN(get_logger(), "Simulation time moved backwards; rediscovering entities");
    if (!result || !rclcpp::ok()) return;
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp.sec = result->sec;
    out.header.stamp.nanosec = result->nsec;
    out.header.frame_id = output_frame_;
    const auto &p = result->pose.Pos(); const auto &q = result->pose.Rot();
    out.pose.position.x=p.X(); out.pose.position.y=p.Y(); out.pose.position.z=p.Z();
    out.pose.orientation.w=q.W(); out.pose.orientation.x=q.X();
    out.pose.orientation.y=q.Y(); out.pose.orientation.z=q.Z();
    publisher_->publish(out);
    ++core_.counters.published;
  }
  std::string world_, model_, tracked_, output_frame_, topic_, deletion_topic_;
  se::Core core_;
  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  gz::transport::Node transport_;
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<GroundTruthNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception &e) {
    std::cerr << "ground_truth_node: " << e.what() << std::endl;
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown();
  return 0;
}
