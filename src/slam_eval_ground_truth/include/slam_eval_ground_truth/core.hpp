#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/math/Pose3.hh>

namespace slam_eval_ground_truth {
using Ids = std::pair<uint64_t, uint64_t>;
// Only direct world children and direct model links are supported. This
// guarantees the two parent-relative transforms have the advertised meaning.
inline std::optional<Ids> resolve(const gz::msgs::Scene &scene,
    const std::string &model, const std::string &link) {
  std::optional<Ids> result;
  unsigned models = 0, links = 0;
  for (const auto &m : scene.model()) {
    if (m.name() != model) continue;
    ++models;
    for (const auto &l : m.link()) {
      if (l.name() == link) { ++links; result = Ids{m.id(), l.id()}; }
    }
  }
  return models == 1 && links == 1 ? result : std::nullopt;
}
struct Counters {
  uint64_t received = 0, published = 0, missing = 0, resets = 0,
           id_changes = 0, invalid = 0, duplicates = 0;
};
struct Result {
  int32_t sec;
  uint32_t nsec;
  gz::math::Pose3d pose;
};
class Core {
public:
  Core(std::string model, std::string link) : model_(std::move(model)), link_(std::move(link)) {}
  Counters counters;
  uint64_t generation = 0;
  std::optional<Ids> ids;
  void invalidate() { ids.reset(); ++generation; }
  bool set_ids(Ids next) {
    bool changed = !ids || *ids != next;
    if (previous_ids_ && *previous_ids_ != next) ++counters.id_changes;
    previous_ids_ = next;
    ids = next;
    return changed;
  }
  std::optional<Result> process(const gz::msgs::Pose_V &msg) {
    ++counters.received;
    if (!msg.has_header() || !msg.header().has_stamp()) {
      ++counters.invalid; return {};
    }
    const auto &s = msg.header().stamp();
    if (s.sec() < 0 || s.sec() > std::numeric_limits<int32_t>::max() ||
        s.nsec() < 0 || s.nsec() >= 1000000000) {
      ++counters.invalid; return {};
    }
    const int64_t t = s.sec() * int64_t{1000000000} + s.nsec();
    if (last_received_ && t < *last_received_) {
      ++counters.resets;
      invalidate();
      last_published_.reset();
    }
    last_received_ = t;
    if (!ids) { ++counters.missing; return {}; }
    const gz::msgs::Pose *model = nullptr, *link = nullptr;
    unsigned nm = 0, nl = 0;
    for (const auto &p : msg.pose()) {
      if (p.id() == ids->first && p.name() == model_) { model = &p; ++nm; }
      if (p.id() == ids->second && p.name() == link_) { link = &p; ++nl; }
    }
    if (nm != 1 || nl != 1) {
      ++counters.missing; invalidate(); return {};
    }
    if (last_published_ && t <= *last_published_) {
      ++counters.duplicates; return {};
    }
    auto a = pose(*model), b = pose(*link);
    if (!a || !b) { ++counters.invalid; return {}; }
    last_published_ = t;
    return Result{static_cast<int32_t>(s.sec()), static_cast<uint32_t>(s.nsec()), *a * *b};
  }
private:
  static std::optional<gz::math::Pose3d> pose(const gz::msgs::Pose &p) {
    const auto &v = p.position(); const auto &q = p.orientation();
    for (double x : {v.x(),v.y(),v.z(),q.w(),q.x(),q.y(),q.z()})
      if (!std::isfinite(x)) return {};
    const double norm = q.w()*q.w()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z();
    if (std::abs(norm-1.0) > 1e-3) return {};
    return gz::math::Pose3d(gz::math::Vector3d(v.x(),v.y(),v.z()),
                           gz::math::Quaterniond(q.w(),q.x(),q.y(),q.z()));
  }
  std::string model_, link_;
  std::optional<Ids> previous_ids_;
  std::optional<int64_t> last_received_, last_published_;
};
}
