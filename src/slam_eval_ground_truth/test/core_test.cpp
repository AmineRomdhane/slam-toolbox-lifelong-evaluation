#include <iostream>
#include <stdexcept>
#include "slam_eval_ground_truth/core.hpp"
using namespace slam_eval_ground_truth;
void check(bool value) { if (!value) throw std::runtime_error("check failed"); }
gz::msgs::Pose_V message(int sec, int model=43, int link=44) {
  gz::msgs::Pose_V m; m.mutable_header()->mutable_stamp()->set_sec(sec);
  m.mutable_header()->mutable_stamp()->set_nsec(123456789);
  // Link deliberately precedes model. Composition rotates its offset.
  auto *l=m.add_pose(); l->set_name("base_footprint"); l->set_id(link);
  l->mutable_position()->set_x(1); l->mutable_orientation()->set_w(1);
  auto *p=m.add_pose(); p->set_name("burger"); p->set_id(model);
  p->mutable_position()->set_x(2); p->mutable_position()->set_z(3);
  p->mutable_orientation()->set_w(std::sqrt(0.5));
  p->mutable_orientation()->set_z(std::sqrt(0.5));
  return m;
}
int main() {
  gz::msgs::Scene scene;
  auto *model=scene.add_model(); model->set_name("burger"); model->set_id(43);
  auto *link=model->add_link(); link->set_name("base_footprint"); link->set_id(44);
  check(resolve(scene,"burger","base_footprint")==std::optional<Ids>({43,44}));
  check(!resolve(scene,"absent","base_footprint"));
  Core c("burger","base_footprint"); c.set_ids({43,44});
  auto r=c.process(message(10));
  check(r.has_value() && r->sec==10 && r->nsec==123456789);
  check(std::abs(r->pose.Pos().X()-2)<1e-10 && std::abs(r->pose.Pos().Y()-1)<1e-10);
  check(std::abs(r->pose.Pos().Z()-3)<1e-10);
  check(!c.process(message(10))); check(c.counters.duplicates==1);
  check(!c.process(message(1))); check(c.counters.resets==1 && !c.ids);
  c.set_ids({43,44}); check(c.process(message(2)).has_value());
  check(!c.process(message(3,80,81))); check(!c.ids);
  c.set_ids({80,81}); check(c.counters.id_changes==1);
  check(c.process(message(4,80,81)).has_value());
  auto invalid=message(5,80,81); invalid.mutable_pose(1)->set_name("other");
  check(!c.process(invalid));
  c.set_ids({80,81});
  auto bad=message(6,80,81); bad.mutable_pose(0)->mutable_orientation()->set_w(0);
  check(!c.process(bad)); check(c.counters.invalid==1);
  scene.add_model()->CopyFrom(scene.model(0));
  check(!resolve(scene,"burger","base_footprint"));
  std::cout << "PASS composition, exact stamp, reordered entities, duplicates, reset, replacement, invalid pose, ambiguous scene\n";
}
