// Gazebo's unmodified Sensors/Ogre2 pipeline is the geometric authority.
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity.pb.h>
#include <filesystem>
#include <gz/msgs/boolean.pb.h>
#include <gz/math/Pose3.hh>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <stdexcept>
int main(int argc,char **argv) {try {
 if(argc!=4)throw std::runtime_error("Usage: cast_reference world.sdf lidar_poses.csv rays.csv");
 gz::transport::Node node;std::mutex mutex;std::condition_variable cv;
 gz::math::Pose3d target;bool ready=false;int matches=0;std::string expectedFrame;gz::msgs::LaserScan last;
 std::function<void(const gz::msgs::LaserScan&)> cb=[&](const auto &m){
  std::lock_guard<std::mutex> l(mutex);if(!ready)return;
  auto &p=m.world_pose().position();auto &q=m.world_pose().orientation();
  gz::math::Pose3d actual(p.x(),p.y(),p.z(),0,0,0);actual.Rot()=gz::math::Quaterniond(q.w(),q.x(),q.y(),q.z());
  if(m.frame()!=expectedFrame || (actual.Pos()-target.Pos()).Length()>1e-7 || std::abs(std::abs(actual.Rot().Dot(target.Rot()))-1)>1e-9)return;
  last=m;++matches;cv.notify_all();};
 if(!node.Subscribe("/reference/scan",cb))throw std::runtime_error("Subscribe failed");
 gz::sim::ServerConfig config;config.SetSdfFile(argv[1]);config.SetHeadlessRendering(true);
 gz::sim::Server server(config);if(!server.Run(false,0,false))throw std::runtime_error("Server run failed");
 std::ifstream input(argv[2]);std::string line;std::getline(input,line);
 std::ofstream output(argv[3]);output<<std::setprecision(17)<<"scan_index,canonical_stamp_ns,ray_index,range_m,angle_rad,world_x,world_y,world_z,qx,qy,qz,qw\n";
 std::ifstream templ(std::filesystem::path(argv[1]).parent_path()/"reference_sensor.sdf");std::string sensorTemplate((std::istreambuf_iterator<char>(templ)),{});
 int index=0;
 while(std::getline(input,line)){
  for(auto &c:line)if(c==',')c=' ';std::istringstream row(line);long long stamp;double x,y,z,qx,qy,qz,qw;row>>stamp>>x>>y>>z>>qx>>qy>>qz>>qw;
  if(!row)throw std::runtime_error("Invalid pose row");
  {std::lock_guard<std::mutex> l(mutex);target=gz::math::Pose3d(gz::math::Vector3d(x,y,z),gz::math::Quaterniond(qw,qx,qy,qz));matches=0;expectedFrame="reference_"+std::to_string(index);ready=true;}
  std::string xml=sensorTemplate;
  auto replace=[&](const std::string &key,const std::string &value){size_t pos=0;while((pos=xml.find(key,pos))!=std::string::npos){xml.replace(pos,key.size(),value);pos+=value.size();}};
  std::ostringstream pose;pose<<std::setprecision(17)<<target;replace("@POSE@",pose.str());replace("@INDEX@",std::to_string(index));
  gz::msgs::EntityFactory req;req.set_sdf(xml);req.set_allow_renaming(false);
  gz::msgs::Boolean rep;bool result=false;
  if(!node.Request("/world/default/create",req,10000,rep,result)||!result||!rep.data())throw std::runtime_error("create failed");
  std::unique_lock<std::mutex> lock(mutex);
  // Two matching-pose frames eliminate transition frames; pose is verified on every accepted scan.
  if(!cv.wait_for(lock,std::chrono::seconds(30),[&]{return matches>=2;}))throw std::runtime_error("No two matching-pose lidar frames");
  auto m=last;ready=false;lock.unlock();
  if(m.count()!=360 || m.ranges_size()!=360 || m.vertical_count()!=1 || std::abs(m.angle_min())>1e-12 || std::abs(m.angle_max()-6.28)>1e-12 || std::abs(m.range_min()-.12)>1e-12 || std::abs(m.range_max()-3.5)>1e-12)throw std::runtime_error("Sensor geometry mismatch");
  for(int j=0;j<360;++j){double r=m.ranges(j);if(std::isnan(r)||r<.12||r>3.5&&std::isfinite(r))throw std::runtime_error("Invalid range");output<<index<<','<<stamp<<','<<j<<','<<r<<','<<m.angle_min()+j*m.angle_step()<<','<<x<<','<<y<<','<<z<<','<<qx<<','<<qy<<','<<qz<<','<<qw<<'\n';}
  gz::msgs::Entity remove;remove.set_name(expectedFrame);remove.set_type(gz::msgs::Entity::MODEL);
  if(!node.Request("/world/default/remove",remove,10000,rep,result)||!result||!rep.data())throw std::runtime_error("remove failed");
  if(index%50==0)std::cout<<"Accepted pose "<<index<<" stamp "<<stamp<<std::endl;++index;
 }
 output.close();server.Stop();std::cout<<"Complete: "<<index<<" poses"<<std::endl;
 }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;return 1;}return 0;}
