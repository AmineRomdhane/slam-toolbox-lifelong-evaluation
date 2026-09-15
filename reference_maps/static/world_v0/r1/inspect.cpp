#include <sdf/Root.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Sensor.hh>
#include <sdf/Lidar.hh>
#include <iostream>
int main(int argc,char**argv){sdf::Root r;auto e=r.Load(argv[1]);for(auto &x:e)std::cerr<<x<<"\n";auto m=r.Model();auto l=m->LinkByName("base_scan");auto s=l->SensorByIndex(0);gz::math::Pose3d p;auto pe=s->SemanticPose().Resolve(p,"base_footprint");for(auto &x:pe)std::cerr<<x<<"\n";auto n=s->LidarSensor()->LidarNoise();std::cout<<"physical_sensor_pose_in_base_footprint "<<p<<"\nnoise_type "<<int(n.Type())<<" mean "<<n.Mean()<<" stddev "<<n.StdDev()<<"\n";return pe.empty()?0:1;}
