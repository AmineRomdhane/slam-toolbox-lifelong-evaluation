#!/usr/bin/env python3
import csv,json,math,os,time,signal
from pathlib import Path
import yaml
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from geometry_msgs.msg import PoseStamped,TwistStamped
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from control_core import Controller,wrap

class RouteNode(Node):
    def __init__(self):
        super().__init__('slam_eval_route_controller')
        defaults=dict(max_linear_velocity=.10,max_angular_velocity=.30,
            max_linear_acceleration=.10,max_angular_acceleration=.30,
            position_tolerance=.015,heading_tolerance=.015,waypoint_timeout=90.,
            max_tracking_error=.20,feedback_timeout=.50)
        self.p={k:self.declare_parameter(k,v).value for k,v in defaults.items()}
        if any(not math.isfinite(v) or v<=0 for v in self.p.values()):
            raise ValueError('All control limits must be finite and positive')
        route_path=self.declare_parameter('route_file',get_package_share_directory('slam_eval_route_controller')+'/routes/r1.yaml').value
        self.route=yaml.safe_load(Path(route_path).read_text())
        if not self.route.get('waypoints') or any(len(p)!=2 or not all(math.isfinite(v) for v in p) for p in self.route['waypoints']):
            raise ValueError('Invalid route waypoints')
        if not math.isfinite(self.route['final_yaw']): raise ValueError('Invalid final yaw')
        self.c=Controller(self.route,self.p)
        self.pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.status=self.create_publisher(String,'/route/status',10)
        self.sub=self.create_subscription(PoseStamped,'/ground_truth/pose',self.feedback,100)
        self.out=Path(self.declare_parameter('output_dir','/tmp/slam_route_trial').value)
        self.out.mkdir(parents=True,exist_ok=False)
        self.f=(self.out/'trajectory.csv').open('w')
        self.csv=csv.writer(self.f)
        self.csv.writerow(['sec','nanosec','x','y','z','qx','qy','qz','qw','yaw','waypoint','state','tracking_error','v','w'])
        self.rows=[]; self.start_pose=None; self.last_wall=time.monotonic(); self.done=False
        self.timer=self.create_timer(.1,self.watch,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def command(self,stamp,v,w):
        m=TwistStamped(); m.header.stamp=stamp; m.header.frame_id='base_footprint'
        m.twist.linear.x=v; m.twist.angular.z=w; self.pub.publish(m)
    def watch(self):
        if self.start_pose is None:
            self.status.publish(String(data=json.dumps(dict(route_id=self.route["route_id"],current_waypoint=0,elapsed_simulation_time=0.,position_error=0.,heading_error=0.,state="WAITING"))))
        if self.start_pose is not None and not self.done and time.monotonic()-self.last_wall>2.:
            self.c.abort('wall-time feedback watchdog')
            self.command(self.get_clock().now().to_msg(),0.,0.)
            self.finish()
    def feedback(self,m):
        if self.done: return
        self.last_wall=time.monotonic()
        t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        p,q=m.pose.position,m.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if m.header.frame_id!=self.route['reference_frame']:
            self.c.abort('wrong reference frame')
        if self.start_pose is None:
            self.start_pose=dict(sec=m.header.stamp.sec,nanosec=m.header.stamp.nanosec,
                frame_id=m.header.frame_id,xyz=[p.x,p.y,p.z],quaternion_xyzw=[q.x,q.y,q.z,q.w])
            (self.out/'configuration.json').write_text(json.dumps(dict(route=self.route,parameters=self.p,actual_start_pose=self.start_pose),indent=2))
            self.get_logger().info('Starting '+self.route['route_id']+' at '+str(self.start_pose))
        v,w=self.c.step(t,p.x,p.y,yaw)
        self.command(m.header.stamp,v,w)
        row=[m.header.stamp.sec,m.header.stamp.nanosec,p.x,p.y,p.z,q.x,q.y,q.z,q.w,yaw,self.c.i,self.c.state,self.c.tracking_error,v,w]
        self.csv.writerow(row); self.rows.append(row)
        s=dict(route_id=self.route['route_id'],current_waypoint=self.c.i,
          elapsed_simulation_time=t-(self.c.start if self.c.start is not None else t),position_error=self.c.position_error,
          heading_error=self.c.heading_error,tracking_error=self.c.tracking_error,state=self.c.state,reason=self.c.reason)
        self.status.publish(String(data=json.dumps(s)))
        if self.c.state in ('COMPLETED','ABORTED'): self.finish()
    def finish(self):
        if self.done:return
        self.done=True; self.f.close()
        elapsed=0. if self.c.start is None or self.c.last is None else self.c.last-self.c.start
        self.status.publish(String(data=json.dumps(dict(route_id=self.route['route_id'],
            current_waypoint=self.c.i,elapsed_simulation_time=elapsed,
            position_error=self.c.position_error,heading_error=self.c.heading_error,
            tracking_error=self.c.tracking_error,state=self.c.state,reason=self.c.reason))))
        rows=self.rows
        if rows:
            last=rows[-1]; ts=lambda r:r[0]+r[1]*1e-9
            result=dict(route_id=self.route['route_id'],state=self.c.state,reason=self.c.reason,
             duration=ts(last)-ts(rows[0]),path_length=sum(math.hypot(b[2]-a[2],b[3]-a[3]) for a,b in zip(rows,rows[1:])),
             maximum_tracking_error=max(r[12] for r in rows),mean_tracking_error=sum(r[12] for r in rows)/len(rows),
             final_position_error=math.dist(last[2:4],self.route['waypoints'][-1]),
             final_yaw_error=abs(wrap(last[9]-self.route['final_yaw'])),samples=len(rows),
             actual_start_pose=self.start_pose)
            (self.out/'result.json').write_text(json.dumps(result,indent=2))
            self.get_logger().info(json.dumps(result))
def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n=RouteNode()
    def interrupted(*_):
        n.c.abort("interrupted"); n.command(n.get_clock().now().to_msg(),0.,0.); n.finish()
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGTERM,interrupted)
    try:
        while rclpy.ok() and not n.done:rclpy.spin_once(n,timeout_sec=.1)
    finally:
        n.command(n.get_clock().now().to_msg(),0.,0.)
        n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
