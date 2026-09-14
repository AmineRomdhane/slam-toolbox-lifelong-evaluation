import math
def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))
def slew(old, target, accel, dt):
    return old + max(-accel*dt, min(accel*dt, target-old))
def lateral(p, a, b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    den=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den)) if den else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
class Controller:
    def __init__(self, route, p):
        self.route,self.p=route,p
        self.state='WAITING'; self.i=0; self.v=0.; self.w=0.
        self.start=None; self.last=None; self.segment=None
        self.position_error=0.; self.heading_error=0.; self.tracking_error=0.
        self.reason=''; self.final=False; self.final_started=False
    def abort(self, reason):
        self.state='ABORTED'; self.reason=reason; self.v=self.w=0.
    def step(self, t, x, y, yaw):
        if self.state in ('COMPLETED','ABORTED'): return 0.,0.
        if not all(math.isfinite(z) for z in (t,x,y,yaw)):
            self.abort('nonfinite feedback'); return 0.,0.
        if self.start is None:
            self.start=t; self.last=t; self.deadline=t+self.p['waypoint_timeout']
            self.segment=(x,y); self.state='ROTATING'
            if math.dist((x,y),self.route['waypoints'][0])>self.p['max_tracking_error']:
                self.abort('start outside route admission limit')
            return 0.,0.
        dt=t-self.last; self.last=t
        if dt<0 or dt>self.p['feedback_timeout']:
            self.abort('time reset or feedback gap'); return 0.,0.
        if dt==0: return self.v,self.w
        if t>self.deadline:
            self.abort('waypoint timeout'); return 0.,0.
        goal=self.route['waypoints'][self.i]
        self.position_error=math.dist((x,y),goal)
        self.tracking_error=lateral((x,y),self.segment,goal)
        if self.tracking_error>self.p['max_tracking_error']:
            self.abort('tracking limit exceeded'); return 0.,0.
        angle=self.route['final_yaw'] if self.final else math.atan2(goal[1]-y,goal[0]-x)
        self.heading_error=wrap(angle-yaw)
        # Leave rotation-drift margin only on corrective final approaches.
        approach_tolerance=self.p['position_tolerance']*(.1 if self.final_started else 1.)
        vtarget=wtarget=0.
        if self.state=='ROTATING':
            if not self.final_started and not self.final and self.position_error<=approach_tolerance:
                self.advance(t,x,y)
                return self.v,self.w
            e=self.heading_error
            if abs(e)>self.p['heading_tolerance']:
                wtarget=math.copysign(min(self.p['max_angular_velocity'],1.5*abs(e),
                                    math.sqrt(2*self.p['max_angular_acceleration']*abs(e))),e)
            elif abs(self.w)<0.001:
                if self.final:
                    if self.position_error<=self.p['position_tolerance']:
                        self.state='COMPLETED'
                    else:
                        # Reuse the final waypoint approach, then restore final yaw.
                        # Keep the original final-phase deadline across all retries.
                        self.final=False
                else: self.state='TRANSLATING'
        elif self.state=='TRANSLATING':
            if self.position_error>approach_tolerance:
                d=max(0.,self.position_error-approach_tolerance*.5)
                vtarget=min(self.p['max_linear_velocity'],math.sqrt(2*self.p['max_linear_acceleration']*d),1.2*d)
                wtarget=max(-self.p['max_angular_velocity'],min(self.p['max_angular_velocity'],1.5*self.heading_error))
            elif abs(self.v)<0.001 and abs(self.w)<0.001:
                self.advance(t,x,y)
        self.v=slew(self.v,vtarget,self.p['max_linear_acceleration'],dt)
        self.w=slew(self.w,wtarget,self.p['max_angular_acceleration'],dt)
        return self.v,self.w
    def advance(self,t,x,y):
        if self.i==len(self.route['waypoints'])-1:
            self.final=True
            if not self.final_started:
                self.final_started=True
                self.deadline=t+self.p['waypoint_timeout']
        else:
            self.segment=tuple(self.route['waypoints'][self.i])
            self.i+=1
            self.deadline=t+self.p['waypoint_timeout']
        self.state='ROTATING'
