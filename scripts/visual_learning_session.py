#!/usr/bin/env python3
"""Local commissioning session: small ROS steps, continuous hold, recorded evidence.

Requires an already running ROS controller stack. Never opens the serial bus.
Camera images and measured calibration remain in the supplied local directory.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_flex_joint',
          'wrist_flex_joint', 'wrist_roll_joint', 'gripper_joint']


def rotation(axis, angle):
    axis = np.asarray(axis, float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*skew@skew


def claw_position(model, positions):
    by_child = {j.find('child').get('link'): j for j in model.findall('joint')}
    chain, link = [], 'gripper_frame_link'
    while link != 'base_link':
        j = by_child[link]
        chain.append(j)
        link = j.find('parent').get('link')
    transform = np.eye(4)
    for j in reversed(chain):
        origin = j.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
        local = np.eye(4)
        local[:3, 3] = xyz
        local[:3, :3] = rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
        if j.get('type') in ('revolute', 'continuous'):
            local[:3,:3] = local[:3,:3]@rotation([float(v) for v in j.find('axis').get('xyz').split()], positions[j.get('name')])
        transform = transform@local
    return dict(zip(('x','y','z'), transform[:3,3].tolist()))


class Session:
    def __init__(self, args):
        self.args = args
        self.cal = json.loads(args.calibration.read_text())
        self.model = ET.parse(args.model).getroot()
        self.offsets = {j.get('name'): float(j.find("param[@name='position_offset_rad']").text)
                        if j.find("param[@name='position_offset_rad']") is not None else 0
                        for j in self.model.findall('.//ros2_control/joint')}
        self.node = rclpy.create_node('visual_learning_session')
        self.positions, self.last_feedback = {}, 0
        self.trace, self.active, self.latched, self.command_deadline = [], None, False, 0
        self.controller_check = None
        self.controllers_active, self.controllers_checked_at = False, 0
        self.lock = threading.Lock()
        self.node.create_subscription(JointState, '/joint_states', self.feedback, 10)
        self.node.create_subscription(JointTrajectoryControllerState, '/joint_trajectory_controller/controller_state', self.tracking, 10)
        self.arm = ActionClient(self.node, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.gripper = ActionClient(self.node, ParallelGripperCommand, '/gripper_controller/gripper_cmd')
        self.controllers = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
        threading.Thread(target=self.monitor, daemon=True).start()

    def feedback(self, msg):
        if set(JOINTS).issubset(msg.name):
            self.positions = dict(zip(msg.name, msg.position))
            self.last_feedback = time.monotonic()

    def tracking(self, msg):
        if self.active is not None:
            self.trace.append({'time':time.time(),'joints':list(msg.joint_names),
                               'reference':list(msg.reference.positions),'feedback':list(msg.feedback.positions),
                               'error':list(msg.error.positions)})

    def fresh(self):
        return len(self.positions) >= 6 and time.monotonic()-self.last_feedback < 0.4

    def model_positions(self, positions):
        return {n: positions[n]+self.offsets.get(n,0) for n in JOINTS}

    def state(self):
        positions = self.positions.copy()
        return {'fresh':self.fresh(),'latched':self.latched,'busy':self.lock.locked(),
                'raw_ros_positions':positions,
                'estimated_claw_m':claw_position(self.model,self.model_positions(positions)) if len(positions)>=6 else None}

    def wait(self, future, seconds):
        until = time.monotonic()+seconds
        while not future.done() and time.monotonic()<until:
            time.sleep(0.02)
        if not future.done():
            raise RuntimeError('ROS operation timed out')
        return future.result()

    def ready(self):
        if not self.fresh() or self.latched:
            raise RuntimeError('Stale feedback or stop latched')
        if not self.controllers.service_is_ready():
            raise RuntimeError('Controller manager unavailable')
        if not self.controllers_active or time.monotonic()-self.controllers_checked_at>5:
            raise RuntimeError('Required controllers not active')

    def capture(self, directory, label):
        for role in ('external','wrist'):
            frame = urllib.request.urlopen(self.args.camera_url+'/snapshot/'+role,timeout=3).read()
            if not frame.startswith(b'\xff\xd8'):
                raise RuntimeError('Camera JPEG unavailable')
            (directory/(label+'-'+role+'.jpg')).write_bytes(frame)
        (directory/(label+'-state.json')).write_text(json.dumps({'timestamp':time.time(),**self.state()},indent=2))

    def cancel(self):
        self.latched = True
        if self.active is not None:
            self.active.cancel_goal_async()

    def monitor(self):
        while rclpy.ok():
            if self.controllers.service_is_ready():
                if self.controller_check is not None and self.controller_check.done():
                    response=self.controller_check.result()
                    active={c.name for c in response.controller if c.state=='active'} if response else set()
                    self.controllers_active={'joint_state_broadcaster','joint_trajectory_controller','gripper_controller'}.issubset(active)
                    self.controllers_checked_at=time.monotonic();self.controller_check=None
                if self.controller_check is None and time.monotonic()-self.controllers_checked_at>1:
                    self.controller_check=self.controllers.call_async(ListControllers.Request())
            if self.active is not None and (not self.fresh() or time.monotonic()>self.command_deadline):
                self.cancel()
            if self.fresh():
                positions=self.positions.copy();mapped=self.model_positions(positions)
                rows=[]
                for n in JOINTS:
                    name=n.removesuffix('_joint');cal=self.cal[name]
                    ticks=round(positions[n]*4096/(2*math.pi))+2048
                    fraction=min(1,max(0,(ticks-cal['range_min'])/(cal['range_max']-cal['range_min'])))
                    rows.append({'name':name,'degrees':math.degrees(mapped[n]),'ticks':ticks,
                                 'normalized':fraction*100 if name=='gripper' else fraction*200-100,
                                 'range_min':cal['range_min'],'range_max':cal['range_max'],
                                 'in_range':cal['range_min']<=ticks<=cal['range_max']})
                data={'timestamp':time.time(),'xyz_m':claw_position(self.model,mapped),'joints':rows,
                      'torque_off':False,'within_model_limits':all(float(j.find('limit').get('lower'))<=mapped[j.get('name')]<=float(j.find('limit').get('upper')) for j in self.model.findall('joint') if j.get('name') in mapped),'source':'ROS hardware feedback'}
                tmp=self.args.telemetry.with_suffix('.tmp');tmp.write_text(json.dumps(data));tmp.replace(self.args.telemetry)
            time.sleep(.1)

    def step(self, joint=None, delta=None, deltas=None, duration=1.5, coordinated=False):
        deltas = deltas if deltas is not None else {joint: delta}
        deltas = {n: float(v) for n, v in deltas.items()}
        if not deltas or any(n not in JOINTS or not math.isfinite(v) or abs(v)>(2.3 if coordinated else .12) for n,v in deltas.items()):
            raise ValueError('Each joint step must be finite and at most 0.12 rad')
        if not math.isfinite(duration) or not 1.5<=duration<=8 or (coordinated and max(abs(v) for v in deltas.values())*1.875/duration>.7):
            raise ValueError('Trajectory duration or peak speed outside commissioning bounds')
        if 'gripper_joint' in deltas and len(deltas)>1:
            raise ValueError('Gripper moves separately from the arm')
        joint = next(iter(deltas))
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('Motion busy')
        directory=self.args.records/(str(time.time_ns())+'-'+joint)
        directory.mkdir(parents=True,mode=0o700)
        result={'requested_deltas_rad':deltas,'success':False}
        try:
            self.ready();start=self.positions.copy()
            for n in JOINTS:
                cal=self.cal[n.removesuffix('_joint')];ticks=round(start[n]*4096/(2*math.pi))+2048
                if not cal['range_min']-2<=ticks<=cal['range_max']+2:
                    raise RuntimeError('Current joint outside recorded travel: '+n)
            desired=start.copy()
            for n, change in deltas.items():
                desired[n] += change
                cal=self.cal[n.removesuffix('_joint')]
                ticks=round(desired[n]*4096/(2*math.pi))+2048
                if not cal['range_min']<=ticks<=cal['range_max']:
                    raise RuntimeError('Target outside measured travel: '+n)
            for fraction in np.linspace(0,1,100):
                intermediate={n:start[n]+fraction*(desired[n]-start[n]) for n in JOINTS}
                predicted=claw_position(self.model,self.model_positions(intermediate))
                if joint!='gripper_joint' and predicted['z']<self.args.minimum_claw_height:
                    raise RuntimeError('Predicted claw is too close to the tabletop')
            target=desired[joint]
            result.update(initial=start,targets={n:desired[n] for n in deltas},predicted_claw_m=predicted)
            self.capture(directory,'before')
            if not self.fresh():raise RuntimeError('Feedback stale before command')
            if joint=='gripper_joint':
                client=self.gripper;goal=ParallelGripperCommand.Goal()
                goal.command.name=['gripper_joint'];goal.command.position=[target]
            else:
                client=self.arm;goal=FollowJointTrajectory.Goal()
                goal.trajectory.joint_names=JOINTS[:-1]
                point=JointTrajectoryPoint();point.positions=[desired[n] for n in JOINTS[:-1]]
                point.velocities=[0.0]*5;point.accelerations=[0.0]*5
                point.time_from_start.sec=int(duration);point.time_from_start.nanosec=int((duration-int(duration))*1e9)
                initial_point=JointTrajectoryPoint()
                initial_point.positions=[start[n] for n in JOINTS[:-1]]
                initial_point.velocities=[0.0]*5;initial_point.accelerations=[0.0]*5
                goal.trajectory.points=[initial_point,point]
                for n in JOINTS[:-1]:
                    tol=JointTolerance();tol.name=n;tol.position=.025;goal.goal_tolerance.append(tol)
                goal.goal_time_tolerance.sec=1
            handle=self.wait(client.send_goal_async(goal),.75)
            if not handle.accepted:raise RuntimeError('Goal rejected')
            self.active=handle;self.command_deadline=time.monotonic()+duration+1.5;self.trace=[]
            pending=handle.get_result_async()
            while not pending.done() and time.monotonic()<self.command_deadline:
                if not self.fresh() or any(abs(self.positions[n]-start[n])>.04 for n in JOINTS if n not in deltas):
                    self.cancel();raise RuntimeError('Feedback lost or unexpected other-joint movement')
                time.sleep(.02)
            response=self.wait(pending,.2)
            result.update(action_status=response.status,final=self.positions.copy(),tracking_errors_rad={n:self.positions[n]-desired[n] for n in deltas})
            if response.status!=4 or any(abs(self.positions[n]-desired[n])>.025 for n in deltas):
                observed = {n: {'position_rad': self.positions[n],
                               'requested_rad': desired[n],
                               'achieved_delta_rad': self.positions[n]-start[n],
                               'direction': 1 if deltas[n]>0 else -1}
                            for n in deltas if abs(self.positions[n]-desired[n])>.025}
                # Undertravel may be payload sag or contact, not a mechanical stop.
                # Preserve the calibration and record a contextual candidate limit.
                event={'timestamp':time.time(),'candidate_limits':observed,
                       'context_positions':self.positions.copy(),
                       'classification':'undertravel; contact/load/stall unconfirmed',
                       'record_directory':str(directory)}
                with (self.args.records/'observed-movement-limits.jsonl').open('a') as stream:
                    stream.write(json.dumps(event)+'\n')
                result['observed_movement_limits']=event
                raise RuntimeError('Requested position not achieved')
            result['success']=True
            return result
        except Exception as error:
            if self.active is not None:self.active.cancel_goal_async()
            result['error']=str(error)
            raise
        finally:
            self.active=None
            try:self.capture(directory,'after')
            except Exception as error:result['capture_error']=str(error)
            (directory/'result.json').write_text(json.dumps(result,indent=2))
            (directory/'tracking.json').write_text(json.dumps(self.trace))
            self.lock.release()


    def path(self, poses):
        """Execute a checked arm-only stroke. Caller must verify paper geometry."""
        self.ready()
        if not isinstance(poses, list) or not 1 <= len(poses) <= 1000:
            raise ValueError('Expected 1 to 1000 stroke points')
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('Motion busy')
        directory = self.args.records/(str(time.time_ns())+'-stroke')
        directory.mkdir(parents=True, mode=0o700)
        result = {'success': False}
        try:
            self.ready()
            start = self.positions.copy()
            previous = start.copy()
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS[:-1]
            initial = JointTrajectoryPoint()
            initial.positions = [start[n] for n in JOINTS[:-1]]
            initial.velocities = [0.0]*5
            initial.accelerations = [0.0]*5
            goal.trajectory.points.append(initial)
            elapsed = 0.0
            for item in poses:
                values = item['positions']
                if not values or any(n not in JOINTS[:-1] for n in values):
                    raise ValueError('Stroke points accept arm joints only')
                desired = previous.copy()
                desired.update({n:float(v) for n,v in values.items()})
                duration = float(item['duration'])
                if not math.isfinite(duration) or duration < .15:
                    raise ValueError('Invalid segment duration')
                if max(abs(desired[n]-previous[n]) for n in JOINTS[:-1])*1.875/duration > .7:
                    raise ValueError('Stroke exceeds peak speed bound')
                for pose, slack in ((previous, 2), (desired, 0)):
                    for n in JOINTS:
                        ticks = pose[n]*4096/(2*math.pi)+2048
                        cal = self.cal[n.removesuffix('_joint')]
                        if not math.isfinite(ticks) or not cal['range_min']-slack <= ticks <= cal['range_max']+slack:
                            raise ValueError('Stroke outside recorded travel')
                for fraction in np.linspace(0,1,20):
                    intermediate = {n:previous[n]+fraction*(desired[n]-previous[n]) for n in JOINTS}
                    if claw_position(self.model,self.model_positions(intermediate))['z'] < self.args.minimum_claw_height:
                        raise ValueError('Stroke violates clearance proxy')
                elapsed += duration
                if elapsed > 60:
                    raise ValueError('Stroke exceeds command lease limit')
                point = JointTrajectoryPoint()
                point.positions = [desired[n] for n in JOINTS[:-1]]
                point.velocities = [0.0]*5
                point.accelerations = [0.0]*5
                point.time_from_start.sec = int(elapsed)
                point.time_from_start.nanosec = int((elapsed-int(elapsed))*1e9)
                goal.trajectory.points.append(point)
                previous = desired
            for n in JOINTS[:-1]:
                tolerance = JointTolerance();tolerance.name=n;tolerance.position=.025
                goal.goal_tolerance.append(tolerance)
            goal.goal_time_tolerance.sec = 1
            self.capture(directory,'before')
            self.ready()
            handle = self.wait(self.arm.send_goal_async(goal),.75)
            if not handle.accepted:
                raise RuntimeError('Stroke rejected')
            self.active=handle;self.command_deadline=time.monotonic()+elapsed+1.5;self.trace=[]
            pending=handle.get_result_async()
            while not pending.done() and time.monotonic()<self.command_deadline:
                if self.latched or not self.fresh() or abs(self.positions['gripper_joint']-start['gripper_joint'])>.04:
                    self.cancel();raise RuntimeError('Stroke feedback or fixed-gripper guard failed')
                time.sleep(.02)
            response=self.wait(pending,.2)
            result.update(action_status=response.status,final=self.positions.copy(),duration=elapsed)
            if response.status!=4 or any(abs(self.positions[n]-previous[n])>.025 for n in JOINTS[:-1]):
                raise RuntimeError('Stroke endpoint not achieved')
            result['success']=True
            return result
        except Exception as error:
            if self.active is not None:self.active.cancel_goal_async()
            result['error']=str(error)
            raise
        finally:
            self.active=None
            try:self.capture(directory,'after')
            except Exception as error:result['capture_error']=str(error)
            (directory/'result.json').write_text(json.dumps(result,indent=2))
            (directory/'tracking.json').write_text(json.dumps(self.trace))
            self.lock.release()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--telemetry',type=Path,required=True)
    parser.add_argument('--records',type=Path,required=True)
    parser.add_argument('--camera-url',default='http://127.0.0.1:8765')
    parser.add_argument('--port',type=int,default=8766)
    parser.add_argument('--minimum-claw-height',type=float,default=.035,help='Commissioning height proxy in metres; requires independent camera clearance checks')
    args=parser.parse_args();rclpy.init();session=Session(args)
    class Handler(BaseHTTPRequestHandler):
        def reply(self,data,status=200):
            body=json.dumps(data).encode();self.send_response(status)
            self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(body)))
            self.end_headers();self.wfile.write(body)
        def do_GET(self):self.reply(session.state())
        def do_POST(self):
            try:
                if self.path=='/stop':session.cancel();self.reply(session.state());return
                if self.path not in ('/step','/pose','/path'):self.reply({'error':'Unknown command'},404);return
                data=json.loads(self.rfile.read(int(self.headers.get('Content-Length','0'))))
                if self.path=='/path':
                    self.reply(session.path(data['poses']))
                elif self.path=='/pose':
                    session.ready()
                    deltas={n:float(v)-session.positions[n] for n,v in data['positions'].items()}
                    self.reply(session.step(deltas=deltas,duration=float(data['duration']),coordinated=True))
                else:self.reply(session.step(data.get('joint'),data.get('delta'),data.get('deltas')))
            except Exception as error:self.reply({'error':str(error),**session.state()},409)
        def log_message(self,*_):pass
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print('Local visual learning session ready; ROS retains current-position hold.',flush=True)
    server.serve_forever()


if __name__=='__main__':main()
