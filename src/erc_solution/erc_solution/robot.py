"""Sensor callbacks, motion commands and checks shared by the mission stages."""
import time
import json
from collections import deque
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState, LaserScan
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32, String
from ros_gz_interfaces.msg import Contacts
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from .kinematics import Kinematics
from .localization import WallLocalizer
from .navigation_view import NavigationView


class Robot(Node):
    def __init__(self):
        super().__init__('erc_book_retrieval')
        self.declare_parameter('shelf_column_number', 2)
        self.declare_parameter('book_colour', 'red')
        self.declare_parameter('images_dir', '/opt/erc_ws/erc_images')
        self.column = self.get_parameter('shelf_column_number').value
        self.colour = self.get_parameter('book_colour').value
        if self.column not in range(1,6) or self.colour not in ('red','green','blue','yellow'):
            raise ValueError('Expected shelf_column_number 1..5 and book_colour red/green/blue/yellow')
        self.images = Path(self.get_parameter('images_dir').value)
        self.images.mkdir(parents=True, exist_ok=True)
        self.trial_wall = time.monotonic()
        self.trial_log = self.images/f'trial_{time.time_ns()}.jsonl'
        self.bridge, self.tf = CvBridge(), Buffer()
        self.listener = TransformListener(self.tf, self)
        self.rgb = self.depth = self.info = None
        self.rgb_queue,self.depth_queue = deque(maxlen=20),deque(maxlen=20)
        self.joints, self.scans, self.contacts, self.bin_hits = {}, {}, {}, {}
        self.joint_stamp = 0.
        self.shelf_contact = (-10.,'')
        self.localizer = WallLocalizer()
        self.localization_stamp = -1.
        self.localization_attempt = -1.
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/erc/mission_status', 10)
        self.column_pub = self.create_publisher(Int32, '/erc/shelf_column_identification', 10)
        self.row_pub = self.create_publisher(Int32, '/erc/shelf_row_identification', 10)
        self.controllers = {}
        self.navigation_view = None
        try:
            self.navigation_view = NavigationView(self)
        except Exception as exc:
            self.get_logger().warning(f'Navigation display unavailable: {exc}')
        camera = '/head_front_camera/head_front_camera/'
        self.create_subscription(Image, camera+'color/image_raw', lambda m: self.rgb_queue.append(m), qos_profile_sensor_data)
        self.create_subscription(Image, camera+'depth/image_rect_raw', lambda m: self.depth_queue.append(m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera+'color/camera_info', lambda m: setattr(self,'info',m), qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        for name in ('front','rear'):
            self.create_subscription(LaserScan, '/scan_'+name+'_raw', lambda m,n=name: self.scans.update({n:m}), qos_profile_sensor_data)
        self.create_subscription(Contacts, '/contacts', self.on_contacts, qos_profile_sensor_data)
        self.create_subscription(Contacts, '/bin_contacts', self.on_bin, qos_profile_sensor_data)
        description = Path(get_package_share_directory('erc_description'))/'urdf/tiago_pro.urdf'
        self.arm = Kinematics(description)
        self.park_arm = Kinematics(description,tip='gripper_left_grasping_link')

    def park(self):
        """Fold both arms before driving; only the right arm handles the book."""
        self.state('COMPACT_STARTUP_POSTURE')
        for model,side,controller in ((self.arm,-1,'arm_right_controller'),
                                      (self.park_arm,1,'arm_left_controller')):
            raised = model.startup_waypoint([self.joints[n] for n in model.names],side)
            self.move_joints(controller,model.names,raised,4.,timeout=20.)
            q = model.compact([self.joints[n] for n in model.names],side)
            current = np.array([self.joints[n] for n in model.names])
            lowest = min(model.points(current+(q-current)*t)[:,2].min() for t in np.linspace(0,1,41))
            if lowest<-.70:
                raise RuntimeError('Parking path would approach the floor')
            self.move_joints(controller,model.names,q,4.,timeout=20.)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def on_joints(self, msg):
        self.joints.update(zip(msg.name,msg.position))
        self.joint_stamp = self.stamp(msg)

    def on_contacts(self, msg):
        """Track shelf contact and which finger sides touch each book."""
        stamp = self.stamp(msg)
        for c in msg.contacts:
            a,b = c.collision1.name,c.collision2.name
            if ('tiago_pro::' in a and 'erc_shelf::' in b) or ('tiago_pro::' in b and 'erc_shelf::' in a):
                self.shelf_contact = (stamp,a+' <-> '+b)
            for finger,other in ((a,b),(b,a)):
                if ('gripper_right' in finger and 'book' in other
                        and ('_left_link::' in finger or '_right_link::' in finger)):
                    model = other.split('::')[0]
                    self.contacts[(model, finger)] = stamp

    def on_bin(self, msg):
        for c in msg.contacts:
            for name in (c.collision1.name,c.collision2.name):
                if 'book' in name:
                    self.bin_hits[name.split('::')[0]] = self.stamp(msg)

    def held(self):
        """A hold needs recent contact on both sides of the gripper."""
        models = {}
        for (book,finger),stamp in self.contacts.items():
            if 0 <= self.now()-stamp < .5:
                models.setdefault(book,set()).add('left' if 'left' in finger else 'right')
        return {book for book,sides in models.items() if len(sides)==2}

    def require_held(self, book, stage, timeout=2.):
        """Stop and recheck both fingers; brief contact gaps happen after braking."""
        self.cmd.publish(Twist())
        start,wall,stable = self.now(),time.monotonic(),None
        while self.now()-start < timeout and time.monotonic()-wall < timeout*30:
            self.tick()
            if book in self.held():
                stable = self.now() if stable is None else stable
                if self.now()-stable >= .25:
                    return
            else:
                stable = None
        raise RuntimeError(f'Book contact not confirmed {stage}')

    def tick(self):
        if not rclpy.ok():
            raise RuntimeError('ROS shutdown')
        rclpy.spin_once(self, timeout_sec=.02)
        self.localize()

    def localize(self):
        """Combine both laser scans and update the arena pose once per scan."""
        if len(self.scans)!=2:
            return
        stamp = min(self.stamp(s) for s in self.scans.values())
        if stamp <= self.localization_attempt:
            return
        self.localization_attempt = stamp
        points = []
        for scan in self.scans.values():
            ranges = np.asarray(scan.ranges)
            angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment
            valid = np.isfinite(ranges)&(ranges>.35)&(ranges<scan.range_max)
            p = np.c_[ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(sum(valid))][::3]
            try:
                m = self.matrix('base_footprint',scan.header.frame_id)
            except TransformException:
                return
            points.append((p @ m[:3,:3].T+m[:3,3])[:,:2])
        if self.localizer.update(np.concatenate(points)) is not None:
            self.localization_stamp = stamp
            if getattr(self, 'heading_initialized', False):
                self.show_navigation('update', self.localizer.pose,
                                     np.concatenate(points), stamp)

    def show_navigation(self, method, *args):
        """A broken optional display must not change mission control."""
        if self.navigation_view is None:
            return
        try:
            getattr(self.navigation_view, method)(*args)
        except Exception as exc:
            self.get_logger().warning(f'Navigation display disabled: {exc}')
            self.navigation_view = None

    def wait(self, seconds):
        start, wall = self.now(), time.monotonic()
        while self.now()-start < seconds:
            self.tick()
            if time.monotonic()-wall > max(30,seconds*30):
                raise RuntimeError('Simulation clock stalled')

    def state(self, name):
        """Stop the base, announce the stage and append its timing to the log."""
        self.cmd.publish(Twist())
        self.get_logger().info(name)
        self.status.publish(String(data=name))
        book = getattr(self,'book_id',None)
        event = {'state':name, 'sim_time':self.now(),
                 'wall_elapsed':time.monotonic()-self.trial_wall,
                 'utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                 'column':self.column,'colour':self.colour,'book_contact_id':book}
        if book in self.bin_hits:
            event['bin_contact_sim_time'] = self.bin_hits[book]
        with self.trial_log.open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(event)+'\n')

    def matrix(self, target, source, stamp=None):
        """Transform source into target; arena pose comes from the lasers."""
        if 'arena' in (target,source):
            if self.localizer.pose is None or self.now()-self.localization_stamp>.6:
                raise TransformException('Laser arena localisation unavailable or stale')
            x,y,yaw = self.localizer.pose
            base = np.eye(4)
            base[:3,:3] = Rotation.from_euler('z',yaw).as_matrix()
            base[:2,3] = [x,y]
            if target == 'arena':
                return base @ self.matrix('base_footprint',source,stamp)
            return self.matrix(target,'base_footprint',stamp) @ np.linalg.inv(base)
        t = self.tf.lookup_transform(target, source, Time() if stamp is None else Time.from_msg(stamp)).transform
        result = np.eye(4)
        result[:3,:3] = Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
        result[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
        return result

    def camera(self, frame='arena'):
        """Pair fresh RGB/depth frames and look up the camera pose at capture time."""
        if not self.rgb_queue or not self.depth_queue or self.info is None:
            return None
        pose = None
        for rgb in reversed(self.rgb_queue):
            if self.now()-self.stamp(rgb)>.5:
                break
            depth = min(self.depth_queue,key=lambda d:abs(self.stamp(d)-self.stamp(rgb)))
            if abs(self.stamp(rgb)-self.stamp(depth))>.04:
                continue
            try:
                # Official URDF renders both sensors at camera_link (zero
                # sensor pose), although the colour TF advertises a 15 mm
                # optical offset. Registered pixels therefore use depth TF.
                pose = self.matrix(frame,depth.header.frame_id,rgb.header.stamp)
            except TransformException:
                continue
            self.rgb,self.depth = rgb,depth
            break
        if pose is None:
            return None
        bgr = self.bridge.imgmsg_to_cv2(self.rgb, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(self.depth, 'passthrough').astype(float)
        if self.depth.encoding == '16UC1':
            depth *= .001
        k = self.info.k
        return bgr,depth,(k[0],k[4],k[2],k[5]),pose

    def evidence(self, label, bgr, box, point=None, row=None):
        """Save the live image with a box and timestamp, plus its location metadata."""
        annotated = bgr.copy()
        x,y,w,h = map(int,box)
        cv2.rectangle(annotated,(x,y),(x+w,y+h),(0,255,0),2)
        text = f'{label} | sim {self.now():.3f}s | UTC '+time.strftime('%Y-%m-%d %H:%M:%S',time.gmtime())
        cv2.rectangle(annotated,(0,0),(annotated.shape[1],25),(0,0,0),-1)
        cv2.putText(annotated,text,(5,18),cv2.FONT_HERSHEY_SIMPLEX,.42,(255,255,255),1)
        path = self.images/f'{label}_{time.time_ns()}.jpg'
        if not cv2.imwrite(str(path),annotated):
            raise RuntimeError(f'Cannot save evidence: {path}')
        record = {'event': 'visual_evidence', 'label': label,
                  'image_path': str(path), 'sim_time': self.now(),
                  'column': self.column, 'colour': self.colour,
                  'box_xywh': [x, y, w, h]}
        if row is not None:
            record['row'] = int(row)
        if point is not None:
            record.update(frame='arena', detected_point=[float(v) for v in point])
        # Keep image metadata beside the JPG so trial JSONL stays stage-only.
        path.with_suffix('.json').write_text(json.dumps(record, indent=2) + '\n')

    def trajectory(self, controller, names, positions, duration):
        if controller not in self.controllers:
            self.controllers[controller] = self.create_publisher(JointTrajectory,'/'+controller+'/joint_trajectory',10)
            deadline = time.monotonic()+5
            while self.controllers[controller].get_subscription_count()==0:
                self.tick()
                if time.monotonic()>deadline:
                    raise RuntimeError(f'Controller topic unavailable: {controller}')
        msg = JointTrajectory(joint_names=names)
        point = JointTrajectoryPoint(positions=[float(v) for v in positions])
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration-int(duration))*1e9)
        msg.points = [point]
        self.controllers[controller].publish(msg)

    def move_joints(self, controller, names, target, duration=3., tolerance=.035, timeout=15.):
        """Send a pose and wait for joint feedback to show it has settled."""
        self.trajectory(controller,names,target,duration)
        start,wall,last = self.now(),time.monotonic(),-1.
        stable = None
        while self.now()-start < timeout and time.monotonic()-wall < timeout*30:
            self.tick()
            if controller.startswith('arm_') and self.now()-self.shelf_contact[0]<.25:
                raise RuntimeError('Arm motion stopped on shelf contact: '+self.shelf_contact[1])
            if self.now()-self.joint_stamp > .5:
                continue
            if self.now()-start > duration and all(n in self.joints for n in names):
                err = max(abs(self.joints[n]-v) for n,v in zip(names,target))
                if err < tolerance:
                    stable = self.now() if stable is None else stable
                    if self.now()-stable > .3:
                        return
                else:
                    stable = None
                if self.now()-last > 3:
                    self.get_logger().info(f'{controller}: measured joint error {err:.3f}')
                    last = self.now()
        raise RuntimeError(f'{controller} did not reach its measured target')

    def grip(self, close=False):
        # The book stops the jaws early, so check contact instead of full closure.
        for q in ([.03,.024,.020,.017] if close else [.06]):
            self.trajectory('gripper_right_controller',['gripper_right_finger_joint'],[q],.8)
            self.wait(1.1)
            if close and self.held():
                # Initial contact alone supplies essentially no clamping force.
                # Add a small jaw preload before attempting to support 300 g.
                measured = self.joints['gripper_right_finger_joint']
                preload = max(.017,measured-.002)
                self.trajectory('gripper_right_controller',['gripper_right_finger_joint'],[preload],.8)
                self.wait(1.1)
                return

    def arm_pose(self, goal_base, duration=3., joint_speed=None):
        """Solve the arm pose, move there and check the measured tool position."""
        goal = self.matrix('torso_lift_link','base_footprint') @ goal_base
        current = [self.joints[n] for n in self.arm.names]
        q = self.arm.solve(goal,current)
        if joint_speed is not None:
            if not np.isfinite(joint_speed) or joint_speed <= 0:
                raise ValueError('Joint speed must be positive and finite')
            # Small carrying steps need less time. Big joint turns still get
            # time to track instead of rushing through a fixed short deadline.
            duration = max(duration, float(np.max(np.abs(np.asarray(q)-current)))/joint_speed)
        self.move_joints('arm_right_controller',self.arm.names,q,duration,
                         tolerance=.025,timeout=max(15.,duration+5.))
        actual = self.matrix('base_footprint','gripper_right_grasping_link')
        error = np.linalg.norm(actual[:3,3]-goal_base[:3,3])
        angle = Rotation.from_matrix(goal_base[:3,:3] @ actual[:3,:3].T).magnitude()
        if error > .015 or angle > .075:
            raise RuntimeError(f'Measured tool misalignment: {error:.3f} m / {angle:.3f} rad')

    def velocity(self,x=0.,y=0.,yaw=0.):
        # Both scans must remain live; obstacle checks cover the direction of translation.
        if max(abs(x),abs(y),abs(yaw)) > 0:
            if x>=0 and self.now()-self.shelf_contact[0]<.25:
                self.cmd.publish(Twist())
                raise RuntimeError('Base motion stopped on shelf contact: '+self.shelf_contact[1])
            for name in ('front','rear'):
                scan = self.scans.get(name)
                if scan is None or self.now()-self.stamp(scan) > .5:
                    self.cmd.publish(Twist())
                    raise RuntimeError('Missing or stale safety laser scan')
                angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
                try:
                    m = self.matrix('base_footprint',scan.header.frame_id)
                except TransformException:
                    self.cmd.publish(Twist())
                    raise RuntimeError('Missing safety laser transform')
                ranges = np.array(scan.ranges)
                valid = np.isfinite(ranges)&(ranges>scan.range_min)&(ranges<scan.range_max)
                pts = np.c_[ranges[valid]*np.cos(angles[valid]),ranges[valid]*np.sin(angles[valid]),np.zeros(sum(valid))]
                pts = pts @ m[:3,:3].T+m[:3,3]
                if np.hypot(x,y) > .001:
                    direction = np.array([x,y])/np.hypot(x,y)
                    along = pts[:,:2] @ direction
                    side = abs(pts[:,0]*direction[1]-pts[:,1]*direction[0])
                    if np.any((along>0)&(along<.39)&(side<.34)):
                        self.cmd.publish(Twist())
                        nearest = pts[(along>0)&(along<.39)&(side<.34)][0]
                        raise RuntimeError(f'Obstacle inside base stopping clearance: {nearest}')
        msg = Twist()
        msg.linear.x,msg.linear.y,msg.angular.z = float(x),float(y),float(yaw)
        self.cmd.publish(msg)

    def navigate(self, xy, yaw, timeout=80.):
        """Drive towards an arena pose, stopping if laser clearance is too small."""
        self.show_navigation('target', xy, yaw)
        start,wall = self.now(),time.monotonic()
        last_log = start
        try:
            while self.now()-start < timeout and time.monotonic()-wall < timeout*30:
                self.tick()
                pose = self.matrix('arena','base_footprint')
                angle = np.arctan2(pose[1,0],pose[0,0])
                turn = np.arctan2(np.sin(yaw-angle),np.cos(yaw-angle))
                delta = pose[:2,:2].T @ (np.asarray(xy)-pose[:2,3])
                if self.now()-last_log>5:
                    self.get_logger().info(f'Navigation remaining: {delta}, yaw {turn:.3f}')
                    last_log = self.now()
                if np.linalg.norm(delta)<.035 and abs(turn)<.03:
                    return
                scale = min(1.,.16/max(np.linalg.norm(delta),.001))
                self.velocity(*(delta*scale if abs(turn)<.15 else [0.,0.]),yaw=np.clip(turn,-.25,.25))
        finally:
            self.cmd.publish(Twist())
        raise RuntimeError('Navigation timed out')
