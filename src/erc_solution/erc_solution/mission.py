"""Run the trial: find the column, pick the book, then bring it to the bin."""
import time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory
from .robot import Robot
from .vision import objects, project, projected_box, row_for_height
from .kinematics import transform, transport_path
from .perception.marker_detector import MarkerDetector


class Mission(Robot):
    def __init__(self):
        super().__init__()
        weights = Path(get_package_share_directory('erc_solution'))/'models/digit_mlp.npz'
        self.markers = MarkerDetector(str(weights), min_conf=.85)
        self.book_id = None
        self.heading_initialized = False

    def observe_marker(self):
        """Read the target digit and return its position in the arena frame."""
        camera = self.camera('base_footprint')
        if camera is None:
            return None
        bgr,depth,k,pose = camera
        valid = []
        for d in self.markers.detect(bgr, roi_frac=1.):
            p = project(depth,k,d.u,d.v)
            if p is not None:
                p = (pose @ np.r_[p,1])[:3]
                if abs(p[2]-2.26)<.18:
                    valid.append((d,p))
        target = [v for v in valid if v[0].digit == self.column]
        if len(target)!=1 or len(valid)<2:
            return None
        ends = sorted(valid,key=lambda v:v[0].u)
        tangent = ends[0][1]-ends[-1][1]
        tangent[2] = 0
        tangent /= np.linalg.norm(tangent)
        normal = np.array([tangent[1],-tangent[0],0.])
        base = np.zeros(3)
        if np.dot(normal,target[0][1]-base)<0:
            normal *= -1
        if not self.heading_initialized:
            # The walls look the same in four directions. Use the visible shelf
            # to work out which way the robot is actually facing.
            self.heading_initialized = self.localizer.initialize_shelf(target[0][1],normal)
            if not self.heading_initialized:
                return None
            self.localization_stamp = self.localization_attempt
        world = self.matrix('arena','base_footprint')
        # Draw the box around the column. Start from the marker we just saw
        # and use the shelf dimensions to project its edges into the image.
        corners = []
        camera_from_base = np.linalg.inv(pose)
        for side in (-.50,.50):
            for height in (.17,2.35):
                corner = target[0][1]+side*tangent
                corner[2] = height
                corners.append((camera_from_base @ np.r_[corner,1])[:3])
        box = projected_box(corners,k,bgr.shape)
        if box is None:
            return None
        return (world @ np.r_[target[0][1],1])[:3],world[:3,:3] @ normal,bgr,box

    def find_column(self):
        """Turn until the requested marker stays clear across five camera frames."""
        self.state('SEARCH_COLUMN')
        self.move_joints('head_controller',['head_1_joint','head_2_joint'],[0.,.15],2.)
        start,wall,stable,previous = self.now(),time.monotonic(),0,None
        last_log = start
        last_seen,last_image = -10.,-1.
        try:
            while self.now()-start<90 and time.monotonic()-wall<1800:
                self.tick()
                seen = self.observe_marker()
                if self.now()-last_log>5:
                    self.get_logger().info(f'Search: camera={self.camera("base_footprint") is not None}, target={seen is not None}, stable={stable}, laser={self.localizer.pose}')
                    last_log = self.now()
                if seen is not None:
                    self.velocity()
                    last_seen = self.now()
                    if self.stamp(self.rgb)==last_image:
                        self.wait(.03)
                        continue
                    last_image = self.stamp(self.rgb)
                    stable = stable+1 if previous is not None and np.linalg.norm(seen[0]-previous)<.06 else 1
                    previous = seen[0]
                    if stable>=5:
                        self.column_pub.publish(Int32(data=self.column))
                        self.evidence(f'column_{self.column}',seen[2],seen[3],seen[0])
                        self.get_logger().info(f'Visual column {self.column}: {seen[0]}, normal {seen[1]}')
                        return seen[:2]
                else:
                    if self.now()-last_seen>1.:
                        stable = 0
                        self.velocity(yaw=-.16)
                    else:
                        self.velocity()
                self.wait(.12)
        finally:
            self.velocity()
        raise RuntimeError('Cannot identify requested marker consistently; no physical-column fallback')

    def observe_book(self, marker, normal):
        """Keep only a matching colour inside the column we identified."""
        camera = self.camera()
        if camera is None:
            return None
        bgr,depth,k,pose = camera
        tangent = np.array([-normal[1],normal[0],0.])
        candidates = []
        for d in objects(bgr,depth,k,self.colour):
            p = (pose @ np.r_[d.point,1])[:3]
            delta = p-marker
            row = row_for_height(p[2])
            if row is not None and abs(delta @ tangent)<.39 and abs(delta @ normal)<.25:
                candidates.append((p,row,bgr,d.box))
        return candidates[0] if len(candidates)==1 else None

    def find_book(self, marker, normal):
        """Look through the shelf heights and save the book's row and image."""
        self.state('IDENTIFY_BOOK')
        for tilt in (.3,0.,-.3,-.6):
            self.move_joints('head_controller',['head_1_joint','head_2_joint'],[0.,tilt],2.)
            previous,stable = None,0
            for _ in range(30):
                self.wait(.1)
                seen = self.observe_book(marker,normal)
                if seen is None:
                    stable = 0
                    continue
                stable = stable+1 if previous is not None and np.linalg.norm(seen[0]-previous)<.025 else 1
                previous = seen[0]
                if stable>=4:
                    self.row_pub.publish(Int32(data=seen[1]))
                    self.evidence(f'row_{seen[1]}_{self.colour}',seen[2],seen[3],seen[0],row=seen[1])
                    self.get_logger().info(f'Visual book row {seen[1]}: {seen[0]}')
                    return seen[0],seen[1]
        raise RuntimeError('Target book not uniquely visible in the requested column')

    def pick(self, book, normal, row):
        """Line up, close on the book, then check the hold before pulling out."""
        self.state('ALIGN_FOR_GRASP')
        yaw = np.arctan2(normal[1],normal[0])
        tangent = np.array([-normal[1],normal[0],0.])
        reach = {1:.55,2:.72,3:.75,4:.85}[row]
        # Unfold away from the shelf. The arm can hit a board on the way to
        # a pose even if that final pose itself is clear.
        self.navigate((book-1.40*normal+.24*tangent)[:2],yaw)
        torso = {1:.345,2:.20,3:.0,4:.0}[row]
        self.move_joints('torso_controller',['torso_lift_joint'],[torso],3.,tolerance=.008)
        self.grip(False)
        self.state('DEPLOY_IN_CLEAR_SPACE')
        self.arm_pose(transform([reach-.09,-.24,book[2]]),4.)
        self.state('APPROACH_WITH_ALIGNED_GRIPPER')
        self.navigate((book-reach*normal+.24*tangent)[:2],yaw)
        p = (self.matrix('base_footprint','arena') @ np.r_[book,1])[:3]
        pre = transform(p+[-.09,0.,0.])
        # Check the lift and pull-out poses before closing. We need a way
        # back out once the book is in the gripper.
        seed = [self.joints[n] for n in self.arm.names]
        torso_from_base = self.matrix('torso_lift_link','base_footprint')
        for dx,dz in ((0.,0.),(.15,0.),(.15,.025),(-.05,.025)):
            planned = pre.copy()
            planned[0,3] += dx
            planned[2,3] += dz
            seed = self.arm.solve(torso_from_base @ planned,seed)
        self.arm_pose(pre,4.)
        self.state('INSERT_GRIPPER')
        # Seat the pad beyond the spine edge, not just its leading corner.
        for advance in np.linspace(0,.15,11)[1:]:
            goal = pre.copy()
            goal[0,3] += advance
            self.arm_pose(goal,.7)
        self.state('CLOSE_AND_VERIFY')
        self.grip(True)
        held = self.held()
        if len(held)!=1:
            raise RuntimeError('No unique book in bilateral finger contact; refusing to lift air')
        self.book_id = next(iter(held))
        self.get_logger().info(f'Bilateral book contact verified: {self.book_id}')
        lifted = goal.copy()
        lifted[2,3] += .025
        self.arm_pose(lifted,1.5)
        self.wait(.6)
        self.require_held(self.book_id,'after test lift')
        self.state('EXTRACT_BOOK')
        retracted = lifted.copy()
        retracted[0,3] -= .20
        self.arm_pose(retracted,2.)
        self.navigate(self.matrix('arena','base_footprint')[:2,3]-.28*normal[:2],yaw)
        self.require_held(self.book_id,'after shelf extraction')

    def find_bin(self):
        """Face the bin's expected area, then locate it with the camera."""
        self.state('IDENTIFY_COLLECTION_BIN')
        base = self.matrix('arena','base_footprint')[:2,3]
        prior = np.array([-1.,0.])-base
        self.navigate(base,np.arctan2(prior[1],prior[0]))
        self.move_joints('head_controller',['head_1_joint','head_2_joint'],[0.,-.65],2.)
        start,wall,previous,stable = self.now(),time.monotonic(),None,0
        last_seen,last_image = -10.,-1.
        try:
            while self.now()-start<70 and time.monotonic()-wall<1400:
                self.tick()
                camera,seen = self.camera(),[]
                if camera is not None:
                    bgr,depth,k,pose = camera
                    for d in objects(bgr,depth,k,'red',bin_mode=True,height_transform=pose):
                        p = (pose @ np.r_[d.point,1])[:3]
                        if .72 < p[2] < 1.02 and -1.5 < p[0] < -.5 and abs(p[1])<.5:
                            seen.append((p,d))
                if len(seen)==1:
                    p,d = seen[0]
                    self.velocity()
                    last_seen = self.now()
                    if self.stamp(self.rgb)==last_image:
                        self.wait(.03)
                        continue
                    last_image = self.stamp(self.rgb)
                    stable = stable+1 if previous is not None and np.linalg.norm(p-previous)<.04 else 1
                    previous = p
                    if stable>=5:
                        self.evidence('collection_bin',bgr,d.box,p)
                        return p
                else:
                    if self.now()-last_seen>1.:
                        stable = 0
                        self.velocity(yaw=.12)
                    else:
                        self.velocity()
                self.wait(.12)
        finally:
            self.velocity()
        raise RuntimeError('Collection bin could not be identified visually')

    def prepare_transport(self):
        """Raise lower-row books after extraction, before approaching the table."""
        tip = self.matrix('base_footprint','gripper_right_grasping_link')
        if tip[2,3] >= 1.20:
            return
        self.state('RAISE_BOOK_IN_CLEAR_SPACE')
        self.move_joints('torso_controller',['torso_lift_joint'],[.345],4.,tolerance=.008)
        self.require_held(self.book_id,'after raising torso for transport')
        tip = self.matrix('base_footprint','gripper_right_grasping_link')
        # The torso alone can clear the table. Don't move a held book through
        # another arm pose when it's already at our carrying height.
        if tip[2,3] >= 1.20:
            return
        goals = transport_path(tip[:3,3])
        seed = [self.joints[n] for n in self.arm.names]
        root = self.matrix('torso_lift_link','base_footprint')
        for goal in goals:
            seed = self.arm.solve(root @ goal,seed)
        for goal in goals:
            # Don't spend 1.8 s on every tiny step. Keep extra time for large
            # joint turns: a blanket 1 s deadline previously lost a book.
            self.arm_pose(goal,.7,joint_speed=.30)
            self.require_held(self.book_id,'while raising arm for transport')

    def lower_over_bin(self, position):
        """Lower up to 9 cm in three steps, stopping if the book touches the bin."""
        self.state('LOWER_INTO_BIN')
        started = self.now()
        goals = []
        for distance in np.linspace(.03, .09, 3):
            point = position.copy()
            point[2] -= distance
            goals.append(transform(point))
        # Check every waypoint before starting the descent.
        root = self.matrix('torso_lift_link', 'base_footprint')
        seed = [self.joints[n] for n in self.arm.names]
        for goal in goals:
            seed = self.arm.solve(root @ goal, seed)
        for goal in goals:
            if self.bin_hits.get(self.book_id, -1) >= started:
                break
            self.require_held(self.book_id, 'while lowering over bin')
            self.arm_pose(goal, 1.0)
            self.wait(.15)
        return started

    def deliver(self, start):
        """Return to start, release over the bin and check matching book contact."""
        self.prepare_transport()
        self.state('RETURN_WITH_BOOK')
        here = self.matrix('arena','base_footprint')
        self.navigate(start[:2,3],np.arctan2(here[1,0],here[0,0]))
        self.require_held(self.book_id,'after return')
        bin_point = self.find_bin()
        delta = bin_point-self.matrix('arena','base_footprint')[:3,3]
        self.navigate(start[:2,3],np.arctan2(delta[1],delta[0]))
        self.state('POSITION_ABOVE_BIN')
        p = (self.matrix('base_footprint','arena') @ np.r_[bin_point,1])[:3]
        p[2] = 1.13  # vertical book's bottom must clear the 0.96 m bin rim
        self.arm_pose(transform(p),4.)
        self.require_held(self.book_id,'before release')
        placement_started = self.lower_over_bin(p)
        self.state('RELEASE_BOOK')
        self.grip(False)
        for _ in range(80):
            # Gentle placement can make contact before the fingers open.
            if self.bin_hits.get(self.book_id,-1)>=placement_started:
                self.state('SUCCESS: held book contacted collection bin')
                return
            self.wait(.1)
        raise RuntimeError('Released book did not produce matching /bin_contacts evidence')

    def run(self):
        """Wait for sensors, then run each stage in order."""
        self.state('WAIT_FOR_SENSORS')
        deadline = time.monotonic()+90
        while time.monotonic()<deadline:
            self.tick()
            if self.camera() is not None and all(n in self.joints for n in self.arm.names) and len(self.scans)==2:
                break
        else:
            raise RuntimeError('RGB-D, joint states or laser scans unavailable')
        start = np.eye(4)  # prescribed Start/End zone, not drifting wheel origin
        self.park()
        marker,normal = self.find_column()
        yaw = np.arctan2(normal[1],normal[0])
        self.state('APPROACH_COLUMN')
        self.navigate((marker-1.15*normal)[:2],yaw)
        book,row = self.find_book(marker,normal)
        self.pick(book,normal,row)
        self.deliver(start)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mission()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node:
            node.state(f'FAILED: {exc}')
            if all(n in node.joints for n in node.arm.names):
                node.trajectory('arm_right_controller',node.arm.names,
                                [node.joints[n] for n in node.arm.names],.3)
                node.wait(.5)
        raise
    finally:
        if node:
            node.cmd.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
