"""Read-only RViz outputs. Targets are commands, not a global planner path."""
from collections import deque
import math

from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster

from .localization import world_points


class NavigationView:
    def __init__(self, node):
        self.node = node
        # Register the display frame without publishing a competing robot TF.
        self.frames = StaticTransformBroadcaster(node)
        anchor = TransformStamped()
        anchor.header.frame_id = 'arena'
        anchor.child_frame_id = 'erc_navigation_display'
        anchor.header.stamp = node.get_clock().now().to_msg()
        anchor.transform.rotation.w = 1.
        self.frames.sendTransform(anchor)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = node.create_publisher(Path, '/erc/navigation/executed', qos)
        self.target_pub = node.create_publisher(PoseArray, '/erc/navigation/targets', qos)
        self.scene_pub = node.create_publisher(MarkerArray, '/erc/navigation/scene', qos)
        self.poses = deque(maxlen=4000)
        self.targets = deque(maxlen=100)
        self.last = -float('inf')

    @staticmethod
    def pose(x, y, yaw):
        pose = Pose()
        pose.position.x, pose.position.y = float(x), float(y)
        pose.orientation.z = math.sin(yaw / 2.)
        pose.orientation.w = math.cos(yaw / 2.)
        return pose

    def target(self, xy, yaw):
        self.targets.append(self.pose(*xy, yaw))
        msg = PoseArray()
        msg.header.frame_id = 'arena'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.poses = list(self.targets)
        self.target_pub.publish(msg)

    def update(self, pose, points, stamp):
        # Limit display work and history; these outputs never feed the controller.
        if stamp - self.last < .2:
            return
        self.last = stamp
        sample = PoseStamped()
        sample.header.frame_id = 'arena'
        sample.header.stamp.sec = int(stamp)
        sample.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        sample.pose = self.pose(*pose)
        self.poses.append(sample)
        self.path_pub.publish(Path(header=sample.header, poses=list(self.poses)))
        walls = Marker(header=sample.header, ns='localization_walls', id=0,
                       type=Marker.LINE_STRIP, action=Marker.ADD)
        walls.pose.orientation.w = 1.
        walls.scale.x = .035
        walls.color.r = walls.color.g = walls.color.b = .7
        walls.color.a = 1.
        walls.points = [Point(x=x, y=y, z=0.) for x, y in
                        [(-3.975, -4.975), (5.975, -4.975), (5.975, 4.975),
                         (-3.975, 4.975), (-3.975, -4.975)]]
        scans = Marker(header=sample.header, ns='localized_laser_returns', id=1,
                       type=Marker.POINTS, action=Marker.ADD)
        scans.pose.orientation.w = 1.
        scans.scale.x = scans.scale.y = .04
        scans.color.r, scans.color.g, scans.color.a = 1., .4, 1.
        scans.points = [Point(x=float(x), y=float(y), z=.05)
                        for x, y in world_points(points, pose)]
        self.scene_pub.publish(MarkerArray(markers=[walls, scans]))
