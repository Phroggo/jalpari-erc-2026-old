"""Development-only continuation of an already contact-verified grasp.

This is NOT a complete competition trial and is never used by solution.launch.py.
All state comes from ROS sensors; no object pose or attachment is injected.
"""
import time
import numpy as np
import rclpy
from erc_solution.mission import Mission

rclpy.init()
n = Mission()
end = time.monotonic()+8
while time.monotonic()<end:
    n.tick()
held = n.held()
if len(held)!=1:
    raise RuntimeError(f'Diagnostic requires one existing bilateral grasp, got {held}')
n.book_id = next(iter(held))
print('DIAGNOSTIC ONLY: retained book',n.book_id,flush=True)
n.move_joints('torso_controller',['torso_lift_joint'],[.345],1.5,tolerance=.004)
n.wait(.6)
print('After torso lift contacts',n.held(),flush=True)
if n.book_id not in n.held():
    raise RuntimeError('Book contact lost during diagnostic lift')
tip = n.matrix('base_footprint','gripper_right_grasping_link')
tip[0,3] -= .16
n.arm_pose(tip,2.)
n.wait(.6)
print('After extraction contacts',n.held(),flush=True)
n.destroy_node()
rclpy.shutdown()
