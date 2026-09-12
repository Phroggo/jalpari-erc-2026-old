"""Stop a stalled arm at its measured joints, inspect contacts and candidate IK.

Development-only diagnostic, never launched by the competition solution.
"""
import time
import numpy as np
import rclpy
from erc_solution.robot import Robot
from erc_solution.kinematics import transform
from ros_gz_interfaces.msg import Contacts

rclpy.init()
n = Robot()
contacts = set()
def report(msg):
    for c in msg.contacts:
        if 'arm_right' in c.collision1.name+c.collision2.name:
            contacts.add((c.collision1.name,c.collision2.name))
n.create_subscription(Contacts,'/contacts',report,10)
end = time.monotonic()+5
while time.monotonic()<end:
    n.tick()
q = np.array([n.joints[j] for j in n.arm.names])
n.trajectory('arm_right_controller',n.arm.names,q,.3)
n.wait(.5)
print('Actual joints',q,flush=True)
print('Actual tool',n.matrix('base_footprint','gripper_right_grasping_link'),flush=True)
print('Contacts',contacts,flush=True)
base_to_torso = n.matrix('torso_lift_link','base_footprint')
for roll in (0.,np.pi):
    goal = base_to_torso @ transform([.51,-.24,1.582],[roll,0,0])
    try:
        sol = n.arm.solve(goal,q)
        print('Roll',roll,'solution',sol,'change',sol-q,flush=True)
    except RuntimeError as e:
        print(e,flush=True)
n.destroy_node()
rclpy.shutdown()
