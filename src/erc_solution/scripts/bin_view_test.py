"""Diagnostic: face the fixed bin area, inspect colour and depth; no release."""
import time
import numpy as np
import cv2
import rclpy
from erc_solution.robot import Robot
from erc_solution.vision import objects

rclpy.init()
n = Robot()
end = time.monotonic()+5
while time.monotonic()<end:
    n.tick()
base = n.matrix('arena','base_footprint')[:2,3]
delta = np.array([-1.,0.])-base
n.navigate(base,np.arctan2(delta[1],delta[0]))
n.move_joints('head_controller',['head_1_joint','head_2_joint'],[0.,-.65],2.)
n.wait(.5)
camera = n.camera()
if camera is None:
    raise RuntimeError('No synchronized camera view')
rgb,depth,k,pose = camera
cv2.imwrite('/opt/erc_ws/src/erc_solution/erc_images/bin_view.jpg',rgb)
hsv = cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
red = ((hsv[:,:,0]<10)|(hsv[:,:,0]>170))&(hsv[:,:,1]>90)&(hsv[:,:,2]>50)
ys,xs = np.nonzero(red&np.isfinite(depth))
z = depth[ys,xs]
points = np.c_[(xs-k[2])*z/k[0],(ys-k[3])*z/k[1],z]
world = points @ pose[:3,:3].T+pose[:3,3]
print('Base',n.localizer.pose,'red world bounds',np.percentile(world,[0,10,50,90,100],axis=0),flush=True)
for option in (None,pose):
    found = objects(rgb,depth,k,'red',True,option)
    print('height gated',option is not None,[(d.box,(pose @ np.r_[d.point,1])[:3]) for d in found],flush=True)
cv2.imwrite('/opt/erc_ws/src/erc_solution/erc_images/bin_red_mask.png',red.astype(np.uint8)*255)
n.velocity()
n.destroy_node()
rclpy.shutdown()
