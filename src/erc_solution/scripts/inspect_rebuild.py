"""Read-only live sensor diagnostic; saves the actual camera frame for inspection."""
import time
import cv2
import numpy as np
import rclpy
from erc_solution.mission import Mission
from erc_solution.vision import objects

rclpy.init()
node = Mission()
deadline = time.monotonic()+15
while time.monotonic()<deadline:
    node.tick()
    camera = node.camera()
    if camera is not None:
        column = node.observe_marker()
        camera = node.camera()
        if camera is None:
            continue
        bgr,depth,k,pose = camera
        cv2.imwrite('/opt/erc_ws/src/erc_solution/erc_images/rebuild_live.jpg',bgr)
        print('camera',node.rgb.header.frame_id,k,'depth',np.nanmin(depth),np.nanmax(depth),flush=True)
        print('base',node.matrix('arena','base_footprint'),'clock',node.now(),flush=True)
        print('head',[(n,v) for n,v in node.joints.items() if 'head_' in n],flush=True)
        print('torso',node.matrix('base_footprint','torso_lift_link'),flush=True)
        print('tip',node.matrix('base_footprint','gripper_right_grasping_link'),flush=True)
        print('markers',node.markers.detect(bgr,1.),flush=True)
        print('column',None if column is None else column[:2],flush=True)
        for d in objects(bgr,depth,k,node.colour):
            print('book',d.box,(pose @ np.r_[d.point,1])[:3],flush=True)
        for d in objects(bgr,depth,k,'red',bin_mode=True,height_transform=pose):
            print('BIN',d.box,(pose @ np.r_[d.point,1])[:3],flush=True)
        print('Held',node.held(),flush=True)
        break
else:
    print('NO CAMERA', 'rgb',node.rgb is not None,'depth',node.depth is not None,'info',node.info is not None,'clock',node.now(),flush=True)
    if node.rgb is not None:
        print('rgb frame',node.rgb.header.frame_id,'stamp',node.stamp(node.rgb),flush=True)
    print(node.tf.all_frames_as_yaml(),flush=True)
node.destroy_node()
rclpy.shutdown()
