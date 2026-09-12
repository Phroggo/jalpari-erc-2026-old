"""Diagnostic continuation of a held book; not a scored complete trial."""
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
    raise RuntimeError(f'Expected one retained book, got {held}')
n.book_id = next(iter(held))
print('DIAGNOSTIC DELIVERY ONLY:',n.book_id,flush=True)
try:
    n.deliver(np.eye(4))
finally:
    n.velocity()
    n.destroy_node()
    rclpy.shutdown()
