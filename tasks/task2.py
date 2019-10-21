#!/bin/python3
import orspy
import math
import utils
import sys

pose = orspy.EulerPose(68.35, 98.35, 0, 0, 0, math.pi)

simulated = False # change this to switch between simulated and real robot
steps = 4

sched = utils.Scheduler()
sched.use_simulated(simulated)

import time

time.sleep(3) # wait for all callbacks
import wws_lite

if not wws_lite.test_connection():
    print("could not connect to WWS")

current_x = None
current_y = None
current_angle = None

def wws_pose_cb(m):
    if simulated:
        current_x = m["payload"]["pose"]["pose"]["position"]["x"]
        current_y = m["payload"]["pose"]["pose"]["position"]["y"]
    else:
        current_x = m["payload"]["pose"]["position"]["x"]
        current_y = m["payload"]["pose"]["position"]["y"]
    current_angle = m["payload"]["pose"]["orientation"]["z"]
    print("x: {}, y: {}, angle: {}".format(current_x, current_y, current_angle))
    wws_lite.unsubscribe(topic)

topic = "{}/pose".format(sched.wws_topic_prefix())

sched.goto(pose, maxspeed=.2, allowed_trans_error=0.4, allowed_rot_error=0.5)# , allowed_trans_error=0.3, allowed_rot_error=0.5)

time.sleep(1)

print("start rotate")

for _ in range(steps * 2 - 2):
    sched.rotate(speed=math.pi/8, maxrot=math.pi/steps)# - math.pi / (steps * 16))

sched.goto(pose, maxspeed=.2, allowed_trans_error=0.4, allowed_rot_error=0.5)# , allowed_trans_error=0.3, allowed_rot_error=0.5)

time.sleep(1)

wws_lite.subscribe(topic, wws_pose_cb)

sys.exit()
