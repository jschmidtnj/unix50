#!/bin/python3
import orspy
import math

sched = orspy.Scheduler()
if sched.connect():
    print("you are connected to the robotics runtime\n")
else:
    print("you failed to connect to the robotics runtime\n")

import time
from orspy import mtype # message types

def robot_added_cb(m): 
    print("robot added: {}".format(m.info.rid)) # rid is "robot ID"

import threading
action_done_event = threading.Event()

def action_update_cb(m):
    print("action update: {}".format(m))
    if m.isDone:
        print("action done")
        action_done_event.set()
        
sched.set_cb(mtype.ACTIONUPDATE, action_update_cb)

sched.set_cb(mtype.ROBOTADDED, robot_added_cb)
time.sleep(3) # wait for all callbacks

simulated = False # change this to switch between simulated and real robot
team_color = 'yellow'
team_member = 1
simulated_rid = "{}sim{}".format(team_color, team_member)
real_rid = "{}robot{}".format(team_color, team_member)

def rid():
    if simulated:
        return simulated_rid
    return real_rid

# maxspeed==None means "as fast as you can go"
def goto(pose, maxspeed=None, allowed_trans_error=None, allowed_rot_error=None):
    action_done_event.clear()
    action_num = sched.start_action(rid(), 'goto_pose', pose=pose, maxspeed=maxspeed, allowed_trans_error=allowed_trans_error, allowed_rot_error=allowed_rot_error)
    print("started action {}".format(action_num))
    action_done_event.wait()
    print("done going to pose")
    return action_num

#this is good #pose = orspy.EulerPose(68, 98.2, 0, 0, 0, math.pi) # hmmm, I wonder where this is...?
pose = orspy.EulerPose(68.35, 98.35, 0, 0, 0, 0*math.pi) # hmmm, I wonder where this is...?
goto(pose, maxspeed=.2, allowed_trans_error=0.15)# , allowed_trans_error=0.3, allowed_rot_error=0.5)

# sched.stop_action(rid(), 0)
