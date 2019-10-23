#!/bin/python3
import orspy
import math
import utils
import wws_lite
from time import sleep
from IPython.display import Image
import matplotlib.pyplot as plt
import urllib.request
import pprint

global success_count
global save_image
save_image = False
success_count = 0
pp = pprint.PrettyPrinter(indent=4)
simulated = False # change this to switch between simulated and real robot

sched = utils.Scheduler()
sched.use_simulated(simulated)

pose = orspy.EulerPose(66.3, 96.6, 0, 0, 0, -math.pi / 3)

sleep(3) # wait for all callbacks

sched.goto(pose, maxspeed=.2, allowed_trans_error=0.4, allowed_rot_error=0.5)# , allowed_trans_error=0.3, allowed_rot_error=0.5)
action = sched.rotate(math.pi/32)

if not wws_lite.test_connection():
    print("could not connect to WWS")

def detect_cb_pose(a, m):
    global detect
    global success_count
    # pp.pprint("detect_cb-- {}".format(m))
    # print(a)
    sleep(0.3)
    if len(m['object']['payload']['detections']) > 0 and m['object']['payload']['detections'][0]['class']['name'] == "beacon":
        print('object seen, closing in...')
        sched.stop_action(a)
        sleep(1)
        action = sched.fwd(.2, maxdist=0.2)
        save_image = True
        success_count += 1
        if success_count < 2:
            sleep(3)
            pose = orspy.EulerPose(66.3, 97.2, 0, 0, 0, -math.pi / 3)
            sched.goto(pose, maxspeed=.2, allowed_trans_error=0.4, allowed_rot_error=0.5)# , allowed_trans_error=0.3, allowed_rot_error=0.5)
            action = sched.rotate(math.pi/32)
        # a = sched.rotate(math.pi/4)
        # action = a
        # pp.pprint("detect_cb-- {}".format(m))
        print("object type- {}".format(m['object']['payload']['detections'][0]['class']['name']))
        if success_count >= 2:
            wws_lite.unsubscribe(topic_pose)
            wws_lite.unsubscribe(topic_img)


def wws_detect_img_cb(m):
    global save_image
    global success_count

    detect = m['image']['url']
    if save_image:
        f = urllib.request.urlopen(detect)
        a = plt.imread(f, 0)
        plt.imshow(a)
        plt.savefig("output{}.png".format(success_count))

topic_img = "{}/object_img".format(sched.wws_topic_prefix())
topic_pose = "{}/object_pose".format(sched.wws_topic_prefix())
wws_lite.subscribe(topic_img, wws_detect_img_cb)
wws_lite.subscribe(topic_pose, lambda x: detect_cb_pose(action, x))

