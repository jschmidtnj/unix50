import utils
sched = utils.Scheduler()
import wws_lite
from IPython.display import Image
import pprint
pp = pprint.PrettyPrinter(indent=4)
import math
from time import sleep
import matplotlib.pyplot as plt
import urllib.request
import numpy as np
import orspy

 
class State:
    def __init__(self, sched):
        self.object_seen = False
        self.sched = sched
        self.object = None
 
        # self.sched.add_robot('codebox1', utils.bottom_I_close2wall)
        self.pose_detect = False
        self.img_detect = False
        self.action = None
        self.waypoints = [
            {
                "coord": orspy.EulerPose(68.73, 105.66, 0, 0, 0, 0),
                "rotate": False
            },
            {
                "coord": orspy.EulerPose(67, 104.3, 0, 0, 0, 0),
                "rotate": False
            },
            {
                "coord": orspy.EulerPose(69.3, 104.3, 0, 0, 0, 0),
                "rotate": False
            },
            {
                "coord": orspy.EulerPose(69.3, 107.5, 0, 0, 0, 0),
                "rotate": False
            },
            {
                "coord": orspy.EulerPose(67, 107.5, 0, 0, 0, 0),
                "rotate": False
            }
        ] # list of positions

        self.dectections = []

    def navigate_to(self, locaction):
        return self.sched.goto(locaction,
                allowed_trans_error=0.3, allowed_rot_error=0.5)

    def rotate(self):
        steps = 4
        for _ in range(steps * 2):
            sched.rotate(speed=math.pi/8, maxrot=math.pi/steps)# - math.pi / (steps * 16))



    def start(self):
        self.action = navigate_to(0)
        self.topic_img = "{}/object_img".format(self.sched.wws_topic_prefix())
        self.topic_pose = "{}/object_pose".format(self.sched.wws_topic_prefix())
        wws_lite.subscribe(self.topic_img, lambda x: self.wws_detect_img_cb(x))
        wws_lite.subscribe(self.topic_pose, lambda x: self.detect_cb_pose(x))

    def clear_action(self):
        self.sched.stop_action(self.action)

    def end(self):
        self.clear_action()
        wws_lite.unsubscribe(self.topic_img)
        wws_lite.unsubscribe(self.topic_pose)
       
    def detect_cb_pose(self, m):
        sleep(0.3)
        if not self.object_seen:
            # print('object seen, closing in...')
            self.sched.stop_action(self.action)
            self.action = self.sched.fwd(.5, maxdist=0.5)
            self.object_seen = True
            self.pose = m
            self.class_name = m['object']['payload']['objects']['detections'][0]['class_name']
            # print(f"object type- {m['object']['payload']['objects']['detections'][0]['class_name']}")
            self.pose_detect = True
 
    def wws_detect_img_cb(self, m):
        if self.object_seen:
            self.img = m
            detect = m['image']['url']
            f = urllib.request.urlopen(detect)
            a = plt.imread(f, 0)
            self.object_img = a
            self.img_detect = True

    def localize_all_objects(self):
        pass
       
    def control_loop(self):
        self.start()
        location_index = 0
        for way in self.waypoints:
            loc = way["coord"]
            do_rotate = way["rotate"]
            sleep(0.1)

            navigate_to(loc)
            if do_rotate:
                self.rotate()
                

            #print('sleeping...')
            #if self.img_detect and self.pose_detect:
            #      break
        self.end()
        self.localize_all_objects()
#       print(self.pose)
        #s = self.pose['object']['payload']['objects']['detections'][0]['bbox']
        #print(f"object position- {np.mean(s[0::2]), np.mean(s[1::2])}")
        # print(self.img)
        #print(f'class- {self.class_name}')
        #plt.imshow(self.object_img)
        #plt.show()
                 
                 
# state = State()
sched.goto(orspy.EulerPose(68.73, 105.66, 0, 0, 0, 0),
                                     allowed_trans_error=0.3, allowed_rot_error=0.5)
# state.control_loop()
# for i in range(100):
#     state.sched.stop_action(i)
# state.delete_state()