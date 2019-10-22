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
import pickle
from contest import Run


class State:
    def __init__(self, sched, simulated):
        self.sched = sched
        # self.sched.add_robot('codebox1', utils.bottom_I_close2wall)
        self.waypoints = [
            {"coord": orspy.EulerPose(64.44, 96.632, 0, 0, 0, 0)},
            {"coord": orspy.EulerPose(66.44, 96.632, 0, 0, 0, 0)},
            {"coord": orspy.EulerPose(64.44, 96.632, 0, 0, 0, 0)},
        ] # list of positions
        self.simulated = simulated
        sched.use_simulated(simulated)
        self.poses = []
        self.images = []
        self.objects = []
        self.last_timestamp = 0
 
    def navigate_to(self, location):
        return self.sched.goto(location,
                               allowed_trans_error=0.5, allowed_rot_error=6.)
 
    def rotate(self):
        steps = 4
        for _ in range(steps * 2):
            sched.rotate(speed=math.pi/8, maxrot=math.pi/steps)# - math.pi / (steps * 16))
 
    def start(self):
        self.topic_img = "{}/object_img".format(self.sched.wws_topic_prefix())
        self.topic_pose = "{}/object_pose".format(self.sched.wws_topic_prefix())
        wws_lite.subscribe(self.topic_img, lambda x: self.wws_detect_img_cb(x))
        wws_lite.subscribe(self.topic_pose, lambda x: self.detect_cb_pose(x))
 
    def clear_action(self):
        self.sched.stop_action(self.action)
 
    def end(self):
        wws_lite.unsubscribe(self.topic_img)
        wws_lite.unsubscribe(self.topic_pose)

    def get_size_data(self, m):
        bbox = None
        if self.simulated:
            bbox = m['object']['payload']['objects']['detections'][0]['bbox']
        else:
            bbox = m['object']['payload']['objects']['detections'][0]['bbox']
        # y_top, x_left, y_bot, x_right = bbox
        return bbox
       
    def detect_cb_pose(self, m):
        delta_same_object = 5
        if self.simulated:
            if len(m['object']['payload']['objects']['detections']) > 0:
                # print(m['object']['payload']['objects']['detections'])
                class_name = m['object']['payload']['objects']['detections'][0]['class_name']
                self.poses.append(m)
                # robot pose
                x = m['pose']['payload']['pose']['pose']['position']['x']
                y = m['pose']['payload']['pose']['pose']['position']['y']
                bbox = self.get_size_data(m)
                size = abs(bbox[1] - bbox[3])
                if m['timestamp'] - self.last_timestamp > delta_same_object:
                    self.objects.append({
                        "name": class_name,
                        "x": x,
                        "y": y,
                        "size": size,
                        "last_image": len(self.images) + 1
                    }) # new object
                else:
                    if size > self.objects[-1]["size"]:
                        self.objects[-1] = {
                            "name": class_name,
                            "x": x,
                            "y": y,
                            "size": size,
                            "last_image": len(self.images)
                        } # overwrite current object data
                self.last_timestamp = m['timestamp']
        else:
            if len(m['object']['payload']['detections']) > 0:
                class_name = m['object']['payload']['detections'][0]['class']['name']
                self.poses.append(m)
                # robot pose
                x = m['pose']['payload']['pose']['position']['x']
                y = m['pose']['payload']['pose']['position']['y']
                bbox = self.get_size_data(m)
                size = abs(bbox[1] - bbox[3])
                if m['timestamp'] - self.last_timestamp > delta_same_object:
                    self.objects.append({
                        "name": class_name,
                        "x": x,
                        "y": y,
                        "size": size,
                        "last_image": len(self.images) + 1
                    }) # new object
                else:
                    if size > self.objects[-1]["size"]:
                        self.objects[-1] = {
                            "name": class_name,
                            "x": x,
                            "y": y,
                            "size": size,
                            "last_image": len(self.images)
                        } # overwrite current object data
                self.last_timestamp = m['timestamp']
        # print(f"object type- {m['object']['payload']['objects']['detections'][0]['class_name']}")
 
    def wws_detect_img_cb(self, m):
        
        # print(len(self.images), len(self.poses))
        if len(self.images) < len(self.poses):
            detect = m['image']['url']
            f = urllib.request.urlopen(detect)
            a = plt.imread(f, 0)
            # print('adding image')
            while len(self.images) < len(self.poses):
                self.images.append(a)

    def control_loop(self):
        self.start()
        count = 0
        for _ in range(1):
            for way in self.waypoints:
                loc = way["coord"]
                self.navigate_to(loc)
                print(f'saving {len(self.poses)} poses and {len(self.images)} images') #to data_{count}.pickle' )
                pickle.dump((self.poses, self.images, self.objects), open(f'/home/ros/spinachbots/jupyter/data{count}.pickle', 'wb'))
                count += 1
        
        print(self.objects)
        run = Run(name="first run", dry_run=self.simulated) # name is only used for display/debugging purposes
        run.start() # start the run
        for obj in self.objects:
            run.localize(obj['name'], [obj['x'], obj['y']]) # submit localization attempt for drone at given [x,y] coordinates
        run.stop() # stop the run
        #run.status # returns one of 'init', 'started', 'stopped'
        #run.events # returns a list of all recorded localization attempt events for this run
        
        self.end()

sched = utils.Scheduler()
# sched.add_robot('codebox1', utils.bottom_I_close2wall)
state = State(sched, simulated=True)
 
# sched.goto(orspy.EulerPose(68.73, 105.66, 0, 0, 0, 0),
#                                      allowed_trans_error=0.3, allowed_rot_error=0.5)
state.control_loop()
state.end()
#print(state.poses)
#print(state.images)
# for i in range(100):
#     state.sched.stop_action(i)
# state.delete_state()




