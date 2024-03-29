import utils
sched = utils.Scheduler()
import wws_lite
from IPython.display import Image
import pprint
pp = pprint.PrettyPrinter(indent=4)
import math
import time
import matplotlib.pyplot as plt
import urllib.request
import numpy as np
import orspy
import pickle
from contest import Run


def pose_from_info(x, y, angle=0):
    return orspy.EulerPose(x, y, 0, 0, 0, angle)


class State:
    def __init__(self, sched, simulated, run_number, try_legit, auto_submit_minutes):
        self.sched = sched
        # self.sched.add_robot('codebox1', utils.bottom_I_close2wall)
        self.waypoints = [
            pose_from_info(66.5, 96.5),
            pose_from_info(66.5, 95.5, 0.01),
            pose_from_info(66.5, 98.5, 0.01),
            pose_from_info(69.1, 98.5, 3*math.pi/2),
            pose_from_info(68.5, 101.5, math.pi),
            pose_from_info(66, 105.5),
            pose_from_info(68.5, 105.5),
            pose_from_info(71, 112, math.pi),
            pose_from_info(68, 113.5, 3.*math.pi/2.),
            pose_from_info(66, 113.8, 3. * math.pi / 2.),
            pose_from_info(66, 111.8, 0.001),
            pose_from_info(67, 108),
            pose_from_info(65.2, 108),
            pose_from_info(65.2, 105.5),
            pose_from_info(66.3, 105.5),
            pose_from_info(65, 102, 0.01),
            pose_from_info(65, 97)
        ]
        self.object_data = {
            "tablet": {
                "num_object": 1,
                "num_submit": 0
            },
            "monitor": {
                "num_object": 1,
                "num_submit": 0,
                "location": {
                    "x": 69.801,
                    "y": 95.691
                }
            },
            "robot-arm": {
                "num_object": 1,
                "num_submit": 0,
                "location": {
                    "x": 65.649,
                    "y": 110.384
                }
            },
            "drone": {
                "num_object": 2,
                "num_submit": 0
            },
            "cube-dock": {
                "num_object": 1,
                "num_submit": 0
            },
            "emergency-stop": {
                "num_object": 1,
                "num_submit": 0,
                "location": {
                    "x": 67.863,
                    "y": 94.853
                }
            },
            "hammer": {
                "num_object": 1,
                "num_submit": 0
            },
            "radio": {
                "num_object": 1,
                "num_submit": 0
            },
            "screwdriver": {
                "num_object": 1,
                "num_submit": 0
            },
            "phone": {
                "num_object": 1,
                "num_submit": 0
            },
            "saw": {
                "num_object": 1,
                "num_submit": 0
            },
            "beacon": {
                "num_object": 3,
                "num_submit": 0
            },
            "shipping-box": {
                "num_object": 2,
                "num_submit": 0
            }
        }
        #] # list of positions
        self.simulated = simulated
        sched.use_simulated(simulated)
        self.poses = []
        self.images = []
        self.objects = []
        self.last_timestamp = 0
        self.run_number = 1
        self.try_legit = try_legit
        self.start_time = time.time()
        self.auto_submit_time = self.start_time + 60 * auto_submit_minutes
        self.rotation_steps = 4
        self.run = Run(name="trial {}".format(self.run_number), dry_run=self.simulated)  # name is only used for display/debugging purposes
        self.run.start()  # start the run

    def navigate_to(self, location):
        return self.sched.goto(location,
                               allowed_trans_error=0.5, allowed_rot_error=6.if location.c == 0 else 0.6,
                               maxspeed=0.3)

    def rotate(self):
        for _ in range(self.rotation_steps * 2):
            sched.rotate(speed=math.pi/8, maxrot=math.pi/self.rotation_steps)# - math.pi / (self.rotation_steps * 16))

    def start(self):
        self.topic_img = "{}/object_img".format(self.sched.wws_topic_prefix())
        self.topic_pose = "{}/object_pose".format(self.sched.wws_topic_prefix())
        wws_lite.subscribe(self.topic_img, lambda x: self.wws_detect_img_cb(x))
        wws_lite.subscribe(self.topic_pose, lambda x: self.detect_cb_pose(x))
        #self.sched.stop_action(-1)

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
            bbox = m['object']['payload']['detections'][0]['bbox']
        # y_top, x_left, y_bot, x_right = bbox
        return bbox

    def detect_cb_pose(self, m):
        delta_same_object = 5
        fudge_factor = 5
        # print(m)
        if self.simulated:
            if len(m['object']['payload']['objects']['detections']) > 0:

                # print(m['object']['payload']['objects']['detections'])
                class_name = m['object']['payload']['objects']['detections'][0]['class_name']
                self.poses.append(m)
                # robot pose
                print(m)
                x = m['pose']['payload']['pose']['pose']['position']['x']
                y = m['pose']['payload']['pose']['pose']['position']['y']
                angle = m["payload"]["pose"]["orientation"]["z"]
                bbox = self.get_size_data(m)
                size = abs(bbox[1] - bbox[3])
                if m['timestamp'] - self.last_timestamp > delta_same_object:
                    self.objects.append({
                        "name": class_name,
                        "x": x,
                        "y": y,
                        "size": size,
                        "first_index": len(self.images),
                        "last_index": len(self.images),
                        "count": 1
                    })  # new object
                else:
                    if size > self.objects[-1]["size"]:
                        self.objects[-1] = {
                            "name": class_name,
                            "x": x,
                            "y": y,
                            "size": size,
                            "first_index": self.objects[-1]["first_index"],
                            "last_index": len(self.images),
                            "count": self.objects[-1]["count"] + 1
                        }  # overwrite current object data
                self.last_timestamp = m['timestamp']
        else:
            if len(m['object']['payload']['detections']) > 0:
                class_name = m['object']['payload']['detections'][0]['class']['name']
                self.poses.append(m)
                # robot pose
                x = m['pose']['payload']['pose']['position']['x']
                y = m['pose']['payload']['pose']['position']['y']
                angle = m["pose"]["payload"]["pose"]["orientation"]["z"]
                bbox = self.get_size_data(m)
                size = abs(bbox[1] - bbox[3])
                if m['timestamp'] - self.last_timestamp > delta_same_object:
                    self.objects.append({
                        "name": class_name,
                        "x": x,# + math.cos(angle) * size * fudge_factor,
                        "y": y,# + math.sin(angle) * size * fudge_factor,
                        "size": size,
                        "first_index": len(self.images),
                        "last_index": len(self.images),
                        "count": 1
                    })  # new object
                else:
                    if size > self.objects[-1]["size"]:
                        self.objects[-1] = {
                            "name": class_name,
                            "x": x,# + math.cos(angle) * size * fudge_factor,
                            "y": y,# + math.sin(angle) * size * fudge_factor,
                            "size": size,
                            "first_index": self.objects[-1]["first_index"],
                            "last_index": len(self.images),
                            "count": self.objects[-1]["count"] + 1
                        }  # overwrite current object data
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
        short_circuit = False
        for way in self.waypoints:
            if time.time() > self.auto_submit_time:
                short_circuit = True
                print("short circuit")
                self.submit()
                
            loc = way
            self.navigate_to(loc)
            print(f'saving {len(self.poses)} poses and {len(self.images)} images')  # to data_{count}.pickle' )
            pickle.dump((self.poses, self.images, self.objects),
                        open(f'/home/ros/spinachbots/jupyter/data{count}.pickle', 'wb'))
            count += 1
            
        if not short_circuit:
            self.submit()

    def submit(self):
        print(self.objects)
        min_count = 1
        potential_shipping = ["Priority Mail - USPS", "FedEx"]
        if self.try_legit:
            for obj in self.objects:
                if obj["count"] > min_count:
                    if obj["name"] in self.object_data:
                        if self.object_data[obj["name"]]["num_submit"] < self.object_data[obj["name"]]["num_object"]:
                            if "location" in self.object_data[obj["name"]]:
                                self.run.localize(obj['name'],
                                        [self.object_data[obj["name"]]["location"]['x'], self.object_data[obj["name"]]["location"]['y']])
                            else:
                                self.run.localize(obj['name'],
                                        [obj['x'], obj['y']])
                            self.object_data[obj["name"]]["num_submit"] += 1
                    else:
                        self.run.localize(obj['name'],
                                [obj['x'], obj['y']])  # submit localization attempt for drone at given [x,y] coordinates
                if obj["name"] == "shipping-box":
                    print("shipping box company at x: {}, y: {}: {}".format(obj['x'], obj['y'], potential_shipping[0]))
        for name in self.object_data:
            if "location" in self.object_data[name] and self.object_data[name]["num_submit"] < self.object_data[name]["num_object"]:
                self.run.localize(name,
                                [self.object_data[name]["location"]['x'], self.object_data[name]["location"]['y']])
                self.object_data[name]["num_submit"] += 1
        self.run.stop()  # stop the run
        # self.run.status # returns one of 'init', 'started', 'stopped'
        # self.run.events # returns a list of all recorded localization attempt events for this run
        self.end()

sched = utils.Scheduler()
# sched.add_robot('codebox1', utils.bottom_I_close2wall)
simulated = False
run_number = 1
try_legit = True
auto_submit_minutes = 9.5
state = State(sched, simulated, run_number, try_legit, auto_submit_minutes)

# sched.goto(orspy.EulerPose(68.73, 105.66, 0, 0, 0, 0),
#                                      allowed_trans_error=0.3, allowed_rot_error=0.5)
state.control_loop()
#state.end()
# for i in range(100):
#     state.sched.stop_action(i)
# state.delete_state()
