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


def pose_from_info(x, y, angle=0):
    return orspy.EulerPose(x, y, 0, 0, 0, angle)


class State:
    def __init__(self, sched, simulated, run_number):
        self.sched = sched
        # self.sched.add_robot('codebox1', utils.bottom_I_close2wall)
        self.waypoints = [
            pose_from_info(66.5, 96.5),
            pose_from_info(66.5, 95.5, 0.01),
            pose_from_info(66.5, 98.5, 0.01),
            pose_from_info(69.5, 98.5, 3*math.pi/2),
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
        ] # list of positions
        self.simulated = simulated
        sched.use_simulated(simulated)
        self.poses = []
        self.images = []
        self.objects = []
        self.last_timestamp = 0
        self.run_number = 1

    def navigate_to(self, location):
        return self.sched.goto(location,
                               allowed_trans_error=0.5, allowed_rot_error=6.if location.c == 0 else 0.6,
                               maxspeed=0.3)

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
                            "first_index": len(self.images),
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
                            "x": x,# + math.cos(angle) * size * fudge_factor,
                            "y": y,# + math.cos(angle) * size * fudge_factor,
                            "size": size,
                            "first_index": len(self.images),
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
        for _ in range(1):
            for way in self.waypoints:
                loc = way
                self.navigate_to(loc)
                print(f'saving {len(self.poses)} poses and {len(self.images)} images')  # to data_{count}.pickle' )
                pickle.dump((self.poses, self.images, self.objects),
                            open(f'/home/ros/spinachbots/jupyter/data{count}.pickle', 'wb'))
                count += 1

        print(self.objects)
        run = Run(name="RUN {}".format(self.run_number), dry_run=self.simulated)  # name is only used for display/debugging purposes
        run.start()  # start the run
        min_count = 1
        potential_shipping = ["Priority Mail - USPS", "FedEx"]
        object_data = {
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
        for obj in self.objects:
            if obj["count"] > min_count:
                if obj["name"] in object_data:
                    if object_data[obj["name"]]["num_submit"] < object_data[obj["name"]]["num_object"]:
                        if "location" in object_data[obj["name"]]:
                            run.localize(obj['name'],
                                    [object_data[obj["name"]]["location"]['x'], object_data[obj["name"]]["location"]['y']])
                        else:
                            run.localize(obj['name'],
                                    [obj['x'], obj['y']])
                        object_data[obj["name"]]["num_submit"] += 1
                else:
                    run.localize(obj['name'],
                            [obj['x'], obj['y']])  # submit localization attempt for drone at given [x,y] coordinates
            if obj["name"] == "shipping-box":
                print("shipping box company at x: {}, y: {}: {}".format(obj['x'], obj['y'], potential_shipping[0]))
        for name in object_data:
            if "location" in object_data[name] and object_data[name]["num_submit"] < object_data[name]["num_object"]:
                run.localize(obj['name'],
                                [object_data[name]["location"]['x'], object_data[name]["location"]['y']])
                object_data[name]["num_submit"] += 1
        run.stop()  # stop the run
        # run.status # returns one of 'init', 'started', 'stopped'
        # run.events # returns a list of all recorded localization attempt events for this run

        self.end()

sched = utils.Scheduler()
# sched.add_robot('codebox1', utils.bottom_I_close2wall)
simulated = True
run_number = 1
state = State(sched, simulated, run_number)

# sched.goto(orspy.EulerPose(68.73, 105.66, 0, 0, 0, 0),
#                                      allowed_trans_error=0.3, allowed_rot_error=0.5)
state.control_loop()
state.end()
# for i in range(100):
#     state.sched.stop_action(i)
# state.delete_state()