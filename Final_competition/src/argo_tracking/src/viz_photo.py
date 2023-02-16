#!/usr/bin/env python

import cv2
import os
import rospy
import pandas as pd
import json
from collections import deque
from publish_utils import publish_location
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.point_cloud2 import PointCloud2
from processing_utils import *
from visualization_msgs.msg import Marker, MarkerArray

log_path = rospy.get_param("/viz_photo/log_path")
label_path = rospy.get_param("/viz_photo/label_path")
Image_PATH = os.path.join(log_path, "ring_front_center")
Pose_PATH = os.path.join(log_path, "poses/")

label_path = label_path + "/per_sweep_annotations_amodal/"

photo_names = sorted([f for f in os.listdir(Image_PATH)], key = natural_key)
pose_files = sorted([f for f in os.listdir(Pose_PATH)], key = natural_key)
tracking_files = sorted([f for f in os.listdir(label_path)], key = natural_key)

"""
For pose timestamp alignment to lidar.
"""
lidar_files = sorted([f for f in os.listdir(os.path.join(log_path,"lidar/"))], key = natural_key)
po = string_split(pose_files)
li = string_split(lidar_files)
index = []
for idx,data in enumerate((po)):
    if data in li:
        index+=[idx]

i = 0
k = 1

class Object():
    def __init__(self,center):
        self.locations = deque(maxlen=20)
        self.locations.appendleft(center)

    def update(self, displacement, yaw_change, center):
        for i in range(len(self.locations)):
            x0, y0 =self.locations[i]
            x1 = x0 * np.cos(yaw_change) + y0 * np.sin(yaw_change) - displacement
            y1 = -x0 * np.sin(yaw_change) + y0 * np.cos(yaw_change)
            self.locations[i] = np.array([x1,y1])
        if center is not None:
            self.locations.appendleft(center)
        
    def reset(self):
        self.locations = deque(maxlen=20)

def read_track_json(tracking_file):
    
    center_list = []
    track_id = []
    with open (tracking_file) as f:
        track_data = (json.load(f))
        for i, data in enumerate(track_data):
            center = np.zeros((2))
            track_id.append(data['track_label_uuid'])
            center[0] = data['center']['x']
            center[1] = data['center']['y']
            center_list.append(center)
            # center[2] = track_data[i]['center']['z']
    return track_id, center_list

def callback(data, args):
    global i
    global k
    global previous_pose
    global previous_rotation
    centers = {}
    id, object_center = read_track_json(label_path+tracking_files[k])
    
    for track_id, center in zip(id,object_center):
        centers[track_id] = center
  
    
    bridge = CvBridge()
    
    for j in range(3):
        img = cv2.imread(os.path.join(Image_PATH,photo_names[i]))
        cam_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
        i = i+1
    # for p in range(1):
    print(f'frame={k}')




    pose,rotation = readJson(Pose_PATH + pose_files[index[k]])
    if previous_pose is None:  #no early frames
        for track_id in centers:   # add all detect object into trackers
            tracker[track_id] = Object(centers[track_id])
            #print(type(tracker))
    else:
        displacement = distance(pose[[0,1,2]],previous_pose[[0,1,2]])
        yaw_change = float(rotation - previous_rotation)
        # update all track object
        for track_id in centers:
            if track_id in tracker:  #if object has been tracked update its past trajectory
                tracker[track_id].update(displacement, yaw_change, centers[track_id])
                # print("Yaw_change:{:.6f}".format(yaw_change))
            else:   #if object not been tracked before
                tracker[track_id] = Object(centers[track_id])
        for track_id in tracker:    #if object has been tracked past but not detect in the present frame
            if track_id not in centers:
                tracker[track_id].update(displacement, yaw_change, None)


        #ego_car.update(displacement,yaw_change)
    previous_pose = pose
    #print(previous_pose)
    previous_rotation = rotation
    #print(len(tracker))
    publish_location(loc_pub,tracker,centers)
    k = k+1
    

def distance(pose, prev_pose):
    return np.linalg.norm(pose - prev_pose)   

 
def readJson(pose_files):
    with open(pose_files) as f:
        pose_data = json.load(f)
        pose = np.array(pose_data['translation'])
        rotation = np.array(pose_data['rotation'])
        r,p,y = euler_from_quaternion(rotation[0],rotation[1],rotation[2],rotation[3]) 
        # returns Json object as a dictionary
        
    return pose, y




if __name__ == '__main__':
    previous_pose = None
    previous_rotation = None
    #ego_car = Object()
    tracker = {} # track_id : Object
    
    while not rospy.is_shutdown():
        centers = {} #track id refer center use dictionary    
        loc_pub = rospy.Publisher('Traj', MarkerArray, queue_size=10)
        cam_pub = rospy.Publisher('camera_front_center', Image, queue_size=10)
        
        rospy.init_node('pub_photo', anonymous=True)
        rospy.Subscriber("/scan", PointCloud2, callback,(previous_pose,previous_rotation))
        rospy.spin()
