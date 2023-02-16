#!/usr/bin/env python
import cv2
import os
import re
import rospy
from rospy.topics import Subscriber
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.point_cloud2 import PointCloud2
from std_msgs.msg import String

# 讀取parameter server 內的參數
loadPath = rospy.get_param("log_path")      
loadPath = os.path.join(loadPath, "ring_front_center")
i = 0
DATA_PATH = loadPath
# DATA_PATH = '/home/ee904/SDC_lecture/hw5_noetic_ws/src/tracking_test_v1.1/argoverse-tracking/test/0f0d7759-fa6e-3296-b528-6c862d061bdd/ring_front_center'
def natural_key(string_):
    return [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', string_)]
photo_names = sorted([f for f in os.listdir(DATA_PATH)], key = natural_key)

def callback(data):
    cam_pub = rospy.Publisher('camera_front_center', Image, queue_size=10)
    bridge = CvBridge()
    global i
    for j in range(3):
      img = cv2.imread(os.path.join(DATA_PATH,photo_names[i]))
      cam_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
      # rospy.loginfo('camera image published')
      i=i+1
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    
    rospy.Subscriber("scan", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    


