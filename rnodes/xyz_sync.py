#!/usr/bin/env python2
# coding:utf-8

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import ros_numpy
#from ai_milking_ros.msg   import BoundingBox, BoundingBoxes
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg        import PointCloud2
from darknet_ros_msgs.msg import BoundingBox

bounding_box = np.array((0,0,0,0))
arr2 = np.zeros((640,480))
pc = PointCloud2()

