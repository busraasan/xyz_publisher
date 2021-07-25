#!/usr/bin/env python2
# coding:utf-8

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg        import PointCloud2
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, Coordinate

bounding_box = np.array((0,0,0,0))
arr2 = np.zeros((640,480))
pc = PointCloud2()
point =  [0.0,0.0,0.0]
coordinates = Coordinate()


def bounding_boxes_callback(data):
    bounding_box[0] = data.bounding_boxes[0].xmin 
    bounding_box[1] = data.bounding_boxes[0].ymin
    bounding_box[2] = data.bounding_boxes[0].xmax 
    bounding_box[3] = data.bounding_boxes[0].ymax

def point_callback(data):
    pc.header       = data.header
    pc.height       = data.height
    pc.width        = data.width
    pc.fields       = data.fields
    pc.is_bigendian = data.is_bigendian
    pc.point_step   = data.point_step
    pc.row_step     = data.row_step
    pc.data         = data.data

    arr = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    arr2 = ros_numpy.point_cloud2.get_xyz_points(arr)
    center_x_pixel = bounding_box[2] - (bounding_box[2] - bounding_box[0])/2
    center_y_pixel = bounding_box[3] - (bounding_box[3] - bounding_box[1])/2

    #to be continued...
    step1 = (bounding_box[2] - bounding_box[0])
    step2 = (bounding_box[3] - bounding_box[1])
    '''for i in range(xmin, xmax, step1):
        for j in range(ymin, ymax, step2):'''

    point = np.array([0,0,0])
    point = arr2[center_y_pixel*640 + center_x_pixel]
    print point
    coordinates.x = point[2] #distance
    coordinates.y = point[0] #horizontal
    coordinates.z = point[1] #vertical
    rospy.sleep(0.001)



def get_pointcloud():
    
    rospy.init_node('point_cloud_listener')
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_callback)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_boxes_callback)

    pub = rospy.Publisher('/basan/xyz_coordinates', Coordinate, queue_size=1)
    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
            pub.publish(coordinates)
            rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    get_pointcloud()
