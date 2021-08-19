#!/usr/bin/env python2
# coding:utf-8

#Depth camera must be started using: roslaunch realsense2_camera rs_camera.launch align_depth:=true, rosbag böyle alınmamış

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg        import PointCloud2
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, Coordinate, ObjectCount
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CompressedImage
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo as cameraInfo
import math
import tf

class xyz_publisher:
    def __init__(self):

        rospy.init_node('depth_image_listener', anonymous=False)

        self.bounding_box = np.array((0,0,0,0))
        #variables for camera parameters
        self.camera_info = np.array((0,0,0,0))
        self.ppx = 0.0
        self.ppy = 0.0
        self.fx = 0.0
        self.fy = 0.0
        #found is used to check whether probe exists
        self.found = False
        self.coordinates = Coordinate()
        self.arr2 = np.zeros((640,480))
        #used for finding the two diagonals of the bounding box
        self.positive_diagonal = np.array((0,0,0))
        self.negative_diagonal = np.array((0,0,0))
        self.point = np.array((0.0,0.0,0.0))

        self.center_x_pixel = 0.0
        self.center_y_pixel = 0.0
        self.xmin = 0
        self.xmax = 0
        self.ymin = 0
        self.ymax = 0

        self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", msg_Image, self.point_callback)
        self.camera_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", cameraInfo, self.camera_callback)
        self.bounding_box_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
        self.object_count_sub = rospy.Subscriber("darknet_ros/found_object", ObjectCount, self.object_callback)
        self.pub = rospy.Publisher('/probe/xyz_coordinates', Coordinate, queue_size=1)
            
        self.rate = rospy.Rate(10) #10Hz

        #count = 0
        #find_count = 0
        while not rospy.is_shutdown():
            #count+=1

            '''rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", msg_Image, self.point_callback)
            rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", cameraInfo, self.camera_callback)
            rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
            rospy.Subscriber("darknet_ros/found_object", ObjectCount, self.object_callback)'''
                
            if(self.found == True):
                #find_count += 1
                print "probe detected"
                self.tf_broadcaster(self.coordinates)
                self.pub.publish(self.coordinates)
            else:
                print "no probe"
            
            #print (float(find_count) / float(count))*100
            #print count
            self.rate.sleep()  


    def find_center(self):

        center_x_pixel = self.bounding_box[2] - (self.bounding_box[2] - self.bounding_box[0])/2
        center_y_pixel = self.bounding_box[3] - (self.bounding_box[3] - self.bounding_box[1])/2


    def calculate_diagonal(self, x1,y1,x2,y2):

        #find the slope of a linear function
        if( (x1-x2) != 0):
            m = (y1-y2)/(x1-x2)
        else:
            m = 0
        b = y1 - m*x1
        return m, b

    def calculate_diagonal_average(self, a,b,c,d): 
        #sum all the points on the diagonal and take the average depth

        m1, b1 = self.calculate_diagonal(a,b,c,d)
        counter = 0
        pointt =  np.array((0.0,0.0,0.0))
        for i in range(self.xmin, self.xmax):
            y_value = i*m1 + b1
            if(y_value == 480):
                y_value -= 1
            pointt[0] += i
            pointt[1] += y_value
            pointt[2] += self.arr2[y_value][i]
            counter+=1

        if(counter != 0):
            pointt = np.divide(pointt, counter)

        return pointt


    def bounding_boxes_callback(self, data):

        self.xmin = data.bounding_boxes[0].xmin 
        self.ymin = data.bounding_boxes[0].ymin
        self.xmax = data.bounding_boxes[0].xmax 
        self.ymax = data.bounding_boxes[0].ymax

    def point_callback(self, data):

        #function to take pixel coordinates and depth of the probe and convert it to a real world xyz point

        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.arr2 = np.array(depth_image, dtype=np.float32)

        self.negative_diagonal = self.calculate_diagonal_average(self.xmin, self.ymin, self.xmax, self.ymax)
        self.positive_diagonal = self.calculate_diagonal_average(self.xmin, self.ymax, self.xmax, self.ymin)

        #Take the diagonal which has the smaller average depth value, the closer one.
        if(self.negative_diagonal[2] < self.positive_diagonal[2]):
            self.point = self.negative_diagonal
        else:
            self.point = self.positive_diagonal
        
        #Use camera intrinsincs to convert pixel coordinates to real world xyz
        self.point[0] = (self.point[0] - self.camera_info[0])*self.point[2] / self.camera_info[2]
        self.point[1] = (self.point[1] - self.camera_info[1])*self.point[2] / self.camera_info[3]
        self.point[2] = self.point[2]

        self.coordinates.x = self.point[2] #distance
        self.coordinates.y = self.point[0] #horizontal
        self.coordinates.z = self.point[1]*(-1) #vertical

        rospy.sleep(0.001)


    def camera_callback(self, cameraData):

        #Set intrinsinc values of the camera

        self.camera_info[0] = cameraData.K[2] #ppx
        self.camera_info[1] = cameraData.K[5] #ppy
        self.camera_info[2] = cameraData.K[0] #fx
        self.camera_info[3] = cameraData.K[4] #fy

    def object_callback(self, data):

        #Set whether probe exists

        if(data.count > 0):
            self.found = True
        else:
            self.found = False

    def tf_broadcaster(self, coordinates):
        br = tf.TransformBroadcaster()
        br.sendTransform(( coordinates.x/1000, coordinates.y/1000, coordinates.z/1000), (0.0,0.0,0.0,1.0), rospy.Time(), 'probe', '/camera_link')


if __name__ == '__main__':

    try:
        xyz_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
    
