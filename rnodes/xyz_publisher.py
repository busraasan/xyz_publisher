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
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes, Coordinate, ObjectCount
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CompressedImage
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo as cameraInfo
import math
import tf

bounding_box = np.array((0,0,0,0))
arr2 = np.zeros((640,480))
point =  np.array((0,0,0))
point2 =  np.array((0,0,0))
point3 =  np.array((0,0,0))
point4 =  np.array((0,0,0))
coordinates = Coordinate()
camera_info = np.array((0,0,0,0))
ppx = 0.0
ppy = 0.0
fx = 0.0
fy = 0.0
found = False

def calculate_diagonal(x1,y1,x2,y2):
    if( (x1-x2) != 0):
        m = (y1-y2)/(x1-x2)
    else:
        m = 0
    b = y1 - m*x1
    return m, b

def bounding_boxes_callback(data):
    
    bounding_box[0] = data.bounding_boxes[0].xmin 
    bounding_box[1] = data.bounding_boxes[0].ymin
    bounding_box[2] = data.bounding_boxes[0].xmax 
    bounding_box[3] = data.bounding_boxes[0].ymax

def point_callback(data):
    xmin = bounding_box[0]
    xmax = bounding_box[2]
    ymin = bounding_box[1]
    ymax = bounding_box[3]


    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    arr2 = np.array(depth_image, dtype=np.float32)
    '''im = Image.fromarray(depth_array)
    im = im.convert("L")'''
    '''pc.header       = data.header
    pc.height       = data.height
    pc.width        = data.width
    pc.fields       = data.fields
    pc.is_bigendian = data.is_bigendian
    pc.point_step   = data.point_step
    pc.row_step     = data.row_step
    pc.data         = data.data

    arr = ros_numpy.point_cloud2.pointcloud2_to_array(pc)
    arr2 = ros_numpy.point_cloud2.get_xyz_points(arr)'''
    center_x_pixel = bounding_box[2] - (bounding_box[2] - bounding_box[0])/2
    center_y_pixel = bounding_box[3] - (bounding_box[3] - bounding_box[1])/2

    m1, b1 = calculate_diagonal(xmin, ymin, xmax, ymax)
    counter = 0
    point2 =  np.array((0,0,0))
    for i in range(xmin, xmax):
        y_value = i*m1 + b1
        if(y_value == 480):
            y_value -= 1
        point2[0] += i
        point2[1] += y_value
        point2[2] += arr2[y_value][i]
        counter+=1

    if(counter != 0):
        point2 = np.divide(point2, counter)

    m2, b2 = calculate_diagonal(xmin, ymax, xmax, ymin)
    counter = 0
    point3 =  np.array((0,0,0))
    for i in range(xmin, xmax):
        y_value = i*m2 + b2
        if(y_value == 480):
            y_value -= 1
        point3[0] += i
        point3[1] += y_value
        point3[2] += arr2[y_value][i]
        counter+=1

    if(counter != 0):
        point3 = np.divide(point3, counter)

    if(point2[2] < point3[2]):
        point4 = point2
    else:
        point4 = point3

    #point4 = convert_depth_to_phys_coord_using_realsense(point4[0], point4[1], point4[2], cameraInfo)
    
    point[0] = (point4[0] - camera_info[0])*point4[2] / camera_info[2]
    point[1] = (point4[1] - camera_info[1])*point4[2] / camera_info[3]
    point[2] = point4[2]

    coordinates.x = point[2] #distance
    coordinates.y = point[0] #horizontal
    coordinates.z = point[1]*(-1) #vertical

    '''coordinates.x = point4[2] #distance
    coordinates.y = point4[0] #horizontal
    coordinates.z = point4[1] #vertical'''
    rospy.sleep(0.001)

def camera_callback(cameraData):
    camera_info[0] = cameraData.K[2] #ppx
    camera_info[1] = cameraData.K[5] #ppy
    camera_info[2] = cameraData.K[0] #fx
    camera_info[3] = cameraData.K[4] #fy

def tf_broadcaster(coordinates):
    br = tf.TransformBroadcaster()
    br.sendTransform(( coordinates.x/1000, coordinates.y/1000, coordinates.z/1000), (0.0,0.0,0.0,1.0), rospy.Time(), 'probe', '/camera_link')

def object_callback(data):
    global found
    if(data.count > 0):
        found = True
    else:
        found = False

def get_pointcloud():
    
    rospy.init_node('depth_image_listener')
    #rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", msg_Image, point_callback)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bounding_boxes_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", cameraInfo, camera_callback)
    rospy.Subscriber("darknet_ros/found_object", ObjectCount, object_callback)

    pub = rospy.Publisher('/basan/xyz_coordinates', Coordinate, queue_size=1)
    rate = rospy.Rate(10) #10Hz
    global found
    while not rospy.is_shutdown():
            
            if(found == True):
                print "probe detected"
                tf_broadcaster(coordinates)
                pub.publish(coordinates)
            else:
                print "no probe"
            rate.sleep()
    
    
    rospy.spin()

if __name__ == '__main__':
    get_pointcloud()
