#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <sstream>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "point_cloud_listener_c");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<PointCloud2>("/camera/depth_registered/points", 1, callback);

    pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;

    pcl::fromROSMsg(cloud_t,PointCloudXYZ);
    std::cout << "width is " << PointCloudXYZ.width << std::endl;
    std::cout << "height is " << PointCloudXYZ.height << std::endl;
    int cloudsize = (PointCloudXYZ.width) * (PointCloudXYZ.height);
    for (int i=0; i< cloudsize; i++){
    std::cout << "(x,y,z) = " << PointCloudXYZ.points[i] << std::endl;

    ros::spin();
    return 0;
}



