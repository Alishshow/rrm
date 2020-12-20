#include "abb_scanner/LaserScanToPointCloud.h"

LaserScanToPointCloud::LaserScanToPointCloud() :
    laser_sub_(n_, "rrbot/laser/scan", 200),
    laser_notifier_(laser_sub_,listener_, "/tool0", 200) {
    ROS_INFO("Initializing scan listener");
//init point cloud
    full_cloud.header.stamp = ros::Time::now();
    full_cloud.header.frame_id = "base_link";
// init laser scan transformation
    laser_notifier_.registerCallback(
            boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(TRANSFORM_TIME_TOLERANCE));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("final_cloud", 1);
}
void LaserScanToPointCloud::scanCallback (const sensor_msgs::LaserScan::ConstPtr& in_scan )
{
// project laser scan to 3D pointcloud

    try
    {
        projector_.transformLaserScanToPointCloud("base_link",*in_scan,temp_cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }// Append new scan to cloud
    pcl::concatenatePointCloud( full_cloud, temp_cloud, full_cloud  );
    //publish scanned object in scan_object frame
    scan_pub_.publish(full_cloud);
}
