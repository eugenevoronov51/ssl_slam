// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"

std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;
nav_msgs::Odometry t265_odom;

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudSurfBuf.push(laserCloudMsg);
}

void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudEdgeBuf.push(laserCloudMsg);
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    pointCloudBuf.push(laserCloudMsg);
}

void odomHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    std::lock_guard<std::mutex> lock(mutex_lock);
    t265_odom = *odomMsg;
}

void odom_estimation(){
    while(ros::ok()){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty() && !pointCloudBuf.empty()){

            //read data
            std::lock_guard<std::mutex> lock(mutex_lock);
            if(!pointCloudBuf.empty() && (pointCloudBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period || 
                                           pointCloudBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period)){
                ROS_WARN("time stamp unaligned error and odom discarded, pls check your data --> odom correction"); 
                pointCloudBuf.pop();
                continue;              
            }

            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period || 
                                               pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period || 
                                               pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            pointCloudBuf.pop();

            Eigen::Quaterniond q_current(t265_odom.pose.pose.orientation.w,
                                         t265_odom.pose.pose.orientation.x,
                                         t265_odom.pose.pose.orientation.y,
                                         t265_odom.pose.pose.orientation.z);
            Eigen::Vector3d t_current(t265_odom.pose.pose.position.x,
                                      t265_odom.pose.pose.position.y,
                                      t265_odom.pose.pose.position.z);

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map"; 
            laserOdometry.child_frame_id = "base_link"; 
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);
        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period = 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100, velodyneHandler);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);
    ros::Subscriber subT265Odom = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 100, odomHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread odom_estimation_process{odom_estimation};

    ros::spin();

    odom_estimation_process.join();
    return 0;
}
