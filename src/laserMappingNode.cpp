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
#include "laserMappingClass.h"
#include "lidar.h"


LaserMappingClass laserMapping;
lidar::Lidar lidar_param;
std::mutex mutex_lock;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
ros::Publisher map_pub;
ros::Publisher pubLaserOdometry;
std::queue<nav_msgs::OdometryConstPtr> odometry265Buf;
//nav_msgs::Odometry odometry265;

int update_count = 0;
int frame_id=0;
double total_time =0;
int odom_count =0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    mutex_lock.lock();
    pointCloudBuf.push(msg);
    mutex_lock.unlock();
}

void odom265Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //odom_count++;
    //if(odom_count%5!=0)
    //    return;

    mutex_lock.lock();
    odometry265Buf.push(msg);
    mutex_lock.unlock();
}


void laser_mapping(){
    while(1){
        //ROS_WARN("laser_mapping"); 
        //if(!pointCloudBuf.empty()){
            //ROS_WARN("Odometry published"); 
            // Publish odometry
            /*ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation = odometry265.pose.pose.orientation;
            laserOdometry.pose.pose.position = odometry265.pose.pose.position;
            pubLaserOdometry.publish(laserOdometry);*/

            if (!pointCloudBuf.empty() && !odometry265Buf.empty()) {

                // Print the sizes of the buffers
                ROS_INFO("Size of pointCloudBuf: %ld", pointCloudBuf.size());
                ROS_INFO("Size of odometry265Buf: %ld", odometry265Buf.size());


                //ROS_WARN("read data"); 
                //read data
                mutex_lock.lock();
                if(!pointCloudBuf.empty() && odometry265Buf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period > pointCloudBuf.front()->header.stamp.toSec()){
                    double time_diff = (odometry265Buf.front()->header.stamp.toSec()-(0.5*lidar_param.scan_period)) - pointCloudBuf.front()->header.stamp.toSec();
                    ROS_WARN("time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node 1"); 
                    ROS_WARN("Time difference: %f seconds", time_diff);  // Added print statement
                    ROS_INFO("Scan period: %f seconds", 0.5*lidar_param.scan_period);  // Added print statement
                    pointCloudBuf.pop();
                    mutex_lock.unlock();
                    continue;            
                }

                if(!odometry265Buf.empty() && pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period > odometry265Buf.front()->header.stamp.toSec()){
                    double time_diff = (pointCloudBuf.front()->header.stamp.toSec()-(0.5*lidar_param.scan_period)) - odometry265Buf.front()->header.stamp.toSec();
                    ROS_INFO("time stamp unaligned with path final, pls check your data --> laser mapping node 2");
                    ROS_INFO("Time difference: %f seconds", time_diff); 
                    ROS_INFO("Scan period: %f seconds", 0.5*lidar_param.scan_period);  // Added print statement
                    odometry265Buf.pop();
                    mutex_lock.unlock();
                    continue;  
                }

                ROS_WARN("Odometry published start"); 
                //if time aligned 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
                ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;

                Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
                current_pose.rotate(Eigen::Quaterniond(odometry265Buf.front()->pose.pose.orientation.w,odometry265Buf.front()->pose.pose.orientation.x,odometry265Buf.front()->pose.pose.orientation.y,odometry265Buf.front()->pose.pose.orientation.z));  
                current_pose.pretranslate(Eigen::Vector3d(odometry265Buf.front()->pose.pose.position.x,odometry265Buf.front()->pose.pose.position.y,odometry265Buf.front()->pose.pose.position.z));
                pointCloudBuf.pop();
                odometry265Buf.pop();
                mutex_lock.unlock();
                
                
                update_count++;
                Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
                double displacement = delta_transform.translation().squaredNorm();
                double angular_change = delta_transform.linear().eulerAngles(2,1,0)[0]* 180 / M_PI;

                if(angular_change>90) angular_change = fabs(180 - angular_change);
                
                if(displacement>0.3 || angular_change>20){
                    ROS_INFO("update map %f,%f",displacement,angular_change);
                    last_pose = current_pose;
                    laserMapping.updateCurrentPointsToMap(pointcloud_in,current_pose);

                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_map = laserMapping.getMap();
                    sensor_msgs::PointCloud2 PointsMsg;
                    pcl::toROSMsg(*pc_map, PointsMsg);
                    PointsMsg.header.stamp = pointcloud_time;
                    PointsMsg.header.frame_id = "map";
                    map_pub.publish(PointsMsg); 
                }

                ROS_WARN("Odometry published end"); 
            }
        //}

        //sleep 2 ms every time
        std::chrono::milliseconds dura(200);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis); //
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);
    
    laserMapping.init(map_resolution);
    last_pose.translation().x() = 10;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 10, velodyneHandler);
    //ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 10, odomCallback);
    ros::Subscriber subT265Odom = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 10, odom265Callback);

    //pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 10);
    std::thread laser_mapping_process{laser_mapping};

    ros::spin();

    return 0;
}
