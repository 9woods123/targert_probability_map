//
// Created by woods on 2023/7/5.
//


#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <pcl/octree/octree_search.h>

ros::Subscriber robot_pose_sub;
geometry_msgs::PoseStamped current_pose;



double minX = -100;
double maxX =  100;
double minY = -100;
double maxY =  100;
double minZ = -100;
double maxZ = -75;
size_t targent_number = 1000;  // Number of points

float search_radius =20.0f;  // Search radius for finding points within the given distance
float resolution = 0.5f;  // Voxel size

pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);



void octreeSearchCallback(const ros::TimerEvent& event) {

    pcl::PointXYZ search_point;
    search_point.x = current_pose.pose.position.x;
    search_point.y = current_pose.pose.position.y;
    search_point.z = current_pose.pose.position.z;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;


    if (octree.radiusSearch(search_point, search_radius,
                            pointIdxRadiusSearch,pointRadiusSquaredDistance) > 0)
    {
        // Point(s) found within the search radius
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            pcl::PointXYZ nearby_point = cloud->points[pointIdxRadiusSearch[i]];
        }

    }

}


void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose=*msg;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr generateRandomPoints
        (double min_x, double max_x, double min_y, double max_y, double min_z, double max_z, size_t n){

    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> dist_x(min_x, max_x);
    std::uniform_real_distribution<double> dist_y(min_y, max_y);
    std::uniform_real_distribution<double> dist_z(min_z, max_z);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = n;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (int i = 0; i < n; ++i) {
        cloud->points[i].x=dist_x(gen);
        cloud->points[i].y=dist_y(gen);
        cloud->points[i].z=dist_z(gen);
    }
    return cloud;
}