//
// Created by woods on 2023/7/5.
//
#include "target_detection_sim/target_detection_sim.h"

int main(int argc, char *argv[])
{

    //=========== Generate random target position and store in an octree for fast searching.
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    static_target = generateRandomPoints(minX,maxX,minY,maxY,minZ,maxZ,targent_number);
    octree.setInputCloud(static_target);
    octree.addPointsFromInputCloud();
    //=========== Generate random target position and store in an octree for fast searching.




    ros::init(argc, argv, "target_detection_sim");
    ros::NodeHandle nh;
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), octreeSearchCallback);

    return 0;
}