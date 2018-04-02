#include <iostream>

#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"



class converter {
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;

  ros::Publisher vert_pcl;

  ros::Subscriber laser;

  laser_geometry::LaserProjection projector_;


public:
  converter(ros::NodeHandle &nh) {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic

    laser = nh.subscribe<sensor_msgs::LaserScan>("lidar1/scan", 100, &converter::processLaserScan, this);

    vert_pcl = nh.advertise<sensor_msgs::PointCloud2>("pcl_v", 100, true);


  }




  void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {

    sensor_msgs::LaserScan laser = *scan;

    sensor_msgs::PointCloud2 cloud;

    projector_.projectLaser(laser, cloud);

    vert_pcl.publish(cloud);

  }


};




int main(int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "laser_to_pcl");

  ros::NodeHandle nh;
  converter laser(nh);


  ros::spin();


}
