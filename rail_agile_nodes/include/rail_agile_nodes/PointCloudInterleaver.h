#ifndef POINT_CLOUD_FILTER_H_
#define POINT_CLOUD_FILTER_H_

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>

// C++
#include <boost/thread/recursive_mutex.hpp>

class PointCloudInterleaver
{
public:
  PointCloudInterleaver();

  void publishClouds();

private:
  void cloud1Callback(const sensor_msgs::PointCloud2 &pc);
  void cloud2Callback(const sensor_msgs::PointCloud2 &pc);

  ros::NodeHandle n, pnh;
  ros::Subscriber cloud1Subscriber;
  ros::Subscriber cloud2Subscriber;
  ros::Publisher cloudPublisher;

  bool cloud1Received;
  bool cloud2Received;

  sensor_msgs::PointCloud2 cloud1;
  sensor_msgs::PointCloud2 cloud2;
  tf::TransformListener transformListener;

  boost::recursive_mutex api_mutex;
};

#endif
