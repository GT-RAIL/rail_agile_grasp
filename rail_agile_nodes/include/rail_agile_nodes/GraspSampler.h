#ifndef GRASP_SAMPLER_H_
#define GRASP_SAMPLER_H_

// ROS
#include <actionlib/client/simple_action_client.h>
#include <rail_agile_nodes/PoseWithHeuristic.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <rail_agile_grasp_msgs/ChangePointCloud.h>
#include <rail_agile_grasp_msgs/GraspsWithWorkspace.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// C++
#include <algorithm>
#include <boost/thread/recursive_mutex.hpp>

class GraspSampler
{
public:
  GraspSampler();

private:
  void cloudCallback(const sensor_msgs::PointCloud2 &pc);

  bool changePointCloudTopicCallback(rail_agile_grasp_msgs::ChangePointCloud::Request &req, rail_agile_grasp_msgs::ChangePointCloud::Response &res);

  void graspsCallback(const rail_agile_grasp_msgs::GraspsWithWorkspace &candidates);

  Eigen::Quaterniond computePrincipalDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string);

  double static squaredDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2);

  double static squaredDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);

  double static squaredDistance(Eigen::Vector3d v1, Eigen::Vector3d v2);

  ros::NodeHandle n, pnh;

  ros::Subscriber graspSubscriber;
  ros::Subscriber cloudSubscriber;
  ros::Publisher graspsPublisher;
  ros::Publisher debugPublisher;

  ros::ServiceServer changePointCloudTopicServer;

  boost::recursive_mutex cloudMutex;

  double neighborhoodRadius;
  double orientationThreshold;
  int clusterSize;
  bool cloudReceived;
  bool removeTable;
  sensor_msgs::PointCloud2 cloud;
  tf::TransformBroadcaster tfBroadcaster;
  tf::TransformListener tfListener;
};

#endif
