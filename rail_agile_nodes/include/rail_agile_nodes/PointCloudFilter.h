#ifndef POINT_CLOUD_FILTER_H_
#define POINT_CLOUD_FILTER_H_

// ROS
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudFilter
{
public:
  PointCloudFilter();

private:
  void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

  ros::NodeHandle n, pnh;
  ros::Subscriber cloudSubscriber;
  ros::Publisher cloudPublisher;

  pcl::CropBox<pcl::PointXYZRGB> cropBox;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
};

#endif
