#include "rail_agile_nodes/PointCloudFilter.h"

using namespace std;

PointCloudFilter::PointCloudFilter() : pnh("~")
{
  Eigen::Vector4f minPoint;
  minPoint[0] = -0.33f;
  minPoint[1] = -0.2f;
  minPoint[2] = 0.53f;
  Eigen::Vector4f maxPoint;
  maxPoint[0] = 0.33f;
  maxPoint[1] = 0.2f;
  maxPoint[2] = 0.95f;
  cropBox.setMin(minPoint);
  cropBox.setMax(maxPoint);

  voxelGrid.setLeafSize(0.005f, 0.005f, 0.005f);

  string cloudTopic("/camera/depth_registered/points");
  pnh.getParam("cloud_topic", cloudTopic);

  cloudSubscriber = n.subscribe(cloudTopic, 1, &PointCloudFilter::cloudCallback, this);
  cloudPublisher = pnh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
}

void PointCloudFilter::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc)
{
  //filter the cloud, publish it
  pcl::PointCloud<pcl::PointXYZRGB> cloud(*pc);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOutPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  cropBox.setInputCloud(cloudPtr);
  cropBox.filter(*cloudOutPtr);

  cloudPublisher.publish(cloudOutPtr);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agile_test_cloud_filter");
  PointCloudFilter pcf;

  ros::spin();

  return EXIT_SUCCESS;
}