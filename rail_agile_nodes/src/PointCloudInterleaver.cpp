#include "rail_agile_nodes/PointCloudInterleaver.h"

using namespace std;

PointCloudInterleaver::PointCloudInterleaver() : pnh("~")
{
  string cloud1Topic("/camera1/depth_registered/points");
  string cloud2Topic("/camera2/depth_registered/points");
  pnh.getParam("cloud1_topic", cloud1Topic);
  pnh.getParam("cloud2_topic", cloud2Topic);

  cloud1Subscriber = n.subscribe(cloud1Topic, 1, &PointCloudInterleaver::cloud1Callback, this);
  cloud2Subscriber = n.subscribe(cloud2Topic, 1, &PointCloudInterleaver::cloud2Callback, this);
  cloudPublisher = pnh.advertise<sensor_msgs::PointCloud2>("interleaved_clouds", 1);

  cloud1Received = false;
  cloud2Received = false;
}

void PointCloudInterleaver::cloud1Callback(const sensor_msgs::PointCloud2 &pc)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);
  cloud1 = pc;
  cloud1Received = true;
}

void PointCloudInterleaver::cloud2Callback(const sensor_msgs::PointCloud2 &pc)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);
  cloud2 = pc;
  cloud2Received = true;
}

void PointCloudInterleaver::publishClouds()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex);
  if (cloud1Received && cloud2Received)
  {
    //transform second cloud to frame of first cloud
    sensor_msgs::PointCloud c2;
    sensor_msgs::PointCloud c2Transformed;
    sensor_msgs::PointCloud2 cloud2Final;
    cloud2Final.header.frame_id = cloud1.header.frame_id;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2, c2);
    c2Transformed.header.frame_id = cloud1.header.frame_id;
    //transformListener.transformPointCloud(cloud1.header.frame_id, c2, c2Transformed);
    transformListener.transformPointCloud(cloud1.header.frame_id, ros::Time(0), c2, c2.header.frame_id, c2Transformed);
    sensor_msgs::convertPointCloudToPointCloud2(c2Transformed, cloud2Final);

    cloudPublisher.publish(cloud1);
    cloudPublisher.publish(cloud2Final);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "agile_test_cloud_interleaver");
  PointCloudInterleaver pci;

  ros::Rate loopRate(60);
  while (ros::ok())
  {
    pci.publishClouds();
    ros::spinOnce();
    loopRate.sleep();
  }

  return EXIT_SUCCESS;
}