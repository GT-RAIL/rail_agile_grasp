#ifndef POINT_CLOUD_CLICKER_H_
#define POINT_CLOUD_CLICKER_H_

//ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <remote_manipulation_markers/CreateSphere.h>
#include <pcl_ros/point_cloud.h>
#include <rail_agile_grasp_msgs/ChangePointCloud.h>
#include <rail_agile_grasp_msgs/ClickImagePointAction.h>
#include <rail_agile_grasp_msgs/FindGraspsAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudClicker
{

public:

  /**
   * \brief Constructor
   */
  PointCloudClicker();

  void updateMarker();

private:
  void processInteractiveCloudFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc);

  bool changePointCloudTopicCallback(rail_agile_grasp_msgs::ChangePointCloud::Request &req, rail_agile_grasp_msgs::ChangePointCloud::Response &res);

  bool requestMarkerUpdate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void executeClickedPointCallback(const rail_agile_grasp_msgs::ClickImagePointGoalConstPtr &goal);

  void executeClickedPointNavidgetCallback(const rail_agile_grasp_msgs::ClickImagePointGoalConstPtr &goal);

  ros::NodeHandle n;
  ros::NodeHandle pnh;

  //topics
  ros::Subscriber cloudSubscriber;

  //services
  ros::ServiceServer changePointCloudTopicServer;
  ros::ServiceServer updateMarkerServer;
  ros::ServiceClient navidgetClient;

  //actionlib
  actionlib::SimpleActionClient<rail_agile_grasp_msgs::FindGraspsAction> acFindGrasps;
  actionlib::SimpleActionServer<rail_agile_grasp_msgs::ClickImagePointAction> clickedPointServer;
  actionlib::SimpleActionServer<rail_agile_grasp_msgs::ClickImagePointAction> clickedPointNavidgetServer;

  boost::recursive_mutex cloudMutex;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;
  visualization_msgs::InteractiveMarker interactiveCloud;

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr fullPointCloudPtr;
  sensor_msgs::PointCloud2 cloud;
  pcl::CropBox<pcl::PointXYZRGB> cropBox;
  pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;

  bool newCloudReceived;  //flag for whether a new point cloud was received since the previous main loop execution
  bool cloudInitialized;  //flag for first point cloud received from a new topic

  tf::TransformListener tfListener;
};

#endif
