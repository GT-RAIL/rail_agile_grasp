#include <rail_agile_nodes/PointCloudClicker.h>

using namespace std;

PointCloudClicker::PointCloudClicker() :
    acFindGrasps("/rail_agile_grasp/find_grasps"), pnh("~"),
    clickedPointServer(pnh, "click_image_point", boost::bind(&PointCloudClicker::executeClickedPointCallback, this, _1), false),
    clickedPointCPServer(pnh, "click_image_point_cp", boost::bind(&PointCloudClicker::executeClickedPointCPCallback, this, _1), false)
{
  ROS_INFO("Point cloud clicker is starting...");
  //read parameters
  string cloudTopic("/camera/depth_registered/points");
  pnh.getParam("cloud_topic", cloudTopic);

  cloudSubscriber = n.subscribe(cloudTopic, 1, &PointCloudClicker::cloudCallback, this);

  Eigen::Vector4f minPoint;
  minPoint[0] = -1.0f;
  minPoint[1] = -1.0f;
  minPoint[2] = 0.3f;
  Eigen::Vector4f maxPoint;
  maxPoint[0] = 1.0f;
  maxPoint[1] = 1.0f;
  maxPoint[2] = 2.0f;
  cropBox.setMin(minPoint);
  cropBox.setMax(maxPoint);

  voxelGrid.setLeafSize(0.05f, 0.05f, 0.05f);

  imServer.reset( new interactive_markers::InteractiveMarkerServer("point_cloud_clicker", "clickable_point_markers", false));

  ros::Duration(0.1).sleep();

  imServer->applyChanges();

  interactiveCloud.pose.orientation.w = 1.0;
  interactiveCloud.scale = 1.0;
  interactiveCloud.name = "interactive_cloud_marker";
  interactiveCloud.description = "Clickable Point Cloud";

  newCloudReceived = false;
  cloudInitialized = false;

  changePointCloudTopicServer = pnh.advertiseService("change_point_cloud_topic", &PointCloudClicker::changePointCloudTopicCallback, this);
  updateMarkerServer = pnh.advertiseService("request_marker_update", &PointCloudClicker::requestMarkerUpdate, this);

  cpClient = n.serviceClient<remote_manipulation_markers::CreateSphere>("constrained_positioning/create_sphere");

  ROS_INFO("Starting action servers...");
  clickedPointServer.start();
  clickedPointCPServer.start();
  ROS_INFO("Action servers started.");
}

void PointCloudClicker::executeClickedPointCallback(const rail_agile_grasp_msgs::ClickImagePointGoalConstPtr &goal)
{
  rail_agile_grasp_msgs::ClickImagePointResult result;
  rail_agile_grasp_msgs::ClickImagePointFeedback feedback;

  feedback.message = "Waiting for camera data...";
  clickedPointServer.publishFeedback(feedback);

  ros::Rate loopRate(30);
  ros::Time startTime = ros::Time::now();
  while (!cloudInitialized)
  {
    loopRate.sleep();
    ros::spinOnce();

    if (ros::Time::now() - startTime >= ros::Duration(10.0))
    {
      feedback.message = "Could not receive camera data, an error has occurred.";
      clickedPointServer.publishFeedback(feedback);

      ROS_INFO("No point cloud received, could not calculate poses.");
      result.success = false;
      clickedPointServer.setSucceeded(result);
      return;
    }
  }

  feedback.message = "Calculating grasps at the clicked point...";
  clickedPointServer.publishFeedback(feedback);

  //convert clicked point to camera image point
  double px = ((double)(fullPointCloudPtr->width))/((double)goal->imageWidth) * goal->x;
  double py = ((double)(fullPointCloudPtr->height))/((double)goal->imageHeight) * goal->y;

  //lookup 3d point for camera image point
  const pcl::PointXYZRGB &point = fullPointCloudPtr->at(px, py);

  if (pcl::isFinite(point))
  {
    ROS_INFO("Calculating possible grasps for point %f, %f, %f, in frame %s...", point.x, point.y, point.z, fullPointCloudPtr->header.frame_id.c_str());

    rail_agile_grasp_msgs::FindGraspsGoal findGraspsGoal;
    findGraspsGoal.useClassifier = false;
    findGraspsGoal.workspace.mode = rail_agile_grasp_msgs::Workspace::CENTERED_ROI;
    findGraspsGoal.workspace.roiCenter.header.frame_id = fullPointCloudPtr->header.frame_id;
    findGraspsGoal.workspace.roiCenter.point.x = point.x;
    findGraspsGoal.workspace.roiCenter.point.y = point.y;
    findGraspsGoal.workspace.roiCenter.point.z = point.z;
    findGraspsGoal.workspace.roiDimensions.x = 0.1;
    findGraspsGoal.workspace.roiDimensions.y = 0.1;
    findGraspsGoal.workspace.roiDimensions.z = 0.1;

    acFindGrasps.sendGoal(findGraspsGoal);
    acFindGrasps.waitForResult(ros::Duration(7.0));

    if (!acFindGrasps.getResult()->success)
    {
      feedback.message = "Couldn't find any grasps, please try another point.";
      clickedPointServer.publishFeedback(feedback);
      result.success = false;
    }
    else
    {
      feedback.message = "Grasps calculated successfully.";
      clickedPointServer.publishFeedback(feedback);
      result.success = true;
    }
  }
  else
  {
    feedback.message = "There was an error reading the clicked point. Please try again.";
    clickedPointServer.publishFeedback(feedback);
    result.success = false;
  }

  clickedPointServer.setSucceeded(result);
}

void PointCloudClicker::executeClickedPointCPCallback(const rail_agile_grasp_msgs::ClickImagePointGoalConstPtr &goal)
{
  rail_agile_grasp_msgs::ClickImagePointResult result;
  rail_agile_grasp_msgs::ClickImagePointFeedback feedback;

  feedback.message = "Waiting for camera data...";
  clickedPointCPServer.publishFeedback(feedback);

  ros::Rate loopRate(30);
  ros::Time startTime = ros::Time::now();
  while (!cloudInitialized)
  {
    loopRate.sleep();
    ros::spinOnce();

    if (ros::Time::now() - startTime >= ros::Duration(10.0))
    {
      feedback.message = "Could not receive camera data, an error has occurred.";
      clickedPointCPServer.publishFeedback(feedback);

      ROS_INFO("No point cloud received, could not create grasp area.");
      result.success = false;
      clickedPointCPServer.setSucceeded(result);
      return;
    }
  }

  feedback.message = "Setting a grasp area at the clicked point...";
  clickedPointCPServer.publishFeedback(feedback);

  //convert clicked point to camera image point
  double px = ((double)(fullPointCloudPtr->width))/((double)goal->imageWidth) * goal->x;
  double py = ((double)(fullPointCloudPtr->height))/((double)goal->imageHeight) * goal->y;

  //lookup 3d point for camera image point
  const pcl::PointXYZRGB &point = fullPointCloudPtr->at(px, py);

  if (pcl::isFinite(point))
  {
    ROS_INFO("Initializing constrained positioning marker at point %f, %f, %f, in frame %s...", point.x, point.y, point.z, fullPointCloudPtr->header.frame_id.c_str());

    geometry_msgs::PointStamped inputPoint;
    inputPoint.point.x = point.x;
    inputPoint.point.y = point.y;
    inputPoint.point.z = point.z;
    inputPoint.header.frame_id = fullPointCloudPtr->header.frame_id;

    geometry_msgs::PointStamped transformedPoint;
    transformedPoint.header.frame_id = "table_base_link";
    tfListener.transformPoint("table_base_link", inputPoint, transformedPoint);

    remote_manipulation_markers::CreateSphere srv;
    srv.request.point = transformedPoint;
    if (!cpClient.call(srv))
    {
      feedback.message = "There was an error setting a new grasp area.";
      clickedPointCPServer.publishFeedback(feedback);
      result.success = false;
    }
    else
    {
      feedback.message = "New grasp area set.";
      clickedPointCPServer.publishFeedback(feedback);
      result.success = true;
    }
  }
  else
  {
    feedback.message = "There was an error reading the clicked point. Please try again.";
    clickedPointCPServer.publishFeedback(feedback);
    result.success = false;
  }

  clickedPointCPServer.setSucceeded(result);
}

void PointCloudClicker::cloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pc)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);

  fullPointCloudPtr = pc;

  //crop point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cropBox.setInputCloud(pc);
  cropBox.filter(*croppedCloud);

  //downsample point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  voxelGrid.setInputCloud(croppedCloud);
  voxelGrid.filter(*filteredCloud);

  //convert to sensor_msgs::PointCloud2
  pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*filteredCloud, *tempCloud);
  pcl_conversions::fromPCL(*tempCloud, cloud);

  newCloudReceived = true;
  cloudInitialized = true;
}

bool PointCloudClicker::changePointCloudTopicCallback(rail_agile_grasp_msgs::ChangePointCloud::Request &req, rail_agile_grasp_msgs::ChangePointCloud::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);
  cloudSubscriber = n.subscribe(req.cloudTopic, 1, &PointCloudClicker::cloudCallback, this);

  newCloudReceived = false;
  cloudInitialized = false;

  return true;
}

void PointCloudClicker::processInteractiveCloudFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);
  switch (feedback->event_type)
  {
    //Send a stop command so that when the marker is released the arm stops moving
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      if (feedback->mouse_point.x == 0 && feedback->mouse_point.y == 0 && feedback->mouse_point.z == 0)
      {
        ROS_INFO("invalid click!");
      }
      else
      {
        const pcl::PointXYZRGB &centerPoint = fullPointCloudPtr->at(((double)(fullPointCloudPtr->width))/2.0, ((double)(fullPointCloudPtr->height))/2.0);
        geometry_msgs::PointStamped centerPointRos;
        centerPointRos.point.x = centerPoint.x;
        centerPointRos.point.y = centerPoint.y;
        centerPointRos.point.z = centerPoint.z;
        centerPointRos.header.frame_id = fullPointCloudPtr->header.frame_id;
        geometry_msgs::PointStamped centerPointTransformed;
        centerPointTransformed.header.frame_id = "table_base_link";
        tfListener.transformPoint("table_base_link", centerPointRos, centerPointTransformed);
        ROS_INFO("Center point: %f, %f, %f", centerPointTransformed.point.x, centerPointTransformed.point.y, centerPointTransformed.point.z);

        ROS_INFO("Calculating possible grasps for point %f, %f, %f, in frame %s...", feedback->mouse_point.x, feedback->mouse_point.y, feedback->mouse_point.z, feedback->header.frame_id.c_str());

        rail_agile_grasp_msgs::FindGraspsGoal findGraspsGoal;
        findGraspsGoal.useClassifier = false;
        findGraspsGoal.workspace.mode = rail_agile_grasp_msgs::Workspace::CENTERED_ROI;
        findGraspsGoal.workspace.roiCenter.header.frame_id = feedback->header.frame_id;
        findGraspsGoal.workspace.roiCenter.point = feedback->mouse_point;
        findGraspsGoal.workspace.roiDimensions.x = 0.1;
        findGraspsGoal.workspace.roiDimensions.y = 0.1;
        findGraspsGoal.workspace.roiDimensions.z = 0.1;

        acFindGrasps.sendGoal(findGraspsGoal);
      }
      break;
    default:
      break;
  }

  //Update interactive marker server
  imServer->applyChanges();
}

bool PointCloudClicker::requestMarkerUpdate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  updateMarker();

  return true;
}

void PointCloudClicker::updateMarker()
{
  if (newCloudReceived)
  {
    boost::recursive_mutex::scoped_lock lock(cloudMutex);

    interactiveCloud.controls.clear();

    /*
    //create new marker
    visualization_msgs::Marker marker;
    // set header field
    marker.header.frame_id = cloud.header.frame_id;

    // default position
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // default scale
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    // set the type of marker and our color of choice
    marker.type = visualization_msgs::Marker::POINTS;
    */

    // convert to an easy to use point cloud message
    sensor_msgs::PointCloud pc;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud, pc);

    // place in the marker message
    /*
    marker.points.resize(pc.points.size());
    marker.colors.resize(pc.points.size());
    */
    //create sphere marker for each point
    visualization_msgs::InteractiveMarkerControl clickControl;
    clickControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    clickControl.name = "interactive_cloud_marker_control";
    for (size_t j = 0; j < pc.points.size(); j++)
    {
      visualization_msgs::Marker marker;
      //marker.header.frame_id = cloud.header.frame_id;

      marker.pose.position.x = pc.points[j].x;
      marker.pose.position.y = pc.points[j].y;
      marker.pose.position.z = pc.points[j].z;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;

      marker.type = visualization_msgs::Marker::SPHERE;
      uint32_t rgb = *reinterpret_cast<int *>(&pc.channels[0].values[j]);
      marker.color.r = (float)((int) ((rgb >> 16) & 0x0000ff))/255.0f;
      marker.color.g = (float)((int) ((rgb >> 8) & 0x0000ff))/255.0f;
      marker.color.b = (float)((int) ((rgb) & 0x0000ff))/255.0f;
      marker.color.a = 1.0;

      clickControl.markers.push_back(marker);
      /*
      marker.points[j].x = pc.points[j].x;
      marker.points[j].y = pc.points[j].y;
      marker.points[j].z = pc.points[j].z;

      // use average RGB
      uint32_t rgb = *reinterpret_cast<int *>(&pc.channels[0].values[j]);
      marker.colors[j].r = (float)((int) ((rgb >> 16) & 0x0000ff))/255.0f;
      marker.colors[j].g = (float)((int) ((rgb >> 8) & 0x0000ff))/255.0f;
      marker.colors[j].b = (float)((int) ((rgb) & 0x0000ff))/255.0f;
      marker.colors[j].a = 1.0;
      */
    }

    //update interactive marker
    /*
    interactiveCloud.header.frame_id = cloud.header.frame_id;
    visualization_msgs::InteractiveMarkerControl clickControl;
    clickControl.markers.push_back(marker);
    clickControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    clickControl.name = "interactive_cloud_marker_control";
    */
    interactiveCloud.header.frame_id = cloud.header.frame_id;
    interactiveCloud.controls.push_back(clickControl);
    imServer->clear();
    imServer->insert(interactiveCloud);
    imServer->setCallback(interactiveCloud.name, boost::bind(&PointCloudClicker::processInteractiveCloudFeedback, this, _1));
    imServer->applyChanges();
    newCloudReceived = false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_clicker");

  PointCloudClicker pcc;

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    //pcc.updateMarker();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
