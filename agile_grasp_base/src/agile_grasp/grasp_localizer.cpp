#include <agile_grasp/grasp_localizer.h>


GraspLocalizer::GraspLocalizer(ros::NodeHandle& node, const std::string& cloud_topic, 
  const std::string& cloud_frame, int cloud_type, const std::string& svm_file_name, 
  const Parameters& params) 
: cloud_left_(new PointCloud()), cloud_right_(new PointCloud()),
cloud_frame_(cloud_frame), svm_file_name_(svm_file_name), num_clouds_(params.num_clouds_),
num_clouds_received_(0), size_left_(0), server(node, "/rail_agile_grasp/find_grasps", boost::bind(&GraspLocalizer::executeFind, this, _1), false)
{
  cloud_type_ = cloud_type;

  // subscribe to input point cloud ROS topic
  if (cloud_type == CLOUD_SIZED)
    cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspLocalizer::cloud_sized_callback, this);
  else if (cloud_type == POINT_CLOUD_2)
    cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspLocalizer::cloud_callback, this);

  // create ROS publisher for grasps
  grasps_pub_ = node.advertise<agile_grasp_base::Grasps>("grasps", 10);
  grasps_with_workspace_pub_ = node.advertise<rail_agile_grasp_msgs::GraspsWithWorkspace>("grasps_with_workspace", 10);

  // setup ROS services
  change_point_cloud_topic_server_ = node.advertiseService("/rail_agile_grasp/change_point_cloud_topic", &GraspLocalizer::changePointCloudTopicCallback, this);

  // create localization object and initialize its parameters
  localization_ = new Localization(params.num_threads_, true, params.plotting_mode_);
  localization_->setCameraTransforms(params.cam_tf_left_, params.cam_tf_right_);
  localization_->setWorkspace(params.workspace_);
  localization_->setNumSamples(params.num_samples_);
  localization_->setFingerWidth(params.finger_width_);
  localization_->setHandOuterDiameter(params.hand_outer_diameter_);
  localization_->setHandDepth(params.hand_depth_);
  localization_->setInitBite(params.init_bite_);
  localization_->setHandHeight(params.hand_height_);

  min_inliers_ = params.min_inliers_;

  if (params.plotting_mode_ == 0)
  {
    plots_handles_ = false;
  }    
  else
  {
    plots_handles_ = false;    
    if (params.plotting_mode_ == 2)
      localization_->createVisualsPub(node, params.marker_lifetime_, cloud_left_->header.frame_id);
  }

  server.start();
  ROS_INFO("server is running");
}


void GraspLocalizer::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);

  if (num_clouds_received_ == num_clouds_)
    return;

  if (num_clouds_received_ == 0)
    pcl::fromROSMsg(*msg, *cloud_left_);
  else if (num_clouds_received_ == 1)
    pcl::fromROSMsg(*msg, *cloud_right_);
  std::cout << "Received cloud # " << num_clouds_received_ << " with " << msg->height * msg->width << " points\n";
  num_clouds_received_++;
}


void GraspLocalizer::cloud_sized_callback(const agile_grasp_base::CloudSized& msg)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);

  pcl::fromROSMsg(msg.cloud, *cloud_left_);
  size_left_ = msg.size_left.data;
  std::cout << "Received cloud with size_left: " << size_left_ << std::endl;
  num_clouds_received_ = 1;
}

void GraspLocalizer::executeFind(const rail_agile_grasp_msgs::FindGraspsGoalConstPtr &goal) {
  rail_agile_grasp_msgs::FindGraspsFeedback feedback;
  rail_agile_grasp_msgs::FindGraspsResult result;

  num_clouds_received_ = 0; //reset so current point clouds are used

  ROS_INFO("Received action request");
  ros::Rate rate(30);
  std::vector<int> indices(0);

  while (!server.isPreemptRequested() && ros::ok()) 
  {
    // wait for point clouds to arrive
    if (num_clouds_received_ == num_clouds_)
    {
      boost::recursive_mutex::scoped_lock lock(cloudMutex);

      // set the new workspace based on actionlib goal message
      Eigen::VectorXd ws(6);
      if (goal->workspace.mode == rail_agile_grasp_msgs::Workspace::WORKSPACE_VOLUME)
      {
        ws << goal->workspace.x_min, goal->workspace.x_max, goal->workspace.y_min, goal->workspace.y_max, goal->workspace.z_min, goal->workspace.z_max;
        ROS_INFO("%f,%f,%f,%f,%f,%f", goal->workspace.x_min, goal->workspace.x_max, goal->workspace.y_min, goal->workspace.y_max, goal->workspace.z_min, goal->workspace.z_max);
      }
      else if (goal->workspace.mode == rail_agile_grasp_msgs::Workspace::CENTERED_ROI)
      {
        if (goal->workspace.roiCenter.header.frame_id != cloud_left_->header.frame_id)
        {
          //Transform to point cloud frame first
          geometry_msgs::PointStamped transformedRoiCenter;
          transformedRoiCenter.header.stamp = ros::Time(0);
          transformedRoiCenter.header.frame_id = cloud_left_->header.frame_id;
          tfListener.transformPoint(cloud_left_->header.frame_id, goal->workspace.roiCenter, transformedRoiCenter);
          ws << transformedRoiCenter.point.x - goal->workspace.roiDimensions.x/2.0, transformedRoiCenter.point.x + goal->workspace.roiDimensions.x/2.0,
              transformedRoiCenter.point.y - goal->workspace.roiDimensions.y/2.0, transformedRoiCenter.point.y + goal->workspace.roiDimensions.y/2.0,
              transformedRoiCenter.point.z - goal->workspace.roiDimensions.z/2.0, transformedRoiCenter.point.z + goal->workspace.roiDimensions.z/2.0;
        }
        else
        {
          ws << goal->workspace.roiCenter.point.x - goal->workspace.roiDimensions.x/2.0, goal->workspace.roiCenter.point.x + goal->workspace.roiDimensions.x/2.0,
              goal->workspace.roiCenter.point.y - goal->workspace.roiDimensions.y/2.0, goal->workspace.roiCenter.point.y + goal->workspace.roiDimensions.y/2.0,
              goal->workspace.roiCenter.point.z - goal->workspace.roiDimensions.z/2.0, goal->workspace.roiCenter.point.z + goal->workspace.roiDimensions.z/2.0;
        }
      }
      localization_->setWorkspace(ws);

      feedback.current_action = "Localizing hands";
      server.publishFeedback(feedback);
          // localize grasps
      if (num_clouds_ > 1)
      {
        PointCloud::Ptr cloud(new PointCloud());
        *cloud = *cloud_left_ + *cloud_right_;
        hands_ = localization_->localizeHands(cloud, cloud_left_->size(), indices, false, false);
      }
      else
      {
        hands_ = localization_->localizeHands(cloud_left_, cloud_left_->size(), indices, false, false);
      }

      if (!goal->useClassifier)
      {
        rail_agile_grasp_msgs::GraspsWithWorkspace finalGrasps;
        finalGrasps.grasps = createRailGraspsMsg(hands_);
        finalGrasps.workspace = goal->workspace;
        grasps_with_workspace_pub_.publish(finalGrasps);
        if (finalGrasps.grasps.grasps.empty())
          result.success = false;
        else
          result.success = true;
        server.setSucceeded(result);
        return;
      }

      feedback.current_action = "Classifying hands as antipodal";
      server.publishFeedback(feedback);
      antipodal_hands_ = localization_->predictAntipodalHands(hands_, svm_file_name_);

      feedback.current_action = "Finding final handles";
      server.publishFeedback(feedback);
      handles_ = localization_->findHandles(antipodal_hands_, min_inliers_, 0.005);

          // publish handles
      grasps_pub_.publish(createGraspsMsg(handles_));
      ros::Duration(1.0).sleep();

          // publish hands contained in handles
      grasps_pub_.publish(createGraspsMsgFromHands(handles_));
      ros::Duration(1.0).sleep();

          // reset
      num_clouds_received_ = 0;

      result.success = true;
      server.setSucceeded(result);
      return;
    }
    ros::spinOnce();
    rate.sleep();
  }

  if (server.isPreemptRequested())
  {
    ROS_INFO("FindGrasps is preempted");
    server.setPreempted();
  }

  //server.publishFeedback(feedback);
}

bool GraspLocalizer::changePointCloudTopicCallback(rail_agile_grasp_msgs::ChangePointCloud::Request &req, rail_agile_grasp_msgs::ChangePointCloud::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);

  if (cloud_type_ == CLOUD_SIZED)
  {
    cloud_sub_ = node_.subscribe(req.cloudTopic, 1, &GraspLocalizer::cloud_sized_callback, this);
    cout << "Subscribed to " << req.cloudTopic << " for agile cloud callback" << endl;
  }
  else if (cloud_type_ == POINT_CLOUD_2)
  {
    cloud_sub_ = node_.subscribe(req.cloudTopic, 1, &GraspLocalizer::cloud_callback, this);
    cout << "Subscribed to " << req.cloudTopic << " for sensor_msgs cloud callback" << endl;
  }
  else
  {
    cout << "Did not resubscribe!" << endl;
  }

  num_clouds_received_ = 0;

  return true;
}

void GraspLocalizer::localizeGrasps()
{
  ros::Rate rate(1);
  std::vector<int> indices(0);

  while (ros::ok())
  {
    // wait for point clouds to arrive
    if (num_clouds_received_ == num_clouds_)
    {
      ROS_INFO("Localizing hands...");
      // localize grasps
      if (num_clouds_ > 1)
      {
        PointCloud::Ptr cloud(new PointCloud());
        *cloud = *cloud_left_ + *cloud_right_;
        hands_ = localization_->localizeHands(cloud, cloud_left_->size(), indices, false, false);
      }
      else
      {
        hands_ = localization_->localizeHands(cloud_left_, cloud_left_->size(), indices, false, false);
      }
      ROS_INFO("Hands localized.");

      ROS_INFO("Predicting antipodal hands...");
      antipodal_hands_ = localization_->predictAntipodalHands(hands_, svm_file_name_);
      ROS_INFO("Antipodal hands predicted.");
      ROS_INFO("Finding handles...");
      handles_ = localization_->findHandles(antipodal_hands_, min_inliers_, 0.005);
      ROS_INFO("Handles found.");


      // publish handles
      grasps_pub_.publish(createGraspsMsg(handles_));
      ros::Duration(1.0).sleep();

      // publish hands contained in handles
      grasps_pub_.publish(createGraspsMsgFromHands(handles_));
      ros::Duration(1.0).sleep();
      ROS_INFO("Publishing complete.");

      // reset
      num_clouds_received_ = 0;

      ros::spinOnce();
      rate.sleep();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void GraspLocalizer::findGrasps()
{
  ros::Rate rate(1);
  std::vector<int> indices(0);

  while (ros::ok())
  {
    // wait for point clouds to arrive
    if (num_clouds_received_ == num_clouds_)
    {
      ROS_INFO("Localizing hands...");
      // localize grasps
      if (num_clouds_ > 1)
      {
        PointCloud::Ptr cloud(new PointCloud());
        *cloud = *cloud_left_ + *cloud_right_;
        hands_ = localization_->localizeHands(cloud, cloud_left_->size(), indices, false, false);
      }
      else
      {
        hands_ = localization_->localizeHands(cloud_left_, cloud_left_->size(), indices, false, false);
      }
      ROS_INFO("Hands localized.");

      grasps_pub_.publish(createGraspsMsg(hands_));
      ROS_INFO("Published hands.");
      ros::spinOnce();
      rate.sleep();
      return;
    }

    ros::spinOnce();
    rate.sleep();
  }
}


agile_grasp_base::Grasps GraspLocalizer::createGraspsMsg(const std::vector<GraspHypothesis>& hands)
{
  agile_grasp_base::Grasps msg;

  for (int i = 0; i < hands.size(); i++)
  {
    msg.grasps.push_back(createGraspMsg(hands[i]));
  }

  msg.header.stamp = ros::Time::now();  
  return msg;
}



agile_grasp_base::Grasp GraspLocalizer::createGraspMsg(const GraspHypothesis& hand)
{
  agile_grasp_base::Grasp msg;
  tf::vectorEigenToMsg(hand.getGraspBottom(), msg.center);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getGraspSurface(), msg.surface_center);
  msg.width.data = hand.getGraspWidth();
  return msg;
}

rail_agile_grasp_msgs::Grasps GraspLocalizer::createRailGraspsMsg(const std::vector<GraspHypothesis>& hands)
{
  rail_agile_grasp_msgs::Grasps msg;

  msg.header.frame_id = cloud_left_->header.frame_id;
  for (int i = 0; i < hands.size(); i++)
  {
    msg.grasps.push_back(createRailGraspMsg(hands[i]));
  }

  msg.header.stamp = ros::Time::now();
  return msg;
}

rail_agile_grasp_msgs::Grasp GraspLocalizer::createRailGraspMsg(const GraspHypothesis& hand)
{
  rail_agile_grasp_msgs::Grasp msg;
  tf::vectorEigenToMsg(hand.getGraspBottom(), msg.center);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getGraspSurface(), msg.surface_center);
  msg.width.data = hand.getGraspWidth();
  return msg;
}

agile_grasp_base::Grasps GraspLocalizer::createGraspsMsgFromHands(const std::vector<Handle>& handles)
{
  agile_grasp_base::Grasps msg;
  for (int i = 0; i < handles.size(); i++)
  {
    const std::vector<GraspHypothesis>& hands = handles[i].getHandList();
    const std::vector<int>& inliers = handles[i].getInliers();

    for (int j = 0; j < inliers.size(); j++)
    {
      msg.grasps.push_back(createGraspMsg(hands[inliers[j]]));
    }
  }
  msg.header.stamp = ros::Time::now();
  std::cout << "Created grasps msg containing " << msg.grasps.size() << " hands\n";
  return msg;
}


agile_grasp_base::Grasps GraspLocalizer::createGraspsMsg(const std::vector<Handle>& handles)
{
  agile_grasp_base::Grasps msg;
  for (int i = 0; i < handles.size(); i++)
    msg.grasps.push_back(createGraspMsg(handles[i]));  
  msg.header.stamp = ros::Time::now();
  std::cout << "Created grasps msg containing " << msg.grasps.size() << " handles\n";
  return msg;
}


agile_grasp_base::Grasp GraspLocalizer::createGraspMsg(const Handle& handle)
{
  agile_grasp_base::Grasp msg;
  tf::vectorEigenToMsg(handle.getCenter(), msg.center);
  tf::vectorEigenToMsg(handle.getAxis(), msg.axis);
  tf::vectorEigenToMsg(handle.getApproach(), msg.approach);
  tf::vectorEigenToMsg(handle.getHandsCenter(), msg.surface_center);
  msg.width.data = handle.getWidth();
  return msg;
}
