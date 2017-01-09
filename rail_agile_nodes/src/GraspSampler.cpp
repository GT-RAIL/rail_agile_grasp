#include "rail_agile_nodes/GraspSampler.h"

using namespace std;

GraspSampler::GraspSampler() : pnh("~")
{
  pnh.param("neighborhood_radius", neighborhoodRadius, 0.02);
  pnh.param("orientation_threshold", orientationThreshold, 0.1);
  pnh.param("cluster_size", clusterSize, 5);
  pnh.param("remove_table", removeTable, false);

  neighborhoodRadius = pow(neighborhoodRadius, 2);
  orientationThreshold = pow(orientationThreshold, 2);
  string cloudTopic("/camera/depth_registered/points");
  pnh.getParam("cloud_topic", cloudTopic);

  cloudReceived = false;

  cloudSubscriber = n.subscribe(cloudTopic, 1, &GraspSampler::cloudCallback, this);
  graspSubscriber = n.subscribe("find_grasps/grasps_with_workspace", 1, &GraspSampler::graspsCallback, this);
  graspsPublisher = pnh.advertise<geometry_msgs::PoseArray>("sampled_grasps", 1);
  debugPublisher = pnh.advertise<geometry_msgs::PoseStamped>("debug", 1);

  changePointCloudTopicServer = pnh.advertiseService("change_point_cloud_topic", &GraspSampler::changePointCloudTopicCallback, this);
}

void GraspSampler::cloudCallback(const sensor_msgs::PointCloud2 &pc)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);

  //save cloud
  cloud = pc;

  cloudReceived = true;
}

bool GraspSampler::changePointCloudTopicCallback(rail_agile_grasp_msgs::ChangePointCloud::Request &req, rail_agile_grasp_msgs::ChangePointCloud::Response &res)
{
  boost::recursive_mutex::scoped_lock lock(cloudMutex);
  cloudSubscriber = n.subscribe(req.cloudTopic, 1, &GraspSampler::cloudCallback, this);

  cloudReceived = false;

  return true;
}

void GraspSampler::graspsCallback(const rail_agile_grasp_msgs::GraspsWithWorkspace &candidates)
{
  string frame = "camera_rgb_optical_frame";
  //string frame = "kinect2_rgb_optical_frame";

  if (!cloudReceived)
  {
    ROS_INFO("Waiting for point cloud...");
    ros::Rate loopRate(30);
    ros::Time startTime = ros::Time::now();
    while (!cloudReceived)
    {
      loopRate.sleep();
      ros::spinOnce();

      if (ros::Time::now() - startTime >= ros::Duration(10.0))
      {
        ROS_INFO("No point cloud received, could not calculate poses.");
        geometry_msgs::PoseArray emptyPoses;
        graspsPublisher.publish(emptyPoses);
        return;
      }
    }
    ROS_INFO("Point cloud received.");
  }

  {
    boost::recursive_mutex::scoped_lock lock(cloudMutex);

    //transform point cloud to the frame of the grasps
    sensor_msgs::PointCloud2 transformedCloud;
    pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    //transformedCloud.header.frame_id = frame;
    transformedCloud.header.frame_id = candidates.grasps.header.frame_id;
    pcl_ros::transformPointCloud(transformedCloud.header.frame_id, cloud, transformedCloud, tfListener);
    pcl_conversions::toPCL(transformedCloud, *tempCloud);
    pcl::fromPCLPointCloud2(*tempCloud, *convertedCloud);

    //crop point cloud to workspace
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr croppedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::CropBox<pcl::PointXYZRGB> cropBox;
    Eigen::Vector4f minPoint, maxPoint;
    if (candidates.workspace.mode == rail_agile_grasp_msgs::Workspace::CENTERED_ROI)
    {
      minPoint[0] = (float)(candidates.workspace.roiCenter.point.x - candidates.workspace.roiDimensions.x/2.0);
      minPoint[1] = (float)(candidates.workspace.roiCenter.point.y - candidates.workspace.roiDimensions.y/2.0);
      minPoint[2] = (float)(candidates.workspace.roiCenter.point.z - candidates.workspace.roiDimensions.z/2.0);
      maxPoint[0] = (float)(candidates.workspace.roiCenter.point.x + candidates.workspace.roiDimensions.x/2.0);
      maxPoint[1] = (float)(candidates.workspace.roiCenter.point.y + candidates.workspace.roiDimensions.y/2.0);
      maxPoint[2] = (float)(candidates.workspace.roiCenter.point.z + candidates.workspace.roiDimensions.z/2.0);
    }
    else
    {
      minPoint[0] = (float)(candidates.workspace.x_min);
      minPoint[1] = (float)(candidates.workspace.y_min);
      minPoint[2] = (float)(candidates.workspace.z_min);
      maxPoint[0] = (float)(candidates.workspace.x_max);
      maxPoint[1] = (float)(candidates.workspace.y_max);
      maxPoint[2] = (float)(candidates.workspace.z_max);
    }
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(convertedCloud);
    cropBox.filter(*croppedCloud);

    ROS_INFO("Received %lu grasps...", candidates.grasps.grasps.size());
    vector<geometry_msgs::Pose> graspPoses;

    //convert agile grasps to pose messages
    for (unsigned int i = 0; i < candidates.grasps.grasps.size(); i ++)
    {
      Eigen::Vector3d surfaceCenter;
      Eigen::Vector3d axis, approach, binormal;

      tf::vectorMsgToEigen(candidates.grasps.grasps[i].surface_center, surfaceCenter);
      tf::vectorMsgToEigen(candidates.grasps.grasps[i].axis, axis);
      tf::vectorMsgToEigen(candidates.grasps.grasps[i].approach, approach);
      binormal = approach.cross(axis);

      Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3,3);
      R.col(0) = approach;
      R.col(1) = axis;
      R.col(2) = binormal;

      tf::Matrix3x3 tfR;
      tf::matrixEigenToTF(R, tfR);
      tf::Quaternion q;
      tfR.getRotation(q);
      q.normalize();

      geometry_msgs::Pose graspPose;
      tf::pointEigenToMsg(surfaceCenter, graspPose.position);
      tf::quaternionTFToMsg(q, graspPose.orientation);

      graspPoses.push_back(graspPose);
    }

    ROS_INFO("Checking poses...");
    geometry_msgs::PoseArray finalPoses;
    finalPoses.header.stamp = ros::Time::now();
    //finalPoses.header.frame_id = frame;
    finalPoses.header.frame_id = candidates.grasps.header.frame_id;
    //evaluate all of the grasp poses
    random_shuffle(graspPoses.begin(), graspPoses.end());
    for (int i = 0; i < graspPoses.size(); i ++)
    {
      //find close poses
      vector<geometry_msgs::Pose> poseCluster;
      vector<unsigned int> poseClusterIndices;
      poseCluster.push_back(graspPoses[i]);
      poseClusterIndices.push_back(i);
      for (unsigned int j = 0; j < graspPoses.size(); j ++)
      {
        if (i == j)
          continue;

        if (squaredDistance(graspPoses[i].position, graspPoses[j].position) < neighborhoodRadius &&
            squaredDistance(graspPoses[i].orientation, graspPoses[j].orientation) < orientationThreshold)
        {
          poseCluster.push_back(graspPoses[j]);
          poseClusterIndices.push_back(j);
        }
      }

      //add new pose to final pose list
      if (poseCluster.size() > clusterSize)
      {
        int clusterSize = poseCluster.size();
        geometry_msgs::Pose averagePose;
        sort(poseClusterIndices.begin(), poseClusterIndices.end());
        int slerpCounter = 1;
        tf::Quaternion avgQuat, newQuat;
        for (int j = (int)poseClusterIndices.size() - 1; j >= 0; j --)
        {
          averagePose.position.x += graspPoses[poseClusterIndices[j]].position.x;
          averagePose.position.y += graspPoses[poseClusterIndices[j]].position.y;
          averagePose.position.z += graspPoses[poseClusterIndices[j]].position.z;
          if (slerpCounter == 1)
          {
            tf::quaternionMsgToTF(graspPoses[poseClusterIndices[j]].orientation, avgQuat);
          }
          else
          {
            tf::quaternionMsgToTF(graspPoses[poseClusterIndices[j]].orientation, newQuat);
            avgQuat.slerp(newQuat, 1.0/((double)slerpCounter)).normalize();
          }
          slerpCounter ++;

          graspPoses.erase(graspPoses.begin() + poseClusterIndices[j]);
          if (j <= i)
            i --;
        }
        averagePose.position.x /= (double)clusterSize;
        averagePose.position.y /= (double)clusterSize;
        averagePose.position.z /= (double)clusterSize;
        tf::quaternionTFToMsg(avgQuat, averagePose.orientation);
        finalPoses.poses.push_back(averagePose);
      }
    }

    ROS_INFO("Found %lu poses!", finalPoses.poses.size());
    if (finalPoses.poses.empty())
    {
      return;
    }

    // Pose heuristics

    pcl::SACSegmentation<pcl::PointXYZRGB> planeSegmenter;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // set the segmentaion parameters
    planeSegmenter.setOptimizeCoefficients(true);
    planeSegmenter.setModelType(pcl::SACMODEL_PLANE);
    planeSegmenter.setMethodType(pcl::SAC_RANSAC);
    planeSegmenter.setDistanceThreshold(0.005);
    planeSegmenter.setInputCloud(croppedCloud);
    planeSegmenter.segment(*inliers, *coefficients);

    // cluster within window
    vector<pcl::PointIndices> clusterIndices;
    vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    vector<pcl::search::KdTree<pcl::PointXYZRGB>::Ptr> searchTrees;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    if (removeTable)
    {
      //remove table plane
      pcl::ModelCoefficients::Ptr tableCoefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr tableInliers (new pcl::PointIndices);
      planeSegmenter.setDistanceThreshold(0.01);
      planeSegmenter.setInputCloud(convertedCloud);
      planeSegmenter.segment(*tableInliers, *tableCoefficients);

      pcl::ExtractIndices<pcl::PointXYZRGB> extractTable;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tableRemovedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      extractTable.setInputCloud(convertedCloud);
      extractTable.setIndices(tableInliers);
      extractTable.setNegative(true);
      extractTable.filter(*tableRemovedCloud);

      //re-crop
      cropBox.setInputCloud(tableRemovedCloud);
      cropBox.filter(*croppedCloud);
    }
    if (croppedCloud->size() == 0)
    {
      ROS_INFO("No points in plane-removed point cloud, cannot calculate grasps...");
      return;
    }
    kdTree->setInputCloud(croppedCloud);
    clusterer.setInputCloud(croppedCloud);
    clusterer.setClusterTolerance(0.01);
    clusterer.setMinClusterSize(20);
    clusterer.setMaxClusterSize(10000);
    clusterer.setSearchMethod(kdTree);
    clusterer.extract(clusterIndices);

    for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); it ++)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit ++)
      {
        tempCluster->points.push_back(croppedCloud->points[*pit]);
      }
      tempCluster->width = tempCluster->points.size();
      tempCluster->height = 1;
      tempCluster->is_dense = true;
      clusters.push_back(tempCluster);

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
      searchTree->setInputCloud(tempCluster);
      searchTrees.push_back(searchTree);
    }

    if (clusters.empty())
    {
      ROS_INFO("No clusters could be extracted after plane removal, cannot calculate grasps...");
      return;
    }

    Eigen::Vector3d xVector, yVector, zVector;
    xVector[0] = 1;
    xVector[1] = 0;
    xVector[2] = 0;
    yVector[0] = 0;
    yVector[1] = 1;
    yVector[2] = 0;
    zVector[0] = 0;
    zVector[1] = 0;
    zVector[2] = 1;
    vector<PoseWithHeuristic> rankedFinalPoses;
    for (unsigned int i = 0; i < finalPoses.poses.size(); i ++)
    {
      Eigen::Quaterniond poseQuaternion;
      Eigen::Matrix3d poseRotationMatrix;
      tf::quaternionMsgToEigen(finalPoses.poses[i].orientation, poseQuaternion);
      poseRotationMatrix = poseQuaternion.toRotationMatrix();
      Eigen::Vector3d testXVector = poseRotationMatrix * xVector;

      //heuristic 1: perpendicular to plane
      double h1 = sqrt(pow(testXVector[0] - coefficients->values[0], 2) + pow(testXVector[1] - coefficients->values[1], 2) + pow(testXVector[2] - coefficients->values[2], 2))/2.0;

      //heuristic 2: alignment with principal component analysis
      double minClusterDst = numeric_limits<double>::max();
      int minClusterIndex = 0;
      for (unsigned int j = 0; j < clusters.size(); j ++)
      {
        vector<int> kIndices;
        vector<float> kSqrDistances;
        pcl::PointXYZRGB testPoint;
        testPoint.x = finalPoses.poses[i].position.x;
        testPoint.y = finalPoses.poses[i].position.y;
        testPoint.z = finalPoses.poses[i].position.z;
        kIndices.resize(1);
        kSqrDistances.resize(1);
        searchTrees[j]->nearestKSearch(testPoint, 1, kIndices, kSqrDistances);
        if (kSqrDistances[0] < minClusterDst)
        {
          minClusterDst = kSqrDistances[0];
          minClusterIndex = j;
        }
      }

      Eigen::Quaterniond principalDirection = computePrincipalDirection(clusters[minClusterIndex], frame);

      Eigen::Vector3d testYVector = poseRotationMatrix*yVector;
      Eigen::Vector3d testZVector = poseRotationMatrix*zVector;
      Eigen::Vector3d principalYVector = principalDirection*yVector;
      Eigen::Vector3d principalZVector = principalDirection*zVector;
      double yyDist = min(squaredDistance(testYVector, principalYVector), squaredDistance(testYVector, -1*principalYVector));
      double yzDist = min(squaredDistance(testYVector, principalZVector), squaredDistance(testYVector, -1*principalZVector));
      double zyDist = min(squaredDistance(testZVector, principalYVector), squaredDistance(testZVector, -1*principalYVector));
      double zzDist = min(squaredDistance(testZVector, principalZVector), squaredDistance(testZVector, -1*principalZVector));
      double h2 = sqrt(min(min(min(yyDist, yzDist), zyDist), zzDist))/2.0;

      //heuristic 3: distance from clicked point
      double h3 = sqrt(squaredDistance(finalPoses.poses[i].position, candidates.workspace.roiCenter.point)) /
                  sqrt(pow(candidates.workspace.roiDimensions.x/2.0, 2) + pow(candidates.workspace.roiDimensions.y/2.0, 2) + pow(candidates.workspace.roiDimensions.z/2.0, 2));

      double h = 0.6*h1 + 0.25*h2 + 0.15*h3;
      PoseWithHeuristic rankedPose(finalPoses.poses[i], h);
      rankedFinalPoses.push_back(rankedPose);
    }

    sort(rankedFinalPoses.begin(), rankedFinalPoses.end());
    finalPoses.poses.clear();
    for (unsigned int i = 0; i < rankedFinalPoses.size(); i ++)
    {
      finalPoses.poses.push_back(rankedFinalPoses[i].pose);
    }

    geometry_msgs::PoseStamped debugPose;
    debugPose.header.frame_id = finalPoses.header.frame_id;
    debugPose.pose = rankedFinalPoses[0].pose;
    debugPublisher.publish(debugPose);

    graspsPublisher.publish(finalPoses);
  }
}

Eigen::Quaterniond GraspSampler::computePrincipalDirection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string frame)
{
  // compute principal direction
  Eigen::Matrix3d covariance;
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3d eig_dx = eigen_solver.eigenvectors();
  eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

  //final transform
  const Eigen::Quaterniond qfinal(eig_dx);

  //debug publishing
  geometry_msgs::PoseStamped debugPose;
  debugPose.header.frame_id = frame;
  debugPose.pose.position.x = centroid[0];
  debugPose.pose.position.y = centroid[1];
  debugPose.pose.position.z = centroid[2];
  tf::quaternionEigenToMsg(qfinal, debugPose.pose.orientation);
  //debugPublisher.publish(debugPose);

  return qfinal;
}

double GraspSampler::squaredDistance(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2)
{
  return min(pow(q1.x - q2.x, 2) + pow(q1.y - q2.y, 2) + pow(q1.z - q2.z, 2) + pow(q1.w - q2.w, 2),
    pow(q1.x + q2.x, 2) + pow(q1.y + q2.y, 2) + pow(q1.z + q2.z, 2) + pow(q1.w + q2.w, 2));
}

double GraspSampler::squaredDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2);
}

double GraspSampler::squaredDistance(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
  return (v1 - v2).squaredNorm();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_sampler");
  GraspSampler gs;

  ros::spin();

  return EXIT_SUCCESS;
}