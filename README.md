# rail_agile_grasp
Metapackage for the heuristic-based AGILE grasp detector and classifier. Includes the original AGILE package, the heuristic classifier package, and other supporting functionality.

Additional documentation and details to come...

## Description
This metapackage extends AGILE grasp to use heuristic-based grasp evaluation in place of AGILE's grasp classifier.  Development of this package included adding extra ROS messages, services, and actionlib to the original AGILE code to facilitate communication with the heurist-based evaluator.  Also included are additional packages potentially useful for testing and deployment.

## Menu
 * [agile_grasp_base](#agile_grasp_base)
 * [rail_agile_grasp_msgs](#rail_agile_grasp_msgs)
 * [rail_agile_nodes](#rail_agile_nodes)
  * [hlpr_moveit_wrapper](#hlpr_moveit_wrapper)
  * [common_actions](#common_actions)
  * [primitive_actions](#primitive_actions)
 * [Installation](#installation)
 * [Startup](#startup)
 

## agile_grasp_base
This is the original agile grasp package, modified with an action server to allow grasps to be found on demand.  More details on the functionality of this package can be found in the main [AGILE_grasp](https://github.com/atenpas/agile_grasp), on which this was based.

## rail_agile_grasp_msgs
This package contains action definitions, messages, and services used throughout the metapackage.  For more details, check the message definitions themselves.

## rail_agile_nodes
The primary node of this package, [grasp_sampler](#grasp_sampler), implements grasp clustering and rating based on a set of heuristics designed to select effective grasps for man-made objects.  More details on the heuristics can be found in our paper, A Comparison of Remote Robot Teleoperation Interfaces for General Object Manipulation, to be published in HRI2017.

The package also includes optional supporting nodes for filtering point clouds, creating clickable point clouds via interactive marker server, and interleaving point clouds from multiple sources.  Details of each node can be found below.

### grasp_sampler
This node performs grasp candidate evaluation by evaluating a set of heuristics on grasp poses sampled from the grasp hypotheses determined by the AGILE_grasp base package.  Relevant parameters, action servers, topics, and services are as follows:
 * **Subscribed Topics**
  * `/camera/depth_registered/points`([sensor_msgs/PointCloud2](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/PointCloud2.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; point cloud subscriber; the topic can be set by changing the parameter `cloud_topic` or by calling the `change_point_cloud_topic` service.
  * `/find_grasps/grasps_with_workspace`([rail_agile_grasp_msgs/GraspsWithWorkspace](https://github.com/GT-RAIL/rail_agile_grasp/blob/master/rail_agile_grasp_msgs/msg/GraspsWithWorkspace.msg))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; the set of grasp hypothese calculated by AGILE grasp, to be sampled from and evaluated by this node
 * **Published Topics**
  * `sampled_grasps`([geometry_msgs/PoseArray](http://docs.ros.org/indigo/api/geometry_msgs/html/msg/PoseArray.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; the final set of grasps, ordered from best to worst
 * **Services**
  * `change_point_cloud_topic`([rail_agile_grasp_msgs/ChangePointCloud](https://github.com/GT-RAIL/rail_agile_grasp/blob/master/rail_agile_grasp_msgs/srv/ChangePointCloud.srv))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; service for switching the input point cloud
 * **Parameters**
  * `cloud_topic`(string, "/camera/depth_registered/points")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; initial point cloud topic to subscribe to
  * `neighborhood_radius`(double, 0.02)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; radius defining the sphere of a grasp's neighborhood, in meters, used for clustering grasp hypotheses
  * `orientation_threshold`(double, 0.1)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; maximum difference, in radians, between two grasps' orientations to be clustered
  * `cluster_size`(int, 5)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; maximum number of grasp hypotheses that can compose one cluster
  * `remove_table`(boolean, false)  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; option to remove the dominant plane from the point cloud within the given workspace before calculating the object orientaion heuristic; it's recommended to set this to true if the grasps are being used for tabletop pick-and-place applications.

### point_cloud_clicker
This node facilitates clicking on point clouds by either creating an interactive marker server with a clickable point cloud, or by running action servers that take clicked point information and return points in 3D space.  Relevant parameters, action servers, topics, and services are as follows:
 * **Action Servers**
  * `click_image_point`([rail_agile_grasp_msgs/ClickImagePointAction](https://github.com/GT-RAIL/rail_agile_grasp/blob/master/rail_agile_grasp_msgs/action/ClickImagePoint.action))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; click event from clicking on an image; calculates gasps using AGILE and the `grasp_sampler` node on a workspace centered on the clicked point
 * `click_image_point_navidget`([rail_agile_grasp_msgs/ClickImagePointAction](https://github.com/GT-RAIL/rail_agile_grasp/blob/master/rail_agile_grasp_msgs/action/ClickImagePoint.action))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; click event from clicking on an image; initializes a constrained positioning marker at the clicked point, see the [remote_manipulation_markers](http://wiki.ros.org/remote_manipulation_markers) package for details
 * **Subscribed Topics**
  * `/camera/depth_registered/points`([sensor_msgs/PointCloud2](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/PointCloud2.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; point cloud subscriber; the topic can be set by changing the parameter `cloud_topic` or by calling the `change_point_cloud_topic` service.
 * **Services**
  * `change_point_cloud_topic`([rail_agile_grasp_msgs/ChangePointCloud](https://github.com/GT-RAIL/rail_agile_grasp/blob/master/rail_agile_grasp_msgs/srv/ChangePointCloud.srv))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; service for switching the input point cloud
  * `request_marker_update'([std_srvs/Empty](http://docs.ros.org/indigo/api/std_srvs/html/srv/Empty.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; update the clickable point cloud marker on request
 * **Parameters**
  * `cloud_topic`(string, "/camera/depth_registered/points")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; initial point cloud topic to subscribe to

### point_cloud_filter
This node performs voxel grid filtering on a given input point cloud.  Relevant parameters, action servers, topics, and services are as follows:
 * **Subscribed Topics**
  * `/camera/depth_registered/points`([sensor_msgs/PointCloud2](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/PointCloud2.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; point cloud subscriber; the topic can be set by changing the parameter `cloud_topic`
 * **Published Topics**
  * `filtered_cloud`([sensor_msgs/PointCloud2](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/PointCloud2.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; voxel grid filtered point cloud
 * **Parameters**
  * `cloud_topic`(string, "/camera/depth_registered/points")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; initial point cloud topic to subscribe to

### point_cloud_interleaver
This node subscribes to two point cloud sources and combines them into a single point cloud with a common frame of reference.  Relevant parameters, action servers, topics, and services are as follows:
 * **Subscribed Topics**
  * `/camera1/depth_registered/points`([sensor_msgs/PointCloud2](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/PointCloud2.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; point cloud subscriber; the topic can be set by changing the parameter `cloud1_topic`
  * `/camera2/depth_registered/points`([sensor_msgs/PointCloud2](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/PointCloud2.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; point cloud subscriber; the topic can be set by changing the parameter `cloud2_topic`
 * **Published Topics**
  * `interleaved_clouds`([sensor_msgs/PointCloud2](http://docs.ros.org/indigo/api/sensor_msgs/html/msg/PointCloud2.html))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; combined point cloud
 * **Parameters**
  * `cloud1_topic`(string, "/camera1/depth_registered/points")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; initial point cloud topic to subscribe to
  * `cloud2_topic`(string, "/camera2/depth_registered/points")  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; initial point cloud topic to subscribe to

## Installation
in progress...

## Startup
in progress...
