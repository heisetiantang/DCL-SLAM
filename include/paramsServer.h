#ifndef _PARAM_SERVER_H_
#define _PARAM_SERVER_H_

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
// dcl_slam define
#include "dcl_slam/neighbor_estimate.h"
// pcl
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
// mapping
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;
using namespace std;

typedef pcl::PointXYZI PointPose3D;
struct PointPose6D
{
  float x;
  float y;
  float z;
  float intensity;
  float roll;
  float pitch;
  float yaw;
  double time;
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointPose6D,
								  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
									  float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

struct singleRobot
{
  /*** robot information ***/
  int id_;					// robot id
  std::string name_;		// robot name, for example, 'a', 'b', etc.
  std::string odom_frame_;	// odom frame

  /*** ros subscriber and publisher ***/
  // mapping
  ros::Subscriber sub_optimization_state, sub_pose_estimate_state, sub_rotation_estimate_state;
  ros::Subscriber sub_neighbor_rotation_estimates, sub_neighbor_pose_estimates;
  ros::Publisher pub_optimization_state, pub_pose_estimate_state, pub_rotation_estimate_state;
  ros::Publisher pub_neighbor_rotation_estimates, pub_neighbor_pose_estimates;
  // loop closure
  ros::Subscriber sub_loop_info;
  ros::Publisher pub_loop_info;
  // descriptors
  ros::Subscriber sub_descriptors;
  ros::Publisher pub_descriptors;

  /*** other ***/
  ros::Time time_cloud_input_stamp;								   // recent keyframe timestamp
  double time_cloud_input;										   // and its double type
  dcl_slam::neighbor_estimate estimate_msg;						   // pose and rotation estimate msg
  pcl::PointCloud<PointPose3D>::Ptr keyframe_cloud;				   // recent keyframe pointcloud
  std::vector<pcl::PointCloud<PointPose3D>> keyframe_cloud_array;  // and its array
  // 增加用于颜色的点云****************************************************
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keyframe_cloud_rgb;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> keyframe_cloud_rgb_array;
  ros::Time time_cloud_input_stamp_rgb;
  double time_cloud_input_rgb;
  // 增加pose的vector
  std::vector<gtsam::Pose3> pose_buffer;

  Pose3 piror_odom;	 // piror factor
};

enum class LiDARType
{
  VELODYNE,
  LIVOX
};
enum class DescriptorType
{
  ScanContext,
  LidarIris,
  M2DP
};
enum class OptimizerState
{
  Idle,
  Start,
  Initialization,
  RotationEstimation,
  PoseEstimationInitialization,
  PoseEstimation,
  End,
  PostEndingCommunicationDelay
};

/*** class paramsServer ***/
class paramsServer
{
public:
  paramsServer();

  Eigen::Affine3f gtsamPoseToAffine3f(gtsam::Pose3 pose);

  geometry_msgs::Transform gtsamPoseToTransform(gtsam::Pose3 pose);

  gtsam::Pose3 transformToGtsamPose(const geometry_msgs::Transform& pose);

  gtsam::Pose3 pclPointTogtsamPose3(PointPose6D point);

  pcl::PointCloud<PointPose3D>::Ptr transformPointCloud(pcl::PointCloud<PointPose3D> cloud_in, PointPose6D* pose);

  pcl::PointCloud<PointPose3D>::Ptr transformPointCloud(pcl::PointCloud<PointPose3D> cloud_in, gtsam::Pose3 pose);

  pcl::PointCloud<PointPose3D>::Ptr transformPointCloud(pcl::PointCloud<PointPose3D> cloud_in, PointPose6D* pose,
														gtsam::Pose3* pose_gtsam);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud_in,
															 PointPose6D* pose,gtsam::Pose3* pose_gtsam);

protected:
  ros::NodeHandle nh;

  // robot team
  int number_of_robots_;  // number of robots in robot team

  // robot info
  std::string name_;  // this robot name
  int id_;			  // this robot id

  // frames name
  std::string world_frame_;	 // global frame
  std::string odom_frame_;	 // local frame

  // lidar Sensor Configuration
  LiDARType sensor_;  // lidar type, support 'velodyne 16/64' or 'livox 6'
  int n_scan_;		  // number of lidar channel (i.e., 6, 16, 64)

  // CPU params
  int onboard_cpu_cores_num_;			 // cores number of onboard unit
  float loop_closure_process_interval_;	 // interval of detecting loop (in second)
  float map_publish_interval_;			 // interval of publish global maps (in second)
  float mapping_process_interval_;		 // interval of optmization (in second)

  // Mapping
  bool global_optmization_enable_;			  // enable distributed DGS
  bool use_pcm_;							  // enable pairwise consistency maximization (PCM)
  float pcm_threshold_;						  // confidence probability for PCM (i.e., 0.01, 0.05, 0.1, 0.25, 0.5, 0.75)
  int optmization_maximum_iteration_;		  // maximum iterations time of optimization
  bool use_between_noise_;					  // use between noise flag
  int fail_safe_steps_;						  // steps of fail safe to abort (depend on both fail_safe_wait_time_ and
											  // mapping_process_interval_)
  float fail_safe_wait_time_;				  // wait time for fail safe (in second)
  float rotation_estimate_change_threshold_;  // difference between rotation estimate provides an early stopping
											  // condition
  float pose_estimate_change_threshold_;	  // difference between pose estimate provides an early stopping condition
  float gamma_;								  // gamma value for over relaxation methods
  bool use_flagged_init_;					  // to use flagged initialization or not
  bool use_landmarks_;	 // use landmarks -- landmarks are given symbols as upper case of robot name
  bool use_heuristics_;	 // use heuristics-based algorithm for the max-clique solver

  // keyframe
  float keyframe_distance_threshold_;  // keyframe distance threshold (in meter)
  float keyframe_angle_threshold_;	   // keyframe angle threshold (in rad)

  // downsample
  float map_leaf_size_;		  // scan to map matching downsample rate (default 0.4)
  float descript_leaf_size_;  // descriptor downsample rate (default 0.1)

  // loop closure
  bool intra_robot_loop_closure_enable_;   // enable to search intra-robot loop closre with global descriptor
  bool inter_robot_loop_closure_enable_;   // enable to search intra-robot loop closre with global descriptor
  DescriptorType descriptor_type_num_;	   // descriptor type: ScanContext, LidarIris, M2DP
  int knn_candidates_;					   // k nearest neighbor search of row key
  int exclude_recent_frame_num_;		   // exclude recent keyframe in intra-robot loop closure
  float search_radius_;					   // radius of radius search based intra-robot loop closure
  int match_mode_;						   // iris-feature matching mode, (i.e., 0, 1, 2; default 2)
  int iris_row_;						   // iris-image row
  int iris_column_;						   // iris-image column
  float descriptor_distance_threshold_;	   // iris-feature match threshold
  int ransac_maximum_iteration_;		   // RANSAC maximum iteration time
  float ransac_threshold_;				   // RANSAC threshold (rate: [0 1])
  float ransac_outlier_reject_threshold_;  // RANSAC outlier rejection distancce
  int history_keyframe_search_num_;		   // number of history frames in submap for scan-to-map matching
  float fitness_score_threshold_;		   // ICP fitness score threshold

  // visualization
  float global_map_visualization_radius_;  // radius of radius search based intra-robot loop closure

  // Save pcd
  std::string save_directory_;
};

#endif