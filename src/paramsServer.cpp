#include "paramsServer.h"

paramsServer::paramsServer()
{
  // robot info
  std::string ns = nh.getNamespace();  // namespace of robot
  if (ns.length() != 2)
  {
    ROS_ERROR("Invalid robot prefix (should be either 'a-z' or 'A-Z'): %s", ns.c_str());
    ros::shutdown();
  }
  name_ = ns.substr(1, 1);  // romove '/' character
  id_ = name_[0] - 'a';     // id of robot

  nh.param<int>("/number_of_robots", number_of_robots_, 1);
  if (number_of_robots_ < 1)
  {
    ROS_ERROR("Invalid robot number (must be positive number): %d", number_of_robots_);
    ros::shutdown();
  }

  ns += "/dcl_slam";
  // frames name
  nh.param<std::string>(ns + "/world_frame", world_frame_, "world");
  nh.param<std::string>(ns + "/odom_frame", odom_frame_, "map");

  // lidar configuration
  std::string sensorStr;
  nh.param<std::string>(ns + "/sensor", sensorStr, "velodyne");
  if (sensorStr == "velodyne")
  {
    sensor_ = LiDARType::VELODYNE;
  }
  else if (sensorStr == "livox")
  {
    sensor_ = LiDARType::LIVOX;
  }
  else
  {
    ROS_ERROR("Invalid sensor type (must be either 'velodyne' or 'ouster'): %s ", sensorStr.c_str());
    ros::shutdown();
  }
  nh.param<int>(ns + "/n_scan", n_scan_, 16);

  // CPU Params
  nh.param<int>(ns + "/onboard_cpu_cores_num", onboard_cpu_cores_num_, 8);
  nh.param<float>(ns + "/loop_closure_process_interval", loop_closure_process_interval_, 0.02);
  nh.param<float>(ns + "/map_publish_interval", map_publish_interval_, 10.0);
  nh.param<float>(ns + "/mapping_process_interval", mapping_process_interval_, 0.1);

  // mapping
  nh.param<bool>(ns + "/global_optmization_enable", global_optmization_enable_, false);
  nh.param<bool>(ns + "/use_pcm", use_pcm_, false);
  nh.param<float>(ns + "/pcm_threshold", pcm_threshold_, 0.75);
  nh.param<bool>(ns + "/use_between_noise", use_between_noise_, false);
  nh.param<int>(ns + "/optmization_maximum_iteration", optmization_maximum_iteration_, 100);
  nh.param<float>(ns + "/failsafe_wait_time", fail_safe_wait_time_, 1.0);
  fail_safe_steps_ = fail_safe_wait_time_ / mapping_process_interval_;
  nh.param<float>(ns + "/rotation_estimate_change_threshold", rotation_estimate_change_threshold_, 0.1);
  nh.param<float>(ns + "/pose_estimate_change_threshold", pose_estimate_change_threshold_, 0.1);
  nh.param<float>(ns + "/gamma", gamma_, 1.0);
  nh.param<bool>(ns + "/use_flagged_init", use_flagged_init_, true);
  nh.param<bool>(ns + "/use_landmarks", use_landmarks_, false);
  nh.param<bool>(ns + "/use_heuristics", use_heuristics_, true);

  // downsample
  nh.param<float>(ns + "/map_leaf_size", map_leaf_size_, 0.4);
  nh.param<float>(ns + "/descript_leaf_size", descript_leaf_size_, 0.1);

  // loop closure
  nh.param<bool>(ns + "/intra_robot_loop_closure_enable", intra_robot_loop_closure_enable_, true);
  nh.param<bool>(ns + "/inter_robot_loop_closure_enable", inter_robot_loop_closure_enable_, true);
  std::string descriptor_type_;
  nh.param<std::string>(ns + "/descriptor_type", descriptor_type_, "");
  if (descriptor_type_ == "ScanContext")
  {
    descriptor_type_num_ = DescriptorType::ScanContext;
  }
  else if (descriptor_type_ == "LidarIris")
  {
    descriptor_type_num_ = DescriptorType::LidarIris;
  }
  else if (descriptor_type_ == "M2DP")
  {
    descriptor_type_num_ = DescriptorType::M2DP;
  }
  else
  {
    inter_robot_loop_closure_enable_ = false;
    ROS_WARN("Invalid descriptor type: %s, turn off interloop...", descriptor_type_.c_str());
  }
  nh.param<int>(ns + "/knn_candidates", knn_candidates_, 10);
  nh.param<int>(ns + "/exclude_recent_frame_num", exclude_recent_frame_num_, 30);
  nh.param<float>(ns + "/search_radius", search_radius_, 15.0);
  nh.param<int>(ns + "/match_mode", match_mode_, 2);
  nh.param<int>(ns + "/iris_row", iris_row_, 80);
  nh.param<int>(ns + "/iris_column", iris_column_, 360);
  nh.param<float>(ns + "/descriptor_distance_threshold", descriptor_distance_threshold_, 0.4);
  nh.param<int>(ns + "/history_keyframe_search_num", history_keyframe_search_num_, 16);
  nh.param<float>(ns + "/fitness_score_threshold", fitness_score_threshold_, 0.2);
  nh.param<int>(ns + "/ransac_maximum_iteration", ransac_maximum_iteration_, 1000);
  nh.param<float>(ns + "/ransac_threshold", ransac_threshold_, 0.5);
  nh.param<float>(ns + "/ransac_outlier_reject_threshold", ransac_outlier_reject_threshold_, 0.05);

  // keyframe params
  nh.param<float>(ns + "/keyframe_distance_threshold", keyframe_distance_threshold_, 1.0);
  nh.param<float>(ns + "/keyframe_angle_threshold", keyframe_angle_threshold_, 0.2);

  // visualization
  nh.param<float>(ns + "/global_map_visualization_radius", global_map_visualization_radius_, 200.0);

  // output directory
  nh.param<std::string>(ns + "/save_directory", save_directory_, "/dcl_output");
}

Eigen::Affine3f paramsServer::gtsamPoseToAffine3f(gtsam::Pose3 pose)
{
  return pcl::getTransformation(pose.translation().x(), pose.translation().y(), pose.translation().z(),
                                pose.rotation().roll(), pose.rotation().pitch(), pose.rotation().yaw());
}

geometry_msgs::Transform paramsServer::gtsamPoseToTransform(gtsam::Pose3 pose)
{
  geometry_msgs::Transform transform_msg;
  transform_msg.translation.x = pose.translation().x();
  transform_msg.translation.y = pose.translation().y();
  transform_msg.translation.z = pose.translation().z();
  transform_msg.rotation.w = pose.rotation().toQuaternion().w();
  transform_msg.rotation.x = pose.rotation().toQuaternion().x();
  transform_msg.rotation.y = pose.rotation().toQuaternion().y();
  transform_msg.rotation.z = pose.rotation().toQuaternion().z();

  return transform_msg;
}

gtsam::Pose3 paramsServer::transformToGtsamPose(const geometry_msgs::Transform& pose)
{
  return gtsam::Pose3(gtsam::Rot3::Quaternion(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z),
                      gtsam::Point3(pose.translation.x, pose.translation.y, pose.translation.z));
}

gtsam::Pose3 paramsServer::pclPointTogtsamPose3(PointPose6D point)
{
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(point.roll), double(point.pitch), double(point.yaw)),
                      gtsam::Point3(double(point.x), double(point.y), double(point.z)));
}

pcl::PointCloud<PointPose3D>::Ptr paramsServer::transformPointCloud(pcl::PointCloud<PointPose3D> cloud_in,
                                                                    PointPose6D* pose)
{
  pcl::PointCloud<PointPose3D>::Ptr cloud_out(new pcl::PointCloud<PointPose3D>());

  int cloud_size = cloud_in.size();
  cloud_out->resize(cloud_size);

  Eigen::Affine3f trans_cur = pcl::getTransformation(pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);

#pragma omp parallel for num_threads(onboard_cpu_cores_num_)
  for (int i = 0; i < cloud_size; ++i)
  {
    const auto& p_from = cloud_in.points[i];
    cloud_out->points[i].x =
        trans_cur(0, 0) * p_from.x + trans_cur(0, 1) * p_from.y + trans_cur(0, 2) * p_from.z + trans_cur(0, 3);
    cloud_out->points[i].y =
        trans_cur(1, 0) * p_from.x + trans_cur(1, 1) * p_from.y + trans_cur(1, 2) * p_from.z + trans_cur(1, 3);
    cloud_out->points[i].z =
        trans_cur(2, 0) * p_from.x + trans_cur(2, 1) * p_from.y + trans_cur(2, 2) * p_from.z + trans_cur(2, 3);
    cloud_out->points[i].intensity = p_from.intensity;
  }
  return cloud_out;
}

// 增加颜色点云的变换函数
pcl::PointCloud<pcl::PointXYZRGB>::Ptr paramsServer::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud_in,
                                                                         PointPose6D* pose,gtsam::Pose3* pose_gtsam)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());

  int cloud_size = cloud_in.size();
  cloud_out->resize(cloud_size);

// 使用初始位姿将其变换回去
  gtsam::Point3 translation = pose_gtsam->translation();
  gtsam::Rot3 rotation = pose_gtsam->rotation();

  Eigen::Vector3f translation_eigen(translation.x(), translation.y(), translation.z());

  Eigen::Quaternionf rotation_eigen(rotation.toQuaternion().w(), rotation.toQuaternion().x(),
                                    rotation.toQuaternion().y(), rotation.toQuaternion().z());

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translate(translation_eigen);
  transform.rotate(rotation_eigen);
  Eigen::Affine3f transform_inv = transform.inverse();

  Eigen::Affine3f trans_cur = pcl::getTransformation(pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
  trans_cur = transform_inv * trans_cur;

#pragma omp parallel for num_threads(onboard_cpu_cores_num_)
  for (int i = 0; i < cloud_size; ++i)
  {
    const auto& p_from = cloud_in.points[i];
    cloud_out->points[i].x =
        trans_cur(0, 0) * p_from.x + trans_cur(0, 1) * p_from.y + trans_cur(0, 2) * p_from.z + trans_cur(0, 3);
    cloud_out->points[i].y =
        trans_cur(1, 0) * p_from.x + trans_cur(1, 1) * p_from.y + trans_cur(1, 2) * p_from.z + trans_cur(1, 3);
    cloud_out->points[i].z =
        trans_cur(2, 0) * p_from.x + trans_cur(2, 1) * p_from.y + trans_cur(2, 2) * p_from.z + trans_cur(2, 3);
    cloud_out->points[i].r = cloud_in.points[i].r;
    cloud_out->points[i].g = cloud_in.points[i].g;
    cloud_out->points[i].b = cloud_in.points[i].b;
  }
  return cloud_out;
}

pcl::PointCloud<PointPose3D>::Ptr paramsServer::transformPointCloud(pcl::PointCloud<PointPose3D> cloud_in,
                                                                    gtsam::Pose3 pose)
{
  pcl::PointCloud<PointPose3D>::Ptr cloud_out(new pcl::PointCloud<PointPose3D>());

  int cloud_size = cloud_in.size();
  cloud_out->resize(cloud_size);

  Eigen::Affine3f trans_cur =
      pcl::getTransformation(pose.translation().x(), pose.translation().y(), pose.translation().z(),
                             pose.rotation().roll(), pose.rotation().pitch(), pose.rotation().yaw());

#pragma omp parallel for num_threads(onboard_cpu_cores_num_)
  for (int i = 0; i < cloud_size; ++i)
  {
    const auto& p_from = cloud_in.points[i];
    cloud_out->points[i].x =
        trans_cur(0, 0) * p_from.x + trans_cur(0, 1) * p_from.y + trans_cur(0, 2) * p_from.z + trans_cur(0, 3);
    cloud_out->points[i].y =
        trans_cur(1, 0) * p_from.x + trans_cur(1, 1) * p_from.y + trans_cur(1, 2) * p_from.z + trans_cur(1, 3);
    cloud_out->points[i].z =
        trans_cur(2, 0) * p_from.x + trans_cur(2, 1) * p_from.y + trans_cur(2, 2) * p_from.z + trans_cur(2, 3);
    cloud_out->points[i].intensity = p_from.intensity;
  }
  return cloud_out;
}

pcl::PointCloud<PointPose3D>::Ptr paramsServer::transformPointCloud(pcl::PointCloud<PointPose3D> cloud_in,
                                                                    PointPose6D* pose, gtsam::Pose3* pose_gtsam)
{
  pcl::PointCloud<PointPose3D>::Ptr cloud_out(new pcl::PointCloud<PointPose3D>());

  int cloud_size = cloud_in.size();
  cloud_out->resize(cloud_size);
  // 使用初始位姿将其变换回去
  gtsam::Point3 translation = pose_gtsam->translation();
  gtsam::Rot3 rotation = pose_gtsam->rotation();

  Eigen::Vector3f translation_eigen(translation.x(), translation.y(), translation.z());

  Eigen::Quaternionf rotation_eigen(rotation.toQuaternion().w(), rotation.toQuaternion().x(),
                                    rotation.toQuaternion().y(), rotation.toQuaternion().z());

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translate(translation_eigen);
  transform.rotate(rotation_eigen);
  Eigen::Affine3f transform_inv = transform.inverse();
  Eigen::Affine3f trans_cur = pcl::getTransformation(pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
  trans_cur = transform_inv * trans_cur;

#pragma omp parallel for num_threads(onboard_cpu_cores_num_)
  for (int i = 0; i < cloud_size; ++i)
  {
    const auto& p_from = cloud_in.points[i];
    cloud_out->points[i].x =
        trans_cur(0, 0) * p_from.x + trans_cur(0, 1) * p_from.y + trans_cur(0, 2) * p_from.z + trans_cur(0, 3);
    cloud_out->points[i].y =
        trans_cur(1, 0) * p_from.x + trans_cur(1, 1) * p_from.y + trans_cur(1, 2) * p_from.z + trans_cur(1, 3);
    cloud_out->points[i].z =
        trans_cur(2, 0) * p_from.x + trans_cur(2, 1) * p_from.y + trans_cur(2, 2) * p_from.z + trans_cur(2, 3);
    cloud_out->points[i].intensity = p_from.intensity;
  }
  return cloud_out;
}
