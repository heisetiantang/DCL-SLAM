#include "distributedMapping.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	class distributedMapping: publish visualization msg
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::globalMapThread()
{
  ros::Rate rate(1.0 / map_publish_interval_);	// update global map per 4s (default)

  while (ros::ok())
  {
	rate.sleep();

	publishGlobalMap();	 // global map visualization

	publishLoopClosureConstraint();	 // loop closure visualization
  }
}

void distributedMapping::publishGlobalMap()
{
  // early return
  // if (pub_global_map.getNumSubscribers() == 0 || initial_values->empty() == true )
  // {
  // 	if(pubGlobal_all.getNumSubscribers() == 0) return;

  // }
  if (initial_values->empty() == true)
  {
	return;
  }
  // copy the poses and change to cloud type
  Values poses_initial_guess_copy = *initial_values;
  pcl::PointCloud<PointPose3D>::Ptr poses_3d_cloud_copy(new pcl::PointCloud<PointPose3D>());
  pcl::PointCloud<PointPose6D>::Ptr poses_6d_cloud_copy(new pcl::PointCloud<PointPose6D>());
  for (const Values::ConstKeyValuePair& key_value : poses_initial_guess_copy)
  {
	Symbol key = key_value.key;
	Pose3 pose = poses_initial_guess_copy.at<Pose3>(key);

	PointPose3D pose_3d;
	pose_3d.x = pose.translation().x();
	pose_3d.y = pose.translation().y();
	pose_3d.z = pose.translation().z();
	pose_3d.intensity = key.index();

	PointPose6D pose_6d;
	pose_6d.x = pose_3d.x;
	pose_6d.y = pose_3d.y;
	pose_6d.z = pose_3d.z;
	pose_6d.intensity = pose_3d.intensity;
	pose_6d.roll = pose.rotation().roll();
	pose_6d.pitch = pose.rotation().pitch();
	pose_6d.yaw = pose.rotation().yaw();

	poses_3d_cloud_copy->push_back(pose_3d);
	poses_6d_cloud_copy->push_back(pose_6d);
  }

  // find the closest history key frame
  std::vector<int> indices;
  std::vector<float> distances;
  kdtree_history_keyposes->setInputCloud(poses_3d_cloud_copy);
  kdtree_history_keyposes->radiusSearch(poses_3d_cloud_copy->back(), global_map_visualization_radius_, indices,
										distances, 0);

  // extract visualized key frames
  pcl::PointCloud<PointPose3D>::Ptr global_map_keyframes(new pcl::PointCloud<PointPose3D>());
  pcl::PointCloud<PointPose3D>::Ptr global_map_keyframes_ds(new pcl::PointCloud<PointPose3D>());
  // 将点云按照indices的索引进行拼接。同时将之前得到的带颜色
  for (int i = 0; i < (int)indices.size(); ++i)
  {
	PointPose6D pose_6d_tmp = poses_6d_cloud_copy->points[indices[i]];
	*global_map_keyframes += *transformPointCloud(robots[id_].keyframe_cloud_array[pose_6d_tmp.intensity],
												  &pose_6d_tmp, &robots[id_].pose_buffer[pose_6d_tmp.intensity]);
	// *global_map_keyframes += *transformPointCloud(robots[id_].keyframe_cloud_array[pose_6d_tmp.intensity],
	// 											  &pose_6d_tmp);
	// *global_map_keyframes +=
	// 	robots[id_].keyframe_cloud_array[pose_6d_tmp.intensity];
  }

  // if (!robots[id_].keyframe_cloud_rgb_array.empty())
  if (0)  //暂时没有数据输出
  {
	// cout << "robots[id_].keyframe_cloud_rgb_array is not empty" << endl;
	// 仿照、将带颜色的点云拼接
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_keyframes_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_keyframes_rgb_ds(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (size_t i = 0; i < (int)indices.size(); ++i)
	{
	  PointPose6D pose_6d_tmp = poses_6d_cloud_copy->points[indices[i]];
	  *global_map_keyframes_rgb +=
		  *transformPointCloud(robots[id_].keyframe_cloud_rgb_array[pose_6d_tmp.intensity], &pose_6d_tmp,
                &robots[id_].pose_buffer[pose_6d_tmp.intensity]);
	}

	// pcl::VoxelGrid<pcl::PointXYZRGB> downsample_filter_for_global_map_rgb;	// for global map visualization
	// downsample_filter_for_global_map_rgb.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
	// downsample_filter_for_global_map_rgb.setInputCloud(global_map_keyframes_rgb);
	// downsample_filter_for_global_map_rgb.filter(*global_map_keyframes_rgb_ds);

	// 发布没有降采样的RGB全局地图
	sensor_msgs::PointCloud2 global_map_msg_rgb;
	pcl::toROSMsg(*global_map_keyframes_rgb, global_map_msg_rgb);
	global_map_msg_rgb.header.stamp = robots[id_].time_cloud_input_stamp;
	global_map_msg_rgb.header.frame_id = world_frame_;
	pubGlobal_all_rgb.publish(global_map_msg_rgb);
  }

  // 发布未进行降采样的全局地图
  pcl::PointCloud<PointPose3D>::Ptr global_map_keyframes_copy(new pcl::PointCloud<PointPose3D>());
  *global_map_keyframes_copy = *global_map_keyframes;  // 拷贝一份global_map_keyframes给global_map_keyframes_copy
  sensor_msgs::PointCloud2 global_map_msg_copy;
  pcl::toROSMsg(*global_map_keyframes_copy, global_map_msg_copy);
  global_map_msg_copy.header.stamp = robots[id_].time_cloud_input_stamp;
  global_map_msg_copy.header.frame_id = world_frame_;
  pubGlobal_all.publish(global_map_msg_copy);

  // downsample visualized points
  pcl::VoxelGrid<PointPose3D> downsample_filter_for_global_map;	 // for global map visualization
  downsample_filter_for_global_map.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
  downsample_filter_for_global_map.setInputCloud(global_map_keyframes);
  downsample_filter_for_global_map.filter(*global_map_keyframes_ds);

  // publish global map
  sensor_msgs::PointCloud2 global_map_msg;
  pcl::toROSMsg(*global_map_keyframes_ds, global_map_msg);
  global_map_msg.header.stamp = robots[id_].time_cloud_input_stamp;
  global_map_msg.header.frame_id = world_frame_;
  pub_global_map.publish(global_map_msg);
}

void distributedMapping::publishLoopClosureConstraint()
{
  if (loop_indexs.empty())
  {
	return;
  }

  // loop nodes
  visualization_msgs::Marker nodes;
  nodes.header.frame_id = world_frame_;
  nodes.header.stamp = ros::Time::now();
  nodes.action = visualization_msgs::Marker::ADD;
  nodes.type = visualization_msgs::Marker::SPHERE_LIST;
  nodes.ns = "loop_nodes";
  nodes.id = 0;
  nodes.pose.orientation.w = 1;
  nodes.scale.x = 0.3;
  nodes.scale.y = 0.3;
  nodes.scale.z = 0.3;
  nodes.color.r = 0;
  nodes.color.g = 0.8;
  nodes.color.b = 1;
  nodes.color.a = 1;

  // loop edges
  visualization_msgs::Marker constraints;
  constraints.header.frame_id = world_frame_;
  constraints.header.stamp = ros::Time::now();
  constraints.action = visualization_msgs::Marker::ADD;
  constraints.type = visualization_msgs::Marker::LINE_LIST;
  constraints.ns = "loop_constraints";
  constraints.id = 1;
  constraints.pose.orientation.w = 1;
  constraints.scale.x = 0.1;
  constraints.color.r = 0.9;
  constraints.color.g = 0.9;
  constraints.color.b = 0;
  constraints.color.a = 1;

  // check all accepted loop closure
  Pose3 pose;
  int index0, index1;
  for (auto it = loop_indexs.begin(); it != loop_indexs.end(); ++it)
  {
	index0 = it->first;
	index1 = it->second;

	geometry_msgs::Point p;
	pose = initial_values->at<Pose3>(Symbol('a' + id_, index0));
	p.x = pose.translation().x();
	p.y = pose.translation().y();
	p.z = pose.translation().z();
	nodes.points.push_back(p);
	constraints.points.push_back(p);
	pose = initial_values->at<Pose3>(Symbol('a' + id_, index1));
	p.x = pose.translation().x();
	p.y = pose.translation().y();
	p.z = pose.translation().z();
	nodes.points.push_back(p);
	constraints.points.push_back(p);
  }

  // publish loop closure markers
  visualization_msgs::MarkerArray markers_array;
  markers_array.markers.push_back(nodes);
  markers_array.markers.push_back(constraints);
  pub_loop_closure_constraints.publish(markers_array);
}