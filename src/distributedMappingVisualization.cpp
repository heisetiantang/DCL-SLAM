#include "distributedMapping.h"
#include "NdtMatch/ndt_match.h"
#include <csignal>
#include <pcl/io/pcd_io.h>
bool flg_rgb_map_save = false;
bool flg_intensity_map_save = false;
std::string file_path = std::getenv("HOME") + std::string("/out/map");
std::string file_name_xyzrgb = "globalMap_High_xyzrgb.pcd";
std::string file_name_xyzi = "globalMap_High_xyzi.pcd";
std::mutex mutex_xyzI, mutex_xyzRGB;

// struct Frame_first frame_first;

// pcl::PointCloud<pcl::PointXYZI>::Ptr frame_first_a(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr frame_first_b(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::PointCloud<pcl::PointXYZI>::Ptr frame_first_c(new pcl::PointCloud<pcl::PointXYZI>);
// 程序终止进程
void Stop_flg(int sig)
{
  ROS_WARN("process stop!");
  // 调用global_map_keyframes_rgb
  //  输出rgb地图
  flg_rgb_map_save = true;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  class distributedMapping: publish visualization msg
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void distributedMapping::globalMapThread()
{
  ros::Rate rate(1.0 / map_publish_interval_);  // update global map per 4s (default)

  while (ros::ok())
  {
    rate.sleep();

    publishGlobalMap();  // global map visualization

    publishLoopClosureConstraint();  // loop closure visualization
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
    *global_map_keyframes += *transformPointCloud(robots[id_].keyframe_cloud_array[pose_6d_tmp.intensity], &pose_6d_tmp,
                                                  &robots[id_].pose_buffer[pose_6d_tmp.intensity]);
    // *global_map_keyframes += *transformPointCloud(robots[id_].keyframe_cloud_array[pose_6d_tmp.intensity],
    // 											  &pose_6d_tmp);
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
      *global_map_keyframes_rgb += *transformPointCloud(robots[id_].keyframe_cloud_rgb_array[pose_6d_tmp.intensity],
                                                        &pose_6d_tmp, &robots[id_].pose_buffer[pose_6d_tmp.intensity]);
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

    // 保存全局地图到文件
    signal(SIGINT, Stop_flg);
    if (flg_rgb_map_save)
    {
      // ROS_INFO("地图文件存储路径：%s", file_path.c_str());
      mutex_xyzRGB.lock();
      if (global_map_keyframes_rgb->size() > 0)
      {
        ROS_WARN("robot_id:%s", std::to_string(id_).c_str());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr globalMap_High_Color_save(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*global_map_keyframes_rgb, *globalMap_High_Color_save);
        pcl::PCDWriter writer_multiple_xyzrgb;
        writer_multiple_xyzrgb.writeBinary(file_path + "/" + file_name_xyzrgb, *globalMap_High_Color_save);
        /* if (std::fstream(file_path + "/" + file_name_xyzrgb, std::ios::in).good())
        {
          // 如果PCD文件存在，则追加点
          ROS_INFO("append point cloud");

          // pcl::io::append_point_cloud(file_path + "/" + file_name_xyzrgb, *global_map_keyframes_rgb,
          //                             *globalMap_High_Color_save);
          std::ofstream file(file_path + "/" + file_name_xyzrgb, std::ios::binary | std::ios::app);
          if (!file.is_open())
          {
            std::cerr << "Error: Unable to open file for appending!" << std::endl;
            return;
          }
          if (pcl::io::savePCDFileBinaryCompressed(file_path + "/" + file_name_xyzrgb, *globalMap_High_Color_save) ==
        -1)
          {
            std::cerr << "Error: Unable to save point cloud!" << std::endl;
            return;
          }
          // 关闭文件
          file.close();
        }
        else
        {
          // 如果PCD文件不存在，则创建新文件并写入点
          ROS_INFO("create new file");
          writer_multiple_xyzrgb.writeBinary(file_path + "/" + file_name_xyzrgb, *globalMap_High_Color_save);
        }*/
      }
      else
      {
        ROS_ERROR("globalMap_High is empty");
      }
      mutex_xyzRGB.unlock();

      flg_rgb_map_save = false;
    }
  }





  // // ROS_WARN("当前机器人的id：%d", id_);
  // if (id_ == 0 && !robots[id_].keyframe_cloud->empty())
  // {
  //   ROS_INFO("a 车不动");
  //   frame_first.cloud_xyz_i_first_a = robots[id_].keyframe_cloud;
  // }
  // if (id_ == 1 && !robots[id_].keyframe_cloud->empty())
  // {
  //   ROS_INFO("b");
  //   frame_first.cloud_xyz_i_first_b = robots[id_].keyframe_cloud;
  //   // 设置一个转换函数用于pcl::transformPointCloud
  //   Eigen::Matrix4f transform_result = Eigen::Matrix4f::Identity();
  //   transform_result = init_guess_Get(id_);


  //   transform_result = result_matrix_get(id_, transform_result);
  //   // ROS_INFO("不 frame_first.cloud_xyz_i_first_a :%d",frame_first.cloud_xyz_i_first_a->size());


  //   // 计算ndt变换矩阵
  //   // transform_result = 
  //   pcl::transformPointCloud(*global_map_keyframes, *global_map_keyframes, transform_result);
  // }
  // // if (id_ == 2 && !robots[id_].keyframe_cloud->empty())
  // // {
  // //   ROS_INFO("c");
  // //   // 设置一个转换函数用于pcl::transformPointCloud
  // //   Eigen::Matrix4f transform_result = Eigen::Matrix4f::Identity();
  // //   transform_result = init_guess_Get(id_);

  // //   pcl::transformPointCloud(*global_map_keyframes, *global_map_keyframes, transform_result);
  // // }


  

  // 发布未进行降采样的全局地图
  pcl::PointCloud<PointPose3D>::Ptr global_map_keyframes_copy(new pcl::PointCloud<PointPose3D>());
  *global_map_keyframes_copy = *global_map_keyframes;  // 拷贝一份global_map_keyframes给global_map_keyframes_copy
  sensor_msgs::PointCloud2 global_map_msg_copy;
  pcl::toROSMsg(*global_map_keyframes_copy, global_map_msg_copy);
  global_map_msg_copy.header.stamp = robots[id_].time_cloud_input_stamp;
  global_map_msg_copy.header.frame_id = world_frame_;
  pubGlobal_all.publish(global_map_msg_copy);

  // downsample visualized points
  // pcl::VoxelGrid<PointPose3D> downsample_filter_for_global_map;	 // for global map visualization
  // downsample_filter_for_global_map.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
  // downsample_filter_for_global_map.setInputCloud(global_map_keyframes);
  // downsample_filter_for_global_map.filter(*global_map_keyframes_ds);

  // publish global map
  // sensor_msgs::PointCloud2 global_map_msg;
  // pcl::toROSMsg(*global_map_keyframes_ds, global_map_msg);
  // global_map_msg.header.stamp = robots[id_].time_cloud_input_stamp;
  // global_map_msg.header.frame_id = world_frame_;
  // pub_global_map.publish(global_map_msg);
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
  nodes.color.g = 1;
  nodes.color.b = 0;
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
  constraints.color.r = 1;
  constraints.color.g = 0;
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