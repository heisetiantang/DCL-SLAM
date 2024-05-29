#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <string>
#include <sstream>
#include <sys/stat.h>
#include <signal.h>
#include <stdio.h>
#include <condition_variable>
#include <gtsam/nonlinear/ISAM2.h>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
// dcl_slam define
#include "paramsServer.h"
#include "dcl_slam/loop_info.h"
#include "dcl_slam/global_descriptor.h"
#include "dcl_slam/neighbor_estimate.h"

// mapping
#include "distributed_mapper/distributed_mapper.h"
#include "distributed_mapper/distributed_mapper_utils.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// file iostream
#include <fstream>
#include <iostream>
// log
#include <glog/logging.h>

#include "distributedMapping.h"
#include "NdtMatch/ndt_match.h"

using namespace gtsam;
using namespace std;
ros::Publisher pub_a_map;
ros::Publisher pub_b_map;
ros::Publisher pub_c_map;
ros::Publisher pub_mulitple_map;
// 订阅a、b、c机器人的fastlio前端的去畸变点云话题
ros::Subscriber sub_cloud_registered_a, sub_cloud_registered_b, sub_cloud_registered_c;
// 订阅a、b、c机器人的全局轨迹话题
ros::Subscriber subPath_a, subPath_b, subPath_c;

// 订阅回环rotation_estimates信息
ros::Subscriber sub_neighborms_rotation_estimates_a, sub_neighborms_rotation_estimates_b,
    sub_neighborms_rotation_estimates_c;
ros::Subscriber sub_neighbor_pose_estimates_a, sub_neighbor_pose_estimates_b, sub_neighbor_pose_estimates_c;

// 测试发布
// ros::Publisher pub_test_a;

struct Pose
{
    double timestamp;
    double tx, ty, tz;     // 位移
    double qx, qy, qz, qw; // 四元数
};
typedef pcl::PointXYZI PointPose3D;

/* ******************初始化参数定义****************** */
std::mutex mtx;                                          // 进程锁
std::condition_variable sig_buffer;                      // 终止信号
bool flg_stop = false;                                   // 程序退出标志位
bool flg_export_single = false, flg_export_mulit = true; // 是否输出地图的标志位
bool flg_visual_single = true, flg_visual_mulit = true;  // 是否可视化地图的标志位
// 每隔10秒输出一个标志位
bool time_count = false;

// 轨迹文件和地图文件的位置
#define DIR_GT std::getenv("HOME") + std::string("/out")
std::string gt_file = DIR_GT + std::string("/street_04.txt");
std::ifstream GT(gt_file);
//  定义地图保存位置
#define DIR_MAP std::getenv("HOME") + std::string("/out/map")
std::string map_file_a = DIR_MAP + std::string("/file_a.pcd");
std::string map_file_b = DIR_MAP + std::string("/file_b.pcd");
std::string map_file_c = DIR_MAP + std::string("/file_c.pcd");
std::string map_file_multiple = DIR_MAP + std::string("/file_multiple.pcd");

// 定义变换矩阵的保存位置
#define DIR_OUTPUT std::getenv("HOME") + std::string("/out/dcl_output")
string trans_B2A = DIR_OUTPUT + string("/trans_B2A.txt");
string trans_C2A = DIR_OUTPUT + string("/trans_C2A.txt");
// 创建并初始化 trans_B2A.txt 文件
std::ofstream file_B2A(trans_B2A);
// 创建并初始化 trans_C2A.txt 文件
std::ofstream file_C2A(trans_C2A);

// 输出的
nav_msgs::Path gt_path;

// 融合点云保存容器
std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudFramesArray_a;
std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudFramesArray_b;
std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudFramesArray_c;

// 初始化拼接后的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_stitched(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Path path_a_new;
nav_msgs::Path path_b_new;
nav_msgs::Path path_c_new;

// 轨迹位姿容器
std::vector<nav_msgs::Path> pathArray_a;
std::vector<nav_msgs::Path> pathArray_b;
std::vector<nav_msgs::Path> pathArray_c;

/* ********************函数部分*********************** */
// 判断文件夹是否存在
bool directoryExists(const std::string &path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}
// 程序终止进程
void Stop_flg(int sig)
{
    ROS_WARN("process stop!");
    flg_stop = true;
}

/* // Eigen::Affine3d getTransformationMatrix(const nav_msgs::Path &path)
// {
//     // 获取最新的轨迹点
//     const geometry_msgs::PoseStamped path_point = path.poses.back();

//     // 将轨迹点转换为变换矩阵
//     Eigen::Affine3d transformation_matrix = Eigen::Affine3d::Identity();
//     // 获取位置
//     geometry_msgs::Point position = path_point.pose.position;
//     // 将 position 转换为 Eigen::Vector3d
//     Eigen::Vector3d translation(position.x, position.y, position.z);
//     // 将 translation 赋值给变换矩阵的平移部分
//     transformation_matrix.translation() = translation;

//     // 获取姿态
//     geometry_msgs::Quaternion orientation = path_point.pose.orientation;
//     // 将 orientation 转换为 Eigen::Quaterniond
//     Eigen::Quaterniond eigen_orientation(orientation.x, orientation.y, orientation.z, orientation.w);

//     // 将 eigen_orientation 转换为旋转矩阵
//     transformation_matrix.linear() = eigen_orientation.toRotationMatrix();

//     return transformation_matrix;
// }

// // 拼接点云
// void stitch_clouds()
// {
//     // 开始拼接点云
//     // cout << "stitch_clouds" << endl;
//     // 检查是否有点云要拼接
//     if (cloudFramesArray_a.empty() && cloudFramesArray_b.empty() && cloudFramesArray_c.empty())
//     {
//         ROS_INFO("cloudFramesArray_a.empty() && cloudFramesArray_b.empty() && cloudFramesArray_c.empty()");

//         ROS_INFO("cloudFramesArray_c.size() = %d", cloudFramesArray_c.size());
//         ROS_INFO("cloudFramesArray_b.size() = %d", cloudFramesArray_b.size());
//         ROS_INFO("cloudFramesArray_a.size() = %d", cloudFramesArray_a.size());

//         return;
//     }

//     mtx.lock();
//         // 将cloudFramesArray_a中的点云进行拼接
//         for (int i = 0; i < cloudFramesArray_a.size(); i++)
//         {
//             *cloud_stitched += cloudFramesArray_a[i];
//             cloud_stitched->header.frame_id = cloudFramesArray_a[i].header.frame_id;
//             cloud_stitched->header.stamp = cloudFramesArray_a[i].header.stamp;
//         }
//         // ROS_INFO("cloudFramesArray_a.size() = %d", cloudFramesArray_a.size());
//         // 将cloudFramesArray_b中的点云进行拼接
//         for (int i = 0; i < cloudFramesArray_b.size(); i++)
//         {
//             *cloud_stitched += cloudFramesArray_b[i];
//             cloud_stitched->header.frame_id = cloudFramesArray_b[i].header.frame_id;
//             cloud_stitched->header.stamp = cloudFramesArray_b[i].header.stamp;

//         }
//         // ROS_INFO("cloudFramesArray_b.size() = %d", cloudFramesArray_b.size());
//         // 将cloudFramesArray_c中的点云进行拼接
//         for (int i = 0; i < cloudFramesArray_c.size(); i++)
//         {
//             *cloud_stitched += cloudFramesArray_c[i];
//             cloud_stitched->header.frame_id = cloudFramesArray_c[i].header.frame_id;
//             cloud_stitched->header.stamp = cloudFramesArray_c[i].header.stamp;
//         }
//         // ROS_INFO("cloudFramesArray_c.size() = %d", cloudFramesArray_c.size());

//         mtx.unlock();

//     pcl::PCDWriter writer_multiple;
//     writer_multiple.writeASCII(map_file_multiple, *cloud_stitched);

//     cout << "拼接完毕" << endl;
//     cloud_stitched->clear();

//   /*   // 进行多机器人之间的矫正,确保path_a_new存在，
//     if (path_a_new.poses.empty() && path_b_new.poses.empty() && path_c_new.poses.empty())
//     {
//         ROS_INFO("没有轨迹");
//         return;
//     }

//     // 比较时间
//     // cloudFramesArray_a的第一帧点云的时间戳
//     // uint64_t time_map_a_unit = cloudFramesArray_a[0].header.stamp;
//     // ros::Time time_map_a = ros::Time::fromNSec(static_cast<int64_t>(time_map_a_unit));
//     // for (size_t i = 0; i < path_a_new.poses.size();i++)
//     // {
//     //     // 获取最后一个 PoseStamped 消息的时间戳
//     //     ros::Time last_timestamp_a = path_a_new.poses[i].header.stamp;

//     //     // 比较last_timestamp_a和time_map_a
//     //     if (std::abs(last_timestamp_a - time_map_a) > 1)
//     //     {

//     // cout << last_timestamp_a << endl;
//     //     }
//     // }

//     // 获取机器人a最新的轨迹点
//     Eigen::Affine3d transformation_matrix_a = getTransformationMatrix(path_a_new);
//     // 获取机器人b、c的最新轨迹点
//     Eigen::Affine3d transformation_matrix_b = getTransformationMatrix(path_b_new);
//     Eigen::Affine3d transformation_matrix_c = getTransformationMatrix(path_c_new);
//     // 计算机器人b、c的变换矩阵的逆
//     Eigen::Affine3d transformation_matrix_b_inv = transformation_matrix_b.inverse();
//     Eigen::Affine3d transformation_matrix_c_inv = transformation_matrix_c.inverse();

//     // 计算机器人b、c到a的变换矩阵
//     Eigen::Affine3d transformation_matrix_ab = transformation_matrix_b_inv * transformation_matrix_a;
//     Eigen::Affine3d transformation_matrix_ac = transformation_matrix_c_inv * transformation_matrix_a;

//     // 输出变换矩阵
//     // std::cout << "transformation_matrix_ac:\n"
//     //           << transformation_matrix_ac.matrix() << std::endl;
//     // std::cout << "transformation_matrix_ab:\n"
//     //           << transformation_matrix_ab.matrix() << std::endl;

//     // 设置三个点云指针用于存放矫正后的点云
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pre_a(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pre_b(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pre_c(new pcl::PointCloud<pcl::PointXYZI>());

//     mtx.lock();
//     // 将cloudFramesArray_a中的点云进行拼接

//     // 将cloudFramesArray_a,cloudFramesArray_b,cloudFramesArray_c中的点云进行拼接
//     for (int i = 0; i < cloudFramesArray_a.size(); i++)
//     {
//         *cloud_pre_a += cloudFramesArray_a[i];
//     }
//     // ROS_INFO("cloudFramesArray_a.size() = %d", cloudFramesArray_a.size());
//     // ROS_INFO("path_a_new.size() = %d", path_a_new.poses.size());
//     // 将cloudFramesArray_b中的点云进行拼接
//     for (int i = 0; i < cloudFramesArray_b.size(); i++)
//     {
//         *cloud_pre_b += cloudFramesArray_b[i];
//     }
//     // ROS_INFO("cloudFramesArray_b.size() = %d", cloudFramesArray_b.size());
//     // ROS_INFO("path_b_new.size() = %d", path_b_new.poses.size());
//     // 将cloudFramesArray_c中的点云进行拼接
//     for (int i = 0; i < cloudFramesArray_c.size(); i++)
//     {
//         *cloud_pre_c += cloudFramesArray_c[i];
//     }
//     // ROS_INFO("cloudFramesArray_c.size() = %d", cloudFramesArray_c.size());
//     // ROS_INFO("path_c_new.size() = %d", path_c_new.poses.size());
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_b_a(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_c_a(new pcl::PointCloud<pcl::PointXYZI>());

//     // 将b点云变换到a坐标系下
//     pcl::transformPointCloud(*cloud_pre_b, *cloud_b_a, transformation_matrix_ab);

//     // 将c点云变换到a坐标系下
//     pcl::transformPointCloud(*cloud_pre_c, *cloud_c_a, transformation_matrix_ac);
//     *cloud_stitched += *cloud_pre_a;
//     *cloud_stitched += *cloud_b_a;
//     *cloud_stitched += *cloud_c_a;

//     mtx.unlock();
//     pcl::PCDWriter writer_multiple;
//     writer_multiple.writeASCII(map_file_multiple, *cloud_stitched);

//     cout << "拼接完毕" << endl;
//     cloud_stitched->clear();*/
// }
// // 函数获取轨迹
// void GetGroundTrajectory()
// {
//     geometry_msgs::PoseStamped convert_pose;

//     gt_path.header.frame_id = "world"; // 使用适当的坐标系
//     nav_msgs::Path after_gt_path;
//     after_gt_path.header.frame_id = "world"; // 使用适当的坐标系

//     // 读取第一个真值位姿
//     std::string first_line;
//     std::getline(GT, first_line);
//     std::istringstream first_iss(first_line);
//     double first_timestamp, first_tx, first_ty, first_tz, first_qx, first_qy, first_qz, first_qw;
//     if (!(first_iss >> first_timestamp >> first_tx >> first_ty >> first_tz >> first_qx >> first_qy >> first_qz >>
//     first_qw))
//     {
//         ROS_WARN_STREAM("Failed to parse line: " << first_line);
//         GT.close();
//         return;
//     }
//     // 使用convert_pose保存第一个真值位姿到原点的变换
//     convert_pose.pose.position.x = first_tx;
//     convert_pose.pose.position.y = first_ty;
//     convert_pose.pose.position.z = first_tz;
//     convert_pose.pose.orientation.x = first_qx;
//     convert_pose.pose.orientation.y = first_qy;
//     convert_pose.pose.orientation.z = first_qz;
//     convert_pose.pose.orientation.w = first_qw;

//     // 手动旋转轨迹
//     double pitch_angle = 90.0; // 俯仰角（以度为单位）
//     double yaw_angle = 45.0;   // 偏转角（以度为单位）
//     double roll_angle = 45.0;  // 翻滚角（以度为单位）
//     // 将角度转换为弧度
//     double pitch_rad = pitch_angle * M_PI / 180.0;
//     double yaw_rad = yaw_angle * M_PI / 180.0;
//     double roll_rad = roll_angle * M_PI / 180.0;

//     // 计算旋转的四元数
//     tf::Quaternion rotation_quat;
//     rotation_quat.setRPY(roll_rad, pitch_rad, yaw_rad);
//     std::cout << "rotation_quat: " << rotation_quat.x() << " " << rotation_quat.y() << " " << rotation_quat.z() << "
//     " << rotation_quat.w() << std::endl;

//     std::string line;
//     while (std::getline(GT, line))
//     {
//         std::istringstream iss(line);
//         double timestamp, tx, ty, tz, qx, qy, qz, qw;
//         if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
//         {
//             ROS_WARN_STREAM("Failed to parse line: " << line);
//             GT.close();
//         }

//         // 其他点通过变换进行调整
//         geometry_msgs::PoseStamped pose_stamped;
//         pose_stamped.header.stamp = ros::Time(timestamp);
//         pose_stamped.header.frame_id = gt_path.header.frame_id;
//         pose_stamped.pose.position.x = tx - convert_pose.pose.position.x;
//         pose_stamped.pose.position.y = ty - convert_pose.pose.position.y;
//         // pose_stamped.pose.position.z = tz - convert_pose.pose.position.z;
//         pose_stamped.pose.position.z = 0;
//         pose_stamped.pose.orientation.x = 0;
//         pose_stamped.pose.orientation.y = 0;
//         pose_stamped.pose.orientation.z = 0;
//         pose_stamped.pose.orientation.w = 0;
//         gt_path.poses.push_back(pose_stamped);
//     }
//     // std::cout<<"读取完毕"<<std::endl;
//     // 关闭文件
//     GT.close();
// }

// // 点云订阅
// void save_cloud_registered_Handler_a(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_a)
// {
//     // 空指针检查
//     if (cloud_msg_a == nullptr)
//     {
//         ROS_ERROR("cloud_msg_a is nullptr");
//         return;
//     }
//     if (flg_visual_single)
//     {

//         // 直接将得到的单机地图发布出来
//         sensor_msgs::PointCloud2 cloud_map_temp;
//         cloud_map_temp = *cloud_msg_a;
//         cloud_map_temp.header.frame_id = cloud_msg_a->header.frame_id;
//         cloud_map_temp.header.stamp = ros::Time::now();
//         pub_a_map.publish(cloud_map_temp);
//     }
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_now(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::fromROSMsg(*cloud_msg_a, *cloud_temp_now);
//     cloud_stitched->insert(cloud_stitched->end(), cloud_temp_now->begin(), cloud_temp_now->end());

//     if (flg_export_single)
//     { // 将得到的cloud_msg拷贝到cloudKeyFrames中

//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyFrames_a(new pcl::PointCloud<pcl::PointXYZI>());
//         pcl::fromROSMsg(*cloud_msg_a, *cloudKeyFrames_a);

//         pcl::PCDWriter writer_temp;
//         // 将cloudKeyFrames保存pcd文件，并且输出
//         writer_temp.writeBinaryCompressed(map_file_a, *cloudKeyFrames_a);
//     }
// }
// void save_cloud_registered_Handler_b(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_b)
// {
//     // 空指针检查
//     if (cloud_msg_b == nullptr)
//     {
//         ROS_ERROR("cloud_msg_b is nullptr");
//         return;
//     }
//     if (flg_visual_single)
//     {

//         sensor_msgs::PointCloud2 cloud_map_temp;
//         cloud_map_temp = *cloud_msg_b;
//         cloud_map_temp.header.frame_id = cloud_msg_b->header.frame_id;
//         cloud_map_temp.header.stamp = ros::Time::now();
//         pub_b_map.publish(cloud_map_temp);
//     }
//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_now(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::fromROSMsg(*cloud_msg_b, *cloud_temp_now);
//     cloud_stitched->insert(cloud_stitched->end(), cloud_temp_now->begin(), cloud_temp_now->end());

//     if (flg_export_single)
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyFrames_b(new pcl::PointCloud<pcl::PointXYZI>());
//         pcl::fromROSMsg(*cloud_msg_b, *cloudKeyFrames_b);

//         pcl::PCDWriter writer_temp;
//         // 将cloudKeyFrames保存pcd文件，并且输出
//         writer_temp.writeBinaryCompressed(map_file_b, *cloudKeyFrames_b);
//     }
//     /* if (flg_export_mulit)
//     {
//         // 将 cloudKeyFrames 指针保存到 cloudFramesArray_b
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
//         pcl::fromROSMsg(*cloud_msg_b, *cloud_temp);
//         cloud_temp->header.frame_id = cloud_msg_b->header.frame_id;
//         cloud_temp->header.stamp = ros::Time::now().toNSec()* 1e3;

//         cloudFramesArray_b.push_back(*cloud_temp);
//         // 清零cloud_temp
//         cloud_temp->clear();
//         // cout << "cloudFramesArray_b.size()" << cloudFramesArray_b.size() << endl;
//     } */

// }
// void save_cloud_registered_Handler_c(const sensor_msgs::PointCloud2ConstPtr &cloud_msg_c)
// {
//     // 空指针检查
//     if (cloud_msg_c == nullptr)
//     {
//         ROS_ERROR("cloud_msg_c is nullptr");
//         return;
//     }
//     if (flg_visual_single)
//     {

//         sensor_msgs::PointCloud2 cloud_map_temp;
//         cloud_map_temp = *cloud_msg_c;
//         cloud_map_temp.header.frame_id = cloud_msg_c->header.frame_id;
//         cloud_map_temp.header.stamp = ros::Time::now();
//         pub_c_map.publish(cloud_map_temp);
//     }

//     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_now(new pcl::PointCloud<pcl::PointXYZI>());
//     pcl::fromROSMsg(*cloud_msg_c, *cloud_temp_now);
//     cloud_stitched->insert(cloud_stitched->end(), cloud_temp_now->begin(), cloud_temp_now->end());

//     if (flg_export_single)
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloudKeyFrames_c(new pcl::PointCloud<pcl::PointXYZI>());
//         pcl::fromROSMsg(*cloud_msg_c, *cloudKeyFrames_c);

//         pcl::PCDWriter writer_temp;
//         // 将cloudKeyFrames保存pcd文件，并且输出
//         writer_temp.writeBinaryCompressed(map_file_c, *cloudKeyFrames_c);
//     }
//     /* if (flg_export_mulit)
//     {
//         // 将 cloudKeyFrames 指针保存到 cloudFramesArray_c
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
//         pcl::fromROSMsg(*cloud_msg_c, *cloud_temp);
//         cloud_temp->header.frame_id = cloud_msg_c->header.frame_id;
//         cloud_temp->header.stamp = ros::Time::now().toNSec()* 1e3;
//         cloudFramesArray_c.push_back(*cloud_temp);
//         // cout << "cloudFramesArray_c.size()" << cloudFramesArray_c.size() << endl;
//         // 清零cloud_temp
//         cloud_temp->clear();
//     } */

// }
// // 轨迹订阅
// void subPathHandler_a(const nav_msgs::PathConstPtr &path_msg_a)
// {
//     // 空指针检查
//     if (path_msg_a == nullptr)
//     {
//         ROS_ERROR("cloud_msg is nullptr");
//         return;
//     }

//     // nav_msgs::Path path_msg_temp;
//     // path_msg_temp = *path_msg_a;
//     // pathArray_a.push_back(path_msg_temp);

//     // 初始化path_a_new
//     path_a_new.poses.clear();
//     path_a_new = *path_msg_a;
//     path_a_new.header.frame_id = path_msg_a->header.frame_id;
//     path_a_new.header.stamp = path_msg_a->header.stamp;

//     // 输出轨迹path_a_new的时间戳详细时间
//     // ROS_INFO("path_a_new.header.stamp = %f",
//     (path_a_new.header.stamp.toSec()+path_a_new.header.stamp.toNSec()*1e-9));
//     // 输出轨迹path_msg_a的时间戳
//     // ROS_INFO("path_msg_a->header.stamp = %f",
//     (path_msg_a->header.stamp.toSec()+path_msg_a->header.stamp.toNSec()*1e-9));
// }
// void subPathHandler_b(const nav_msgs::PathConstPtr &path_msg_b)
// {
//     // cout << "subPathHandler_b" << endl;
//     // 空指针检查
//     if (path_msg_b == nullptr)
//     {
//         ROS_ERROR("cloud_msg is nullptr");
//         return;
//     }
//     // nav_msgs::Path path_msg_temp;
//     // path_msg_temp = *path_msg_b;
//     // pathArray_b.push_back(path_msg_temp);
//     path_b_new.poses.clear();
//     path_b_new = *path_msg_b;
//     path_b_new.header.frame_id = path_msg_b->header.frame_id;
//     path_b_new.header.stamp = path_msg_b->header.stamp;
// }
// void subPathHandler_c(const nav_msgs::PathConstPtr &path_msg_c)
// {
//     // cout << "subPathHandler_c" << endl;
//     // 空指针检查
//     if (path_msg_c == nullptr)
//     {
//         ROS_ERROR("cloud_msg is nullptr");
//         return;
//     }
//     // nav_msgs::Path path_msg_temp;
//     // path_msg_temp = *path_msg_c;
//     // pathArray_c.push_back(path_msg_temp);

//     path_c_new.poses.clear();
//     path_c_new = *path_msg_c;
//     path_c_new.header.frame_id = path_msg_c->header.frame_id;
//     path_c_new.header.stamp = path_msg_c->header.stamp;
// } */

void Callback(const sensor_msgs::PointCloud2ConstPtr &pointMsgXYZI_A,
              const sensor_msgs::PointCloud2ConstPtr &pointMsgXYZI_B,
              const sensor_msgs::PointCloud2ConstPtr &pointMsgXYZI_C)
{
    // 空指针检查
    if (pointMsgXYZI_A == nullptr || pointMsgXYZI_B == nullptr || pointMsgXYZI_C == nullptr)
    {
        ROS_ERROR("pointMsgXYZI_A or pointMsgXYZI_B or pointMsgXYZI_C is nullptr");
        return;
    }
    // pointMsgXYZI_A转换成pcl::PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_a(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pointMsgXYZI_A, *cloud_temp_a);
    // pointMsgXYZI_B转换成pcl::PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_b(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pointMsgXYZI_B, *cloud_temp_b);
    // pointMsgXYZI_C转换成pcl::PointXYZI
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_c(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pointMsgXYZI_C, *cloud_temp_c);

    // 计算机器人b、c到a的变换矩阵
    ROS_INFO("接收到数据");
    Eigen::Matrix4f transform_result = Eigen::Matrix4f::Identity();
    transform_result = init_guess_Get(1);
    transform_result = NDTMatching_M(cloud_temp_b, cloud_temp_a, transform_result);
    // 输出transform_result
    std::cout << "Transformation Matrix: \n"
              << transform_result << std::endl;

    // 将变换矩阵写入string trans_B2A = DIR_OUTPUT + string("/trans_B2A.txt");
    if (file_B2A.is_open())
    {
        // 将transform_result写入file_B2A
        file_B2A << transform_result << std::endl;
        file_B2A.close();
    }
    else
    {
        std::cerr << "Unable to open file: " << trans_B2A << std::endl;
        // 创建file_B2A
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gt_map_node");
    ros::NodeHandle nh;
    setlocale(LC_CTYPE, "zh_CN.utf8"); // 识别中文
    setlocale(LC_ALL, "");

       // 判断容器保存文件夹是否存在，存在则将地图保存到该位置
      if (!directoryExists(DIR_OUTPUT))
      {
        ROS_WARN("文件夹不存在");
      }
      else
      {
        ROS_INFO("文件夹存在");
    
      } 

    /*     // 发布当前机器人位姿的话题
        // ros::Publisher pub_gt_path = nh.advertise<nav_msgs::Path>("gt_path", 1, true);
        // 测试发布
        // pub_test_a = nh.advertise<sensor_msgs::PointCloud2>("test_a_cloud_registered", 10, true);

        // 发布单机器人地图
        pub_a_map = nh.advertise<sensor_msgs::PointCloud2>("a_map", 100, true);
        pub_b_map = nh.advertise<sensor_msgs::PointCloud2>("b_map", 100, true);
        pub_c_map = nh.advertise<sensor_msgs::PointCloud2>("c_map", 100, true);
        // pub_mulitple_map = nh.advertise<sensor_msgs::PointCloud2>("multiple_map", 10, true);
        // a、b、c机器人的稠密点云
        // sub_cloud_registered_a = nh.subscribe<sensor_msgs::PointCloud2>("/a/distributedMapping/copyGlobalMap", 100,
       save_cloud_registered_Handler_a);
        // sub_cloud_registered_b = nh.subscribe<sensor_msgs::PointCloud2>("/b/distributedMapping/copyGlobalMap", 100,
       save_cloud_registered_Handler_b);
        // sub_cloud_registered_c = nh.subscribe<sensor_msgs::PointCloud2>("/c/distributedMapping/copyGlobalMap", 100,
       save_cloud_registered_Handler_c);
        // a、b、c机器人的稀疏点云
        sub_cloud_registered_a = nh.subscribe<sensor_msgs::PointCloud2>("/a/distributedMapping/globalMap", 200,
       save_cloud_registered_Handler_a); sub_cloud_registered_b =
       nh.subscribe<sensor_msgs::PointCloud2>("/b/distributedMapping/globalMap", 200, save_cloud_registered_Handler_b);
        sub_cloud_registered_c = nh.subscribe<sensor_msgs::PointCloud2>("/c/distributedMapping/globalMap", 200,
       save_cloud_registered_Handler_c);

        // a、b、c机器人的全局轨迹话题
        subPath_a = nh.subscribe<nav_msgs::Path>("/a/distributedMapping/path", 1000, subPathHandler_a);
        subPath_b = nh.subscribe<nav_msgs::Path>("/b/distributedMapping/path", 1000, subPathHandler_b);
        subPath_c = nh.subscribe<nav_msgs::Path>("/c/distributedMapping/path", 1000, subPathHandler_c);

        // 接收neighbor_estimate信息
        // sub_neighborms_rotation_estimates_a =
       nh.subscribe<dcl_slam::neighbor_estimate>("/a/distributedMapping/neighbor_estimate", 1000,
       sub_neighbor_rotation_estimates_a);
     */

    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud_A(nh, "a/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud_B(nh, "b/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud_C(nh, "c/points", 1);
    // 使用ApproximateTime
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2>
        syncPolicy;
    // 使用ExactTime
    // typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
    //                                                   nav_msgs::Odometry>
    //     syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), subPointCloud_A, subPointCloud_B, subPointCloud_C);
    sync.registerCallback(boost::bind(&Callback, _1, _2, _3));

    // 创建一个定时器，每隔10秒触发一次
    std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now() + std::chrono::seconds(20);
    // 读取键盘输入的退出指令，调用函数使得标志位flg_exit为true，结束循环
    signal(SIGINT, Stop_flg);
    ros::Rate rate(1);
    while (!flg_stop && ros::ok())
    {
        /*         // 读取完毕，发布轨迹
                // pub_gt_path.publish(gt_path);

                // cout << "循环进行" << endl;
                // 检查定时器是否到期，如果当前时间每间隔10秒，则拼接一次
                if (std::chrono::system_clock::now() >= next_time)
                {
                    // 更新定时器
                    next_time = std::chrono::system_clock::now() + std::chrono::seconds(50);
                    time_count = true;
                }
                // 是否输出拼接地图flg_export_mulit
                if (flg_export_mulit)
                {
                    // 输出时间为ctrl+c的时间或者ros节点结束时间
                    if (flg_stop || !ros::ok() || time_count)
                    {
                        // 将拼接后的点云保存到 pcd 文件中
                        // 获得全局地图
                        // stitch_clouds();

                            pcl::PCDWriter writer_multiple;
                            writer_multiple.writeASCII(map_file_multiple, *cloud_stitched);
                            cout << "拼接完毕" << endl;
                            cloud_stitched->clear();


                        time_count = false;
                    }
                }
        */
        ros::spinOnce();
    }

    // 正常退出程序
    return 0;
}