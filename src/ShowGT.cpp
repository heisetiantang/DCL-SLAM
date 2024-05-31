#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
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
// #include "distributed_mapper/distributed_mapper.h"
// #include "distributed_mapper/distributed_mapper_utils.h"

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

ros::Publisher pub_trans_b2a, pub_trans_c2a;
// 创建一个 ROS 消息对象
std_msgs::Float32MultiArray matrix_msg_b2a;
std_msgs::Float32MultiArray matrix_msg_c2a;

/* ******************初始化参数定义****************** */
std::mutex mtx;                     // 进程锁
std::condition_variable sig_buffer; // 终止信号
bool flg_stop = false;              // 程序退出标志位
bool time_count = false;            // 每隔10秒输出一个标志位
// 定义变换矩阵的保存位置
#define DIR_OUTPUT std::getenv("HOME") + std::string("/out/dcl_output")
std::string trans_B2A = DIR_OUTPUT + "/trans_B2A.csv";
std::string trans_C2A = DIR_OUTPUT + "/trans_C2A.csv";
// 创建并初始化文件
std::ofstream file_B2A(trans_B2A);
std::ofstream file_C2A(trans_C2A);

Eigen::Matrix4f result_B2A = Eigen::Matrix4f::Identity();
Eigen::Matrix4f result_C2A = Eigen::Matrix4f::Identity();

/* ********************函数部分*********************** */
void writeMatrixToCSV(const std::string &filename, const Eigen::Matrix4f &matrix)
{
    std::ofstream file(filename, std::ios::app); // 打开文件以追加方式写入
    // std::ofstream file(filename, std::ios::out); // 打开文件以覆盖方式写入
    if (file.is_open())
    {
        for (int i = 0; i < matrix.rows(); ++i)
        {
            for (int j = 0; j < matrix.cols(); ++j)
            {
                file << matrix(i, j);
                if (j < matrix.cols() - 1)
                {
                    file << ",";
                }
            }
            file << "\n";
        }
        file.close();
    }
    else
    {
        std::cerr << "无法打开文件进行写入: " << filename << std::endl;
    }
}

// 判断文件夹是否存在
bool directoryExists(const std::string &path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}
// 创建文件夹
bool createDirectory(const std::string &path)
{
    if (!directoryExists(path))
    {
        if (mkdir(path.c_str(), 0777) == -1)
        {
            std::cerr << "无法创建文件夹: " << path << std::endl;
            return false;
        }
    }
    return true;
}
// 判断文件是否存在
bool fileExists(const std::string &filename)
{
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}
// 创建文件
bool createFile(const std::string &filename)
{
    if (!fileExists(filename))
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "无法创建文件: " << filename << std::endl;
            return false;
        }
        file.close();
    }
    return true;
}
// 程序终止进程
void Stop_flg(int sig)
{
    ROS_WARN("process stop!");
    flg_stop = true;
}

void Callback(const sensor_msgs::PointCloud2ConstPtr &pointMsgXYZI_A,
              const sensor_msgs::PointCloud2ConstPtr &pointMsgXYZI_B,
              const sensor_msgs::PointCloud2ConstPtr &pointMsgXYZI_C)
{
    ROS_INFO("接收到数据");
    if (pointMsgXYZI_A->width == 0)
    {
        // ROS_ERROR("主车辆A车必须存在");
        return;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_a(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*pointMsgXYZI_A, *cloud_temp_a);

    if (!pointMsgXYZI_B->width == 0)
    {
        ROS_INFO("B fabu ");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_b(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pointMsgXYZI_B, *cloud_temp_b);
        // 计算机器人b到a的变换矩阵
        result_B2A = init_guess_Get(1);
        result_B2A = NDTMatching_M(cloud_temp_b, cloud_temp_a, result_B2A);

        // 写入文件

        // 将 Eigen::Matrix4f 转换为消息类型
        matrix_msg_b2a.data.resize(16); // 设置数组大小

        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                matrix_msg_b2a.data[i * 4 + j] = result_B2A(i, j);
            }
        }
    }

    if (!pointMsgXYZI_C->width == 0)
    {
        ROS_INFO("C fabu ");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp_c(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*pointMsgXYZI_C, *cloud_temp_c);
        result_C2A = init_guess_Get(2);
        result_C2A = NDTMatching_M(cloud_temp_c, cloud_temp_a, result_C2A);
        matrix_msg_c2a.data.resize(16); // 设置数组大小

        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                matrix_msg_c2a.data[i * 4 + j] = result_C2A(i, j);
            }
        }
    }

    writeMatrixToCSV(trans_B2A, result_B2A);
    writeMatrixToCSV(trans_C2A, result_C2A);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gt_map_node");
    ros::NodeHandle nh;
    setlocale(LC_ALL, "");

    // 判断文件夹是否存在
    if (!directoryExists(DIR_OUTPUT))
    {
        if (!createDirectory(DIR_OUTPUT))
        {
            return 1;
        }
    }
    // 判断文件是否存在
    if (!createFile(trans_B2A) || !createFile(trans_C2A))
    {
        return 1;
    }

    // 接收点云计算
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud_A(nh, "a/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud_B(nh, "b/points", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud_C(nh, "c/points", 1);
    // 使用ApproximateTime
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                            sensor_msgs::PointCloud2>
        syncPolicy;
    typedef message_filters::Synchronizer<syncPolicy> Sync;

    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), subPointCloud_A, subPointCloud_B, subPointCloud_C);
    sync.setMaxIntervalDuration(ros::Duration(5));
    sync.registerCallback(boost::bind(&Callback, _1, _2, _3));

    // 发布计算出来的矩阵
    pub_trans_b2a = nh.advertise<std_msgs::Float32MultiArray>("matrix_topic_b2a", 10);
    pub_trans_c2a = nh.advertise<std_msgs::Float32MultiArray>("matrix_topic_c2a", 10);
    // 创建一个定时器，每隔10秒触发一次
    std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now() + std::chrono::seconds(20);
    // 读取键盘输入的退出指令，调用函数使得标志位flg_exit为true，结束循环
    signal(SIGINT, Stop_flg);
    ros::Rate rate(20);
    while (!flg_stop && ros::ok())
    {
        ros::spinOnce();

        pub_trans_b2a.publish(matrix_msg_b2a);
        pub_trans_c2a.publish(matrix_msg_c2a);
    }

    // 正常退出程序
    return 0;
}