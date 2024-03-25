/**************************************************************************

Author: SHR

Date:2024.3.16

FileName: main.cpp

Function:main

**************************************************************************/

#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
// #include <pcl/filters/gaussian_filter.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZ PointT;

int main(int argc, char **argv)
{
    // 读取点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // 导入点云
    pcl::io::loadPCDFile("../../data/map.pcd", *cloud1);
    pcl::io::loadPCDFile("../../data/rabbit.pcd", *cloud2);

    // 定义对象
    pcl::visualization::PCLVisualizer viewer;
    int v1(1);
    int v2(2);
    // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);

    //  设置背景颜色，默认黑色
    //   viewer.setBackgroundColor(100, 100, 100); // rgb

    // --- 显示点云数据 ----
    // "cloud1" 为显示id，默认cloud,显示多个点云时用默认会报警告。
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud1); // rgb 显示本身颜色
    // pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud2); // rgb 显示本身颜色

    // 将点云模型添加进可视化窗口
    viewer.addPointCloud(cloud1, "rgb", v1);
    viewer.addPointCloud(cloud2, "cloud2", v2);

    // 开始显示2种方法,任选其一
    // 1. 阻塞式
    viewer.spin();
    return 0;

    // 创建点云指针
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    // // 填充点云数据
    // if (pcl::io::loadPCDFile("../rabbit.pcd", *cloud) == -1)
    // {
    // std::cout << "Can't read pcd file: "  << std::endl;
    // return -1;
    // }

    // // 点云可视化 创建窗口
    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // viewer.showCloud(cloud);

    // // 创建viewer后
    // // while (!viewer.wasStopped ())
    // // {
    // // // viewer.spinOnce ();
    // // // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // // }
    // getchar();  // 在Linux/Mac中
    // return 0;
}