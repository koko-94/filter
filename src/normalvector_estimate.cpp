/**************************************************************************

Author: SHR

Date:2024.3.20

FileName: estimate.cpp

Function:法向量估计

**************************************************************************/

#include <iostream>

#include <pcl/visualization/pcl_visualizer.h> //可视化库
#include <pcl/io/pcd_io.h>                    //输入输出库
#include <pcl/features/normal_3d.h>           //特征库

typedef pcl::PointXYZ PointT;

// 法向量提取 使用点数量，输入点云，法向量点云
void normal_vector(int num, const pcl::PointCloud<PointT>::Ptr &inputcloud,
                   const pcl::PointCloud<pcl::Normal>::Ptr &normalcloud);

int main(int argc, char **argv)
{
    // 读取点云
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // 导入点云
    pcl::io::loadPCDFile("../../data/rabbit.pcd", *cloud);

    normal_vector(8, cloud, cloud_normals);
    return 0;
}

void normal_vector(int num, const pcl::PointCloud<PointT>::Ptr &inputcloud,
                   const pcl::PointCloud<pcl::Normal>::Ptr &normalcloud)
{
    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;           // 创建法向量估计对象
    normal_estimation.setInputCloud(inputcloud);                            // 点云输入
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); // 创建一个空的kdtree
    normal_estimation.setSearchMethod(tree);                                // 将其传递给法线估计对象。
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);    // 定义估计的法线

    // normal_estimation.setRadiusSearch(0.03); // 利用半径为3cm的球体中的所有邻点进行估计
    normal_estimation.setKSearch(num); // 使用当前点周围最近的10个点

    // 进行法线估计并保存
    normal_estimation.compute(*normalcloud);
    pcl::io::savePCDFileASCII("../../data/normalvcs_cloud.pcd", *normalcloud);

    //  定义对象
    pcl::visualization::PCLVisualizer viewer;

    // // 将点云模型添加进可视化窗口
    viewer.addPointCloud(inputcloud, "rgb");
    // viewer.addPointCloud(cloud_filtered, "cloud_filtered", v2);
    viewer.addPointCloudNormals<PointT, pcl::Normal>(inputcloud, normalcloud, 10, 0.5); // 法向量长度、宽度

    // 开始显示2种方法,任选其一
    // 1. 阻塞式
    viewer.spin();
}