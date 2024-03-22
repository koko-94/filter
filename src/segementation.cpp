/**************************************************************************

Author: SHR

Date:2024.3.22

FileName: segmentation.cpp

Function:使用随机一致性算法RANSAC，对点云模型进行拟合处理，去除模型外噪点

**************************************************************************/

#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/common/io.h>   // for copyPointCloud
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
// 提炼模型库 引用失败
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

// 平面模型去除噪点
void user_segmentation_plane(double instance, const pcl::PointCloud<PointT>::Ptr &inputcloud,
                             const pcl::PointCloud<PointT>::Ptr &outputcloud)
{
    // 1 用于存储PointCloud中内部点的索引位置
    std::vector<int> inliers;

    // 2 使用平面或球体模型来构建RandomSampleConsensus对象。
    pcl::SampleConsensusModelPlane<PointT>::Ptr
        model_p(new pcl::SampleConsensusModelPlane<PointT>(inputcloud)); // 平面点云模型

    pcl::RandomSampleConsensus<PointT> ransac(model_p);
    ransac.setMaxIterations(5000);         // 设置最大迭代次数 默认10000
    ransac.setDistanceThreshold(instance); // 设置距离阈值
    ransac.computeModel();                 // 使用RANSAC算法计算模型参数
    ransac.getInliers(inliers);            // 获取内点（即符合模型的点）的索引

    // 4 将内部点复制到final
    pcl::copyPointCloud(*inputcloud, inliers, *outputcloud);
}

// 球体模型
void user_segmentation_sphere(double instance, const pcl::PointCloud<PointT>::Ptr &inputcloud,
                              const pcl::PointCloud<PointT>::Ptr &outputcloud)
{
    // 1 用于存储PointCloud中内部点的索引位置
    std::vector<int> inliers;

    // 2 使用平面或球体模型来构建RandomSampleConsensus对象。
    pcl::SampleConsensusModelSphere<PointT>::Ptr
        model_s(new pcl::SampleConsensusModelSphere<PointT>(inputcloud)); // 球体点云模型

    pcl::RandomSampleConsensus<PointT> ransac(model_s);
    ransac.setMaxIterations(5000);         // 设置最大迭代次数 默认10000
    ransac.setDistanceThreshold(instance); // 设置距离阈值
    ransac.computeModel();                 // 使用RANSAC算法计算模型参数
    ransac.getInliers(inliers);            // 获取内点（即符合模型的点）的索引

    // 4 将内部点复制到final
    pcl::copyPointCloud(*inputcloud, inliers, *outputcloud);
}

// 平面模型分离
void user_segmentation_plane_seg(double instance, const pcl::PointCloud<PointT>::Ptr &inputcloud,
                                 const pcl::PointCloud<PointT>::Ptr &outputcloud)
{
    // 从点云模型提炼对象
    //  创建RANSAC对象并设置参数
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);     // 最小二乘法优化模型系数
    seg.setModelType(pcl::SACMODEL_PLANE); // 平面
    seg.setMethodType(pcl::SAC_RANSAC);    // RANSAC方法
    seg.setMaxIterations(1000);            // 最大迭代次数
    seg.setDistanceThreshold(instance);    // 阈值
    seg.setInputCloud(inputcloud);         // 输入点云

    // 估计平面模型参数
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 模型参数
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);               // 内点
    seg.segment(*inliers2, *coefficients);                                // 分离
    // 检查是否成功找到了模型
    if (inliers2->indices.size() == 0)
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
    // 提取平面上的内点

    // 遍历inliers->indices中的每个索引，将原始点云cloud中对应的点添加到cloud_plane中
    for (std::vector<int>::const_iterator it = inliers2->indices.begin(); it != inliers2->indices.end(); ++it)
    {
        outputcloud->points.push_back(inputcloud->points[*it]);
    }
    outputcloud->width = outputcloud->points.size();
    outputcloud->height = 1;
    outputcloud->is_dense = true;

    // 保存内点到新的PCD文件
    // pcl::io::savePCDFileASCII("plane_inliers.pcd", *outputcloud);

    // 打印模型系数
    std::cout << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
}

int main(int argc, char **argv)
{

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr final(new pcl::PointCloud<PointT>);
    // 用特定模型生成点云，并随机加入一些异常点
    cloud->width = 500;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    // 按要求生成平面或者球形点云
    for (int i = 0; i < cloud->size(); ++i)
    {
        if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
        {
            (*cloud)[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            (*cloud)[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            if (i % 5 == 0) // 异常点
                (*cloud)[i].z = 1024 * rand() / (RAND_MAX + 1.0);
            else if (i % 2 == 0) // 上半球
                (*cloud)[i].z = sqrt(1 - ((*cloud)[i].x * (*cloud)[i].x) - ((*cloud)[i].y * (*cloud)[i].y));
            else // // 下半球
                (*cloud)[i].z = -sqrt(1 - ((*cloud)[i].x * (*cloud)[i].x) - ((*cloud)[i].y * (*cloud)[i].y));
        }
        else
        {
            (*cloud)[i].x = 1024 * rand() / (RAND_MAX + 1.0);
            (*cloud)[i].y = 1024 * rand() / (RAND_MAX + 1.0);
            if (i % 2 == 0) // 异常点
                (*cloud)[i].z = 1024 * rand() / (RAND_MAX + 1.0);
            else // 平面
                (*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
        }
    }

    if (pcl::console::find_argument(argc, argv, "-f") >= 0)
        user_segmentation_plane_seg(0.01, cloud, final);
    else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) // 球体
        user_segmentation_sphere(0.01, cloud, final);

    // 5 在查看器中显示内部点或完整的点云云。
    pcl::visualization::PCLVisualizer viewer;
    if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
        // viewer = simpleVis(final);
        viewer.addPointCloud(final, "rgb");
    else
        // viewer = simpleVis(cloud);
        viewer.addPointCloud(cloud, "rgb");

    viewer.spin();

    return 0;
}
