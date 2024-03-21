#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/common/io.h>   // for copyPointCloud
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

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

    // 1 用于存储PointCloud中内部点的索引位置
    std::vector<int> inliers;

    // 2 使用平面或球体模型来构建RandomSampleConsensus对象。
    pcl::SampleConsensusModelSphere<PointT>::Ptr
        model_s(new pcl::SampleConsensusModelSphere<PointT>(cloud)); // 球体点云模型
    pcl::SampleConsensusModelPlane<PointT>::Ptr
        model_p(new pcl::SampleConsensusModelPlane<PointT>(cloud)); // 平面点云模型

    if (pcl::console::find_argument(argc, argv, "-f") >= 0)
    {
        // 3
        //  pcl::RandomSampleConsensus<PointT> ransac(model_p); // 创建一个RandomSampleConsensus对象，用于估计模型参数
        //  ransac.setDistanceThreshold(.01);                   // 设置点到模型的最大距离阈值，用于确定哪些点是内点
        //  ransac.computeModel();                              // 使用RANSAC算法计算模型参数
        //  ransac.getInliers(inliers);                         // 获取内点（即符合模型的点）的索引

        pcl::RandomSampleConsensus<PointT> ransac(model_p);
        ransac.setMaxIterations(5000);     // 设置最大迭代次数 默认10000
        ransac.setDistanceThreshold(0.01); // 设置距离阈值
        ransac.computeModel();             // 使用RANSAC算法计算模型参数
        ransac.getInliers(inliers);        // 获取内点（即符合模型的点）的索引
        // ransac.setOptimizeCoefficients(true);     // 优化模型系数
    }
    else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) // 球体
    {
        pcl::RandomSampleConsensus<PointT> ransac(model_s);
        ransac.setDistanceThreshold(.01);
        ransac.computeModel();
        ransac.getInliers(inliers);
    }

    // 4 将内部点复制到final
    pcl::copyPointCloud(*cloud, inliers, *final);

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
