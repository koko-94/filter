/**************************************************************************

Author: SHR

Date:2024.3.22

FileName: segmentation.cpp

Function:使用随机一致性算法RANSAC，对点云模型进行拟合处理，去除模型外噪点
         欧几里得聚类分割
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
#include <pcl/filters/project_inliers.h> //模型内点

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill

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
    // // 随机一致性法
    // pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    // pcl::PointCloud<PointT>::Ptr final(new pcl::PointCloud<PointT>);
    // // 用特定模型生成点云，并随机加入一些异常点
    // cloud->width = 500;
    // cloud->height = 1;
    // cloud->is_dense = false;
    // cloud->points.resize(cloud->width * cloud->height);
    // // 按要求生成平面或者球形点云
    // for (int i = 0; i < cloud->size(); ++i)
    // {
    //     if (pcl::console::find_argument(argc, argv, "-s") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
    //     {
    //         (*cloud)[i].x = 1024 * rand() / (RAND_MAX + 1.0);
    //         (*cloud)[i].y = 1024 * rand() / (RAND_MAX + 1.0);
    //         if (i % 5 == 0) // 异常点
    //             (*cloud)[i].z = 1024 * rand() / (RAND_MAX + 1.0);
    //         else if (i % 2 == 0) // 上半球
    //             (*cloud)[i].z = sqrt(1 - ((*cloud)[i].x * (*cloud)[i].x) - ((*cloud)[i].y * (*cloud)[i].y));
    //         else // // 下半球
    //             (*cloud)[i].z = -sqrt(1 - ((*cloud)[i].x * (*cloud)[i].x) - ((*cloud)[i].y * (*cloud)[i].y));
    //     }
    //     else
    //     {
    //         (*cloud)[i].x = 1024 * rand() / (RAND_MAX + 1.0);
    //         (*cloud)[i].y = 1024 * rand() / (RAND_MAX + 1.0);
    //         if (i % 2 == 0) // 异常点
    //             (*cloud)[i].z = 1024 * rand() / (RAND_MAX + 1.0);
    //         else // 平面
    //             (*cloud)[i].z = -1 * ((*cloud)[i].x + (*cloud)[i].y);
    //     }
    // }

    // if (pcl::console::find_argument(argc, argv, "-f") >= 0)
    //     user_segmentation_plane_seg(0.01, cloud, final);
    // else if (pcl::console::find_argument(argc, argv, "-sf") >= 0) // 球体
    //     user_segmentation_sphere(0.01, cloud, final);

    // // 5 在查看器中显示内部点或完整的点云云。
    // pcl::visualization::PCLVisualizer viewer;
    // if (pcl::console::find_argument(argc, argv, "-f") >= 0 || pcl::console::find_argument(argc, argv, "-sf") >= 0)
    //     // viewer = simpleVis(final);
    //     viewer.addPointCloud(final, "rgb");
    // else
    //     // viewer = simpleVis(cloud);
    //     viewer.addPointCloud(cloud, "rgb");

    // viewer.spin();

    // 聚类分割
    // (1)读入点云
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>),
        cloud_f(new pcl::PointCloud<PointT>);
    reader.read("wanduan.pcd", *cloud);
    // reader.read("../../data/new/fracture_line.pcd", *cloud);
    std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl;

    // (2)体素滤波
    //  Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud);
    // vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.setLeafSize(1.0f, 1.0f, 1.0f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl; //

    // (3)使用平面分割模型去掉 点云中的平面
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    // (4)保存去除的平面Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false); // 保留平面
    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

    // (5)除去平面模型的其他点云模型Remove the planar inliers, extract the rest
    extract.setNegative(true); // 保留除平面的模型
    extract.filter(*cloud_f);
    // *cloud_filtered = *cloud_f; // 检测断裂，不需要

    // （6）设置kd树 Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    // (7)聚类索引
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    // 每个点的邻域半径。如果取一个非常小的值，则可能会将实际对象视为多个簇。另一方面，如果将该值设置得过高，则可能会将多个对象视为一个簇。
    ec.setClusterTolerance(1.2); // 2cm
    ec.setMinClusterSize(200);   // 聚类簇的最小点数
    ec.setMaxClusterSize(75000); // 聚类簇的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);
    // std::cout << "size: " << cluster_indices.size() << std::endl;

    int j = 0;
    for (const auto &cluster : cluster_indices) // 每个聚类簇
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        { // 聚类簇中每个点的索引
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << j;
        writer.write<pcl::PointXYZ>("cloud_cluster_" + ss.str() + ".pcd", *cloud_cluster, false); //*
        j++;
    }

    pcl::visualization::PCLVisualizer viewer;
    if (pcl::console::find_argument(argc, argv, "-raw") >= 0)
        viewer.addPointCloud(cloud, "rgb");
    else if (pcl::console::find_argument(argc, argv, "-voxel") >= 0)
        viewer.addPointCloud(cloud_filtered, "rgb");
    else if (pcl::console::find_argument(argc, argv, "-in") >= 0)
        viewer.addPointCloud(cloud_plane, "rgb");
    else if (pcl::console::find_argument(argc, argv, "-out") >= 0)
        viewer.addPointCloud(cloud_f, "rgb");

    viewer.spin();

    return 0;
}
