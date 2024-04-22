/**************************************************************************

Author: SHR

Date:2024.3.16

FileName: subtraction.cpp

Function:两点云相减，得到损伤模型

**************************************************************************/

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/common/common.h> //getminmax
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>                  //体素滤波库
#include <pcl/filters/statistical_outlier_removal.h> //离群值滤波库
#include <pcl/filters/radius_outlier_removal.h>      //离群值滤波器

typedef pcl::PointXYZ PointT;

void subtractPointClouds(const pcl::PointCloud<PointT>::Ptr &cloud1,
                         const pcl::PointCloud<PointT>::Ptr &cloud2,
                         const pcl::PointCloud<PointT>::Ptr &diffcloud,
                         double threshold)
{
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud2);

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    for (size_t i = 0; i < cloud1->points.size(); ++i)
    {
        if (kdtree.nearestKSearch(cloud1->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if (pointNKNSquaredDistance[0] > threshold * threshold)
            {
                diffcloud->points.push_back(cloud1->points[i]);
            }
        }
    }
}

void voxelfilter(float size, const pcl::PointCloud<PointT>::Ptr &inputcloud, const pcl::PointCloud<PointT>::Ptr &outputcloud)
{
    pcl::VoxelGrid<PointT> sor; // 设置滤波器
    sor.setInputCloud(inputcloud);
    sor.setLeafSize(size, size, size); // 设置1m3的立方体
    sor.filter(*outputcloud);          // 保存滤波后点云

    // 计算点云数量进行比较
    std::cerr << "PointCloud before filtering: " << inputcloud->width * inputcloud->height
              << " data points (" << pcl::getFieldsList(*inputcloud) << ")." << std::endl;
    std::cerr << "PointCloud after filtering: " << outputcloud->width * outputcloud->height
              << " data points (" << pcl::getFieldsList(*outputcloud) << ")." << std::endl;
}

void RORfilter(int num, float radius, const pcl::PointCloud<PointT>::Ptr &inputcloud,
               const pcl::PointCloud<PointT>::Ptr &outputcloud)
{

    // 创建半径离群点去除滤波器对象
    pcl::RadiusOutlierRemoval<PointT> outrem;
    // 设置输入点云
    outrem.setInputCloud(inputcloud);
    // 设置搜索半径
    outrem.setRadiusSearch(radius);
    // 设置最小邻居点数阈值
    outrem.setMinNeighborsInRadius(num); // 执行滤波操作
    outrem.filter(*outputcloud);

    std::cerr << "PointCloud before RORfilter: " << inputcloud->width * inputcloud->height
              << " data points (" << inputcloud->points.size() << ")." << std::endl;
    std::cerr << "PointCloud after RORfilter: " << outputcloud->width * outputcloud->height
              << " data points (" << outputcloud->points.size() << ")." << std::endl;
}

void SORfilter(int num, float threshoud, const pcl::PointCloud<PointT>::Ptr &inputcloud,
               const pcl::PointCloud<PointT>::Ptr &outputcloud)
{
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *inputcloud << std::endl;

    // 创建一个pcl::StatisticalOutlierRemoval滤波器。
    // 所有距离查询点的平均距离的标准差大于1的点都将被标记为异常值并被删除。
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(inputcloud);
    sor.setMeanK(num);                 // 每个查询点要分析的邻居数量设置为 越大鲁棒性越好 算力要求越高
    sor.setStddevMulThresh(threshoud); // 标准差阈值设置为 越小滤越多
    sor.filter(*outputcloud);          // 计算输出并将其存储在cloud_filtered中。

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *outputcloud << std::endl;
}

int main(int argc, char **argv)
{
    // 读取点云
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr model2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr diffcloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr diffcloud2(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud2) == -1)
    {
        PCL_ERROR("Couldn't read file target \n");
        return (-1);
    }
    if (pcl::io::loadPCDFile<PointT>(argv[2], *model2) == -1)
    {
        PCL_ERROR("Couldn't read file model \n");
        return (-1);
    }
    voxelfilter(0.3f, cloud2, cloud);
    voxelfilter(0.3f, model2, model);

    std::cout << "cloud points:" << cloud->size() << std::endl;
    std::cout << "model points:" << model->size() << std::endl;
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    // 输出最小和最大点
    std::cout << "Min point: (" << min_pt[0] << ", " << min_pt[1] << ", " << min_pt[2] << ")" << std::endl;
    std::cout << "Max point: (" << max_pt[0] << ", " << max_pt[1] << ", " << max_pt[2] << ")" << std::endl;

    double threshold = 0.1; // 设置差异阈值
    subtractPointClouds(cloud, model, diffcloud, threshold);
    subtractPointClouds(model, cloud, diffcloud2, threshold);

    diffcloud->width = diffcloud->points.size();
    diffcloud->height = 1;
    diffcloud->is_dense = false;
    diffcloud->points.resize(diffcloud->width * diffcloud->height);
    std::cout << "diffcloud points:" << diffcloud->size() << std::endl;
    diffcloud2->width = diffcloud2->points.size();
    diffcloud2->height = 1;
    diffcloud2->is_dense = false;
    diffcloud2->points.resize(diffcloud2->width * diffcloud2->height);
    std::cout << "diffcloud2 points:" << diffcloud2->size() << std::endl;

    // 创建一个新的点云来存储合并后的结果
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 使用 '+' 运算符合并两个点云
    *merged_cloud = *diffcloud + *diffcloud2;
    RORfilter(5, 1.0f, merged_cloud, filtered_cloud);

    pcl::visualization::PCLVisualizer viewer("subview"); // 设置可视化窗口
    // v1 v2是视窗标识符
    int v1(1);
    int v2(2);
    int v3(3);
    int v4(4);
    int v5(5);
    int v6(6);
    // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    viewer.createViewPort(0, 0, 0.33, 0.5, v1);
    viewer.createViewPort(0.33, 0, 0.66, 0.5, v2);
    viewer.createViewPort(0.66, 0, 1, 0.5, v5);
    viewer.createViewPort(0, 0.5, 0.33, 1, v3);
    viewer.createViewPort(0.33, 0.5, 0.66, 1, v4);
    viewer.createViewPort(0.66, 0.5, 1, 1, v6);

    viewer.addPointCloud(cloud, "cloud", v1);
    viewer.addPointCloud(model, "cloud_Voxelfiltered", v2);
    viewer.addPointCloud(diffcloud, "cloud_SORfiltered", v3);
    viewer.addPointCloud(diffcloud2, "cloud_filtered", v4);
    viewer.addPointCloud(merged_cloud, "merged_cloud", v5);
    viewer.addPointCloud(filtered_cloud, "filtered_cloud", v6);
    viewer.spin();
    return 0;
}