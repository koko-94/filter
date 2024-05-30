/**************************************************************************

Author: SHR

Date:2024.3.19

FileName: filter.cpp

Function:some filter for pcl：体素滤波，直通滤波，离群值滤波（距离/搜索半径/搜索点数量）

**************************************************************************/

#include <iostream>                  //基本输入输出库
#include <pcl/point_types.h>         //点云类型库
#include <pcl/filters/passthrough.h> //直通滤波库
#include <pcl/io/pcd_io.h>           //pcl输入输出库
#include <pcl/filters/voxel_grid.h>  //体素滤波库
// #include <pcl/kdtree/kdtree_flann.h>//
#include <pcl/visualization/pcl_visualizer.h>        //可视化库
#include <pcl/filters/statistical_outlier_removal.h> //离群值滤波库
#include <pcl/filters/radius_outlier_removal.h>      //离群值滤波器
#include <pcl/ModelCoefficients.h>                   //模型库
#include <pcl/filters/project_inliers.h>             //

#include <pcl/surface/mls.h>        //最小二乘法平滑滤波
#include <pcl/features/normal_3d.h> //特征库
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;

typedef pcl::PointXYZ PointT;
// 体素滤波 小立方体内所有点云以一个点表示
void voxelfilter(float size, const pcl::PointCloud<PointT>::Ptr &inputcloud,
                 const pcl::PointCloud<PointT>::Ptr &outputcloud);
// 该滤波器根据每个点与其邻居之间的平均距离来工作，并去除那些其平均距离超过一定阈值的点。
void SORfilter(int num, float threshoud, const pcl::PointCloud<PointT>::Ptr &inputcloud,
               const pcl::PointCloud<PointT>::Ptr &outputcloud);
// 该滤波器基于每个点在其一定范围内（即搜索半径内）的邻近点数量进行过滤。用户设定一个搜索半径和一个邻近点数量的阈值
// ，如果一个点在其搜索半径内的邻近点数量少于这个阈值，那么这个点就被认为是离群点并被删除。
void RORfilter(int num, float radius, const pcl::PointCloud<PointT>::Ptr &inputcloud,
               const pcl::PointCloud<PointT>::Ptr &outputcloud);

int main(int argc, char **argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_Voxelfiltered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_SORfiltered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_RORfiltered(new pcl::PointCloud<PointT>);

    // // 5 points
    // cloud->width = 5;
    // cloud->height = 1;
    // cloud->points.resize(cloud->width * cloud->height);

    // // size -1~1
    // for (auto &point : *cloud)
    // {
    //     point.x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    // }

    // std::cerr << "Cloud before filtering: " << std::endl;
    // for (const auto &point : *cloud)
    //     std::cerr << "    " << point.x << " "
    //               << point.y << " "
    //               << point.z << std::endl;

    // pcl::PassThrough<PointTRGB> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("x"); // 指定滤波维度

    // // std::vector<float> limits;
    // // limits.push_back(0.0f);       // 最小值
    // // limits.push_back(1.0f);       // 最大值
    // // pass.setFilterLimits(limits); // 设置裁剪范围

    // pass.setFilterLimits(-1.0f, 0.0f); //  指定滤波范围（0.0,1.0）负逻辑
    // pass.setNegative(true);            // false: 滤除范围外的点（默认）；true:  滤除范围内的点
    // pass.filter(*cloud_filtered);

    // std::cerr << "Cloud after filtering: " << std::endl;
    // for (const auto &point : *cloud_filtered)
    //     std::cerr << "    " << point.x << " "
    //               << point.y << " "
    //               << point.z << std::endl;

    // pcl::PCDReader reader;                                    // 设置读入器
    // reader.read("../../data/table_scene_lms400.pcd", *cloud); // 读取点云
    pcl::io::loadPCDFile(argv[1], *cloud);

    // 应用体素滤波器
    voxelfilter(0.2f, cloud, cloud_Voxelfiltered);
    // SOR应用离群值滤波器
    SORfilter(50, 1.0f, cloud_Voxelfiltered, cloud_SORfiltered);
    // ROR
    // RORfilter(3, 0.005f, cloud, cloud_RORfiltered);
    // 3. 移动最小二乘法平滑

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_SORfiltered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_SORfiltered, *normals, *cloud_with_normals);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    mls.setInputCloud(cloud_SORfiltered);
    mls.setSearchMethod(tree);
    mls.setPolynomialOrder(2); // 2阶
    mls.setSearchRadius(1);    // 用于拟合的K近邻半径。在这个半径里进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。

    // mls.setUpsamplingMethod(RANDOM_UNIFORM_DENSITY); // 增加密度较小区域的密度对于holes的填补却无能为力，具体方法要结合参数使用
    mls.process(*cloud_with_normals);
    pcl::copyPointCloud(*cloud_with_normals, *cloud_filtered);
    //  Viewer
    pcl::visualization::PCLVisualizer viewer("viewer");
    // int v1(1);
    // int v2(2);
    // // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    // viewer.createViewPort(0, 0, 0.5, 1, v1);
    // viewer.createViewPort(0.5, 0, 1, 1, v2);
    // // pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb(cloud); // rgb 显示本身颜色

    // viewer.addPointCloud(cloud, "cloud", v1);
    // viewer.addPointCloud(cloud_filtered, "RORfiltered", v2);
    // viewer.spin();
    int v1(1);
    int v2(2);
    int v3(3);
    int v4(4);
    // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    viewer.createViewPort(0, 0, 0.5, 0.5, v1);
    viewer.createViewPort(0.5, 0, 1, 0.5, v2);
    viewer.createViewPort(0, 0.5, 0.5, 1, v3);
    viewer.createViewPort(0.5, 0.5, 1, 1, v4);

    viewer.addPointCloud(cloud, "cloud", v1);
    viewer.addPointCloud(cloud_Voxelfiltered, "cloud_Voxelfiltered", v2);
    viewer.addPointCloud(cloud_SORfiltered, "cloud_SORfiltered", v3);
    viewer.addPointCloud(cloud_filtered, "cloud_filtered", v4);
    viewer.spin();
    return 0;
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

    std::cerr << "PointCloud before filtering: " << inputcloud->width * inputcloud->height
              << " data points (" << inputcloud->points.size() << ").";
    std::cerr << "PointCloud after filtering: " << outputcloud->width * outputcloud->height
              << " data points (" << outputcloud->points.size() << ").";

    // 保存滤波后的点云到文件
    // pcl::io::savePCDFileASCII("../../data/RORfiltered_cloud.pcd", *outputcloud);
    // pcl::visualization::PCLVisualizer viewer("cloud_RORfiltered"); // 设置可视化窗口
    // // v1 v2是视窗标识符
    // int v1(1);
    // int v2(2);
    // // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    // viewer.createViewPort(0, 0, 0.5, 1, v1);
    // viewer.createViewPort(0.5, 0, 1, 1, v2);
    // // pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb(cloud); // rgb 显示本身颜色

    // viewer.addPointCloud(inputcloud, "cloud", v1);
    // viewer.addPointCloud(outputcloud, "RORfiltered", v2);
    // viewer.spin();
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

    pcl::PCDWriter writer;
    writer.write("../../data/voxel_fliered.pcd", *outputcloud);

    // pcl::visualization::PCLVisualizer viewer("Voxel_filter"); // 设置可视化窗口
    // // v1 v2是视窗标识符
    // int v1(1);
    // int v2(2);
    // // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    // viewer.createViewPort(0, 0, 0.5, 1, v1);
    // viewer.createViewPort(0.5, 0, 1, 1, v2);
    // // pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb(cloud); // rgb 显示本身颜色

    // viewer.addPointCloud(inputcloud, "cloud", v1);
    // viewer.addPointCloud(outputcloud, "filtered", v2);
    // viewer.spin();
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
    // 保存滤波后的点云：
    pcl::PCDWriter writer;
    writer.write<PointT>("../../data/sor_inliers.pcd", *outputcloud, false);

    // 保存异常点：
    // sor.setNegative(true);
    sor.filter(*outputcloud);
    // writer.write<PointT>("../../data/sor_outliers.pcd", *outputcloud, false);

    // pcl::visualization::PCLVisualizer viewer("SOR_filter"); // 设置可视化窗口
    // // v1 v2是视窗标识符
    // int v1(1);
    // int v2(2);
    // // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    // viewer.createViewPort(0, 0, 0.5, 1, v1);
    // viewer.createViewPort(0.5, 0, 1, 1, v2);
    // // pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb(cloud); // rgb 显示本身颜色

    // viewer.addPointCloud(inputcloud, "cloud", v1);
    // viewer.addPointCloud(outputcloud, "filtered", v2);
    // viewer.spin();
}