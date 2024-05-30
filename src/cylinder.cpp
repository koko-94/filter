/**************************************************************************

Author: SHR

Date:2024.4.19

FileName: cylinder.cpp

Function:圆柱模型拟和

**************************************************************************/

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
typedef pcl::PointXYZ PointT;

int main()
{
    // All the objects needed
    pcl::PCDReader reader;
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Read in the cloud data
    reader.read("../../data/main.pcd", *cloud);
    std::cerr << "PointCloud has: " << cloud->size() << " data points." << std::endl;

    // 只保留z轴方向上[0，1.5]内的点
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(0, 1.5);
    // pass.filter(*cloud_filtered);
    // std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
    /*========== 圆柱形分割 ==========*/
    // 创建圆柱体分割对象并设置参数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1); // 曲面法线的影响权重设置为0.1的
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05); // 每个内点到模型的距离阈值:5cm。
    seg.setRadiusLimits(0, 1);      // 将圆柱形模型的半径限制为小于10厘米
    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);

    // 获得圆柱模型内部点和模型参数
    seg.segment(*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // 提取并保存cloud_filtered2中属于圆柱模型的内部点
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);

    if (cloud_cylinder->points.empty())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size() << " data points." << std::endl;
        // writer.write("table_scene_mug_stereo_textured_cylinder.pcd", *cloud_cylinder, false);
    }

    pcl::visualization::PCLVisualizer viewer;
    int v1(1);
    int v2(2);
    // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);

    // 将点云模型添加进可视化窗口
    viewer.addPointCloud(cloud, "cloud", v1);
    viewer.addPointCloud(cloud_cylinder, "cylinder", v2);

    // 开始显示2种方法,任选其一
    // 1. 阻塞式
    viewer.spin();

    return (0);
}
