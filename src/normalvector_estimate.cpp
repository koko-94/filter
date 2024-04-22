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
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>
#include <cmath>

#include <pcl/surface/mls.h> //最小二乘法平滑滤波

// double deg2rad(double degrees)
// {
//     return degrees * M_PI / 180.0;
// }

typedef pcl::PointXYZ PointT;

// 法向量提取 使用点数量，输入点云，法向量点云
void normal_vector(int num, const pcl::PointCloud<PointT>::Ptr &inputcloud,
                   const pcl::PointCloud<pcl::Normal>::Ptr &normalcloud);

inline float signedVolumeOfTriangle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
{
    float v321 = p3.x * p2.y * p1.z;
    float v231 = p2.x * p3.y * p1.z;
    float v312 = p3.x * p1.y * p2.z;
    float v132 = p1.x * p3.y * p2.z;
    float v213 = p2.x * p1.y * p3.z;
    float v123 = p1.x * p2.y * p3.z;
    return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
}

float volumeOfMesh(pcl::PolygonMesh mesh)
{
    float vols = 0.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    for (int triangle = 0; triangle < mesh.polygons.size(); triangle++)
    {
        pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
        pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
        pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
        vols += signedVolumeOfTriangle(pt1, pt2, pt3);
    }
    return abs(vols);
}

int main(int argc, char **argv)
{
    // 读取点云
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // 导入点云
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file inputcloud \n");
        return (-1);
    }
    // pcl::io::loadPCDFile("../../data/rabbit.pcd", *cloud);
    // pcl::io::loadPCDFile("../../data/milk.pcd", *cloud);
    // 法线估计
    normal_vector(8, cloud, cloud_normals);
    // 3. 合并点云和法线
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);

    // 最小二乘法平滑处理
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setSearchMethod(tree);
    mls.setPolynomialOrder(2); // 2阶
    mls.setSearchRadius(0.2);  // 用于拟合的K近邻半径。在这个半径里进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.04);
    // mls.setUpsamplingStepSize(0.02);
    // mls.setPointDensity(100);
    mls.process(*cloud_with_normals);
    pcl::copyPointCloud(*cloud_with_normals, *cloud_filtered);

    // 定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals); // 点云搜索树
    //  4. 三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // 定义三角化对象
    pcl::PolygonMesh triangles;                               // 定义最后网格模型
    gp3.setInputCloud(cloud_with_normals);                    // 输入
    gp3.setSearchRadius(0.3);                                 // 搜索半径
    gp3.setMu(2.5);                                           // 设置被样本点搜索其最近邻点的最远距离，为了使用点云密度的变化
    gp3.setMaximumNearestNeighbors(100);                      // 样本点最大可搜索邻域个数
    gp3.setMaximumSurfaceAngle(M_PI / 4);                     // 设置最大表面角度，用于控制三角化结果的平滑程
    gp3.setMinimumAngle(M_PI / 18);                           // 设置最小角度，用于控制三角化结果的细节程度
    gp3.setMaximumAngle(2 * M_PI / 3);                        // 设置最大角度，用于控制三角化结果的细节程度
    // 以上都是默认参数，可以不设置

    gp3.setNormalConsistency(false); //  设置法向一致性，表示是否保持法向一致；如果设置为true，则生成三角化结果时会对法向进行一致性检查和调整

    gp3.setInputCloud(cloud_with_normals); // 设置带有法向信息的输入点云数据
    gp3.setSearchMethod(tree2);            // 设置搜索方法，用于三角化操作时的点云搜索
    gp3.reconstruct(triangles);            // 进行三角化操作，生成三角化结果

    //  附加顶点信息
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    // std::cout << volumeOfMesh(triangles) << std::endl;

    // Viewer
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addPolygonMesh(triangles);
    // viewer.addCoordinateSystem(1.0);
    // viewer.initCameraParameters();
    viewer.spin();
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

    normal_estimation.setRadiusSearch(0.03); // 利用半径为3cm的球体中的所有邻点进行估计
    // normal_estimation.setKSearch(num); // 使用当前点周围最近的10个点

    // 进行法线估计并保存
    normal_estimation.compute(*normalcloud);
    // pcl::io::savePCDFileASCII("../../data/normalvcs_cloud.pcd", *normalcloud);

    //  定义对象
    // pcl::visualization::PCLVisualizer viewer;

    // // 将点云模型添加进可视化窗口
    // viewer.addPointCloud(inputcloud, "rgb");
    // viewer.addPointCloud(cloud_filtered, "cloud_filtered", v2);
    // viewer.addPointCloudNormals<PointT, pcl::Normal>(inputcloud, normalcloud, 10, 0.5); // 法向量长度、宽度

    // 开始显示2种方法,任选其一
    // 1. 阻塞式
    // viewer.spin();
}