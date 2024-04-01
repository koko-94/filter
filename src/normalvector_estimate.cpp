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
// #include <cmath>

// double deg2rad(double degrees)
// {
//     return degrees * M_PI / 180.0;
// }

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
    // pcl::io::loadPCDFile("../../data/rabbit.pcd", *cloud);
    pcl::io::loadPCDFile("../../data/milk.pcd", *cloud);
    // 法线估计
    normal_vector(8, cloud, cloud_normals);
    // 3. 合并点云和法线
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals);
    // 定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals); // 点云搜索树
    //  4. 三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // 定义三角化对象
    pcl::PolygonMesh triangles;                               // 定义最后网格模型
    gp3.setInputCloud(cloud_with_normals);                    // 输入
    gp3.setSearchRadius(0.2);                                 // 搜索半径
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

    // normal_estimation.setRadiusSearch(0.03); // 利用半径为3cm的球体中的所有邻点进行估计
    normal_estimation.setKSearch(num); // 使用当前点周围最近的10个点

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