/**************************************************************************

Author: SHR

Date:2024.4.20

FileName: mentocarlo.cpp

Function:蒙特卡罗法填充点云内部点

**************************************************************************/

#include <pcl/point_types.h>
#include <pcl/common/common.h> //getminmax
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h> //可视化库

#include <pcl/features/normal_3d.h> //特征库
#include <pcl/surface/gp3.h>
#include <pcl/kdtree/kdtree_flann.h>

// #include <pcl/filters/voxel_grid.h>
// #include <random>

typedef pcl::PointXYZ PointT;

int main(int argc, char *argv[])
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file inputcloud \n");
        return (-1);
    }

    // 2. 估计点云的法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(0.03); // 设置搜索半径
    ne.compute(*normals);     // 计算点云法向量

    // 合并点云和法线
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    // 定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals); // 点云搜索树

    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setNormalConsistency(false);

    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    // gp3.setInputNormals(normals);
    gp3.reconstruct(triangles);

    pcl::PointCloud<PointT>::Ptr internal_cloud(new pcl::PointCloud<PointT>);
    // 定义内部填充的边界范围
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    // 输出最小和最大点
    std::cout << "Min point: (" << min_pt[0] << ", " << min_pt[1] << ", " << min_pt[2] << ")" << std::endl;
    std::cout << "Max point: (" << max_pt[0] << ", " << max_pt[1] << ", " << max_pt[2] << ")" << std::endl;

    // 定义蒙特卡罗采样的次数
    int num_samples = 10000;
    // 使用蒙特卡罗方法生成随机点
    srand(time(NULL));
    for (int i = 0; i < num_samples; ++i)
    {
        float x = min_pt[0] + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_pt[0] - min_pt[0])));
        float y = min_pt[1] + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_pt[1] - min_pt[1])));
        float z = min_pt[2] + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max_pt[2] - min_pt[2])));
        pcl::PointXYZ pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;

        // 检查点是否在三角网格内部
        // bool is_inside = false;
        // // 这里需要实现一个函数来检查点是否在三角网格内部
        // // 可以使用射线法、质心法等
        // if (is_inside)
        // {
        //     internal_cloud->push_back(pt);
        // }
    }

    //  Viewer
    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.addPointCloud(cloud, "inputcloud");
    viewer.spin();

    return 0;
}