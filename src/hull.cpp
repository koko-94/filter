/**************************************************************************

Author: SHR

Date:2024.3.29

FileName: hull.cpp

Function:凸包法计算点云模型体积

**************************************************************************/
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h> //可视化库
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

typedef pcl::PointXYZ PointT;
float area, volume;
std::string model_filename_;

// 计算凸包并返回多边形、体积和面积
int computeConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull,
                      std::vector<pcl::Vertices> &polygons)
{

    float volume;
    float area;
    // 创建凸包对象
    pcl::ConvexHull<pcl::PointXYZ> convex_Hull;
    // 设置输入点云
    convex_Hull.setInputCloud(cloud);
    // 设置维度为3D
    convex_Hull.setDimension(3);
    // 设置计算面积和体积
    convex_Hull.setComputeAreaVolume(true);

    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    // 重建凸包
    convex_Hull.reconstruct(*cloud_hull, polygons);
    // *cloud_hull = *cloud2;

    // 获取体积和面积
    volume = convex_Hull.getTotalVolume();
    area = convex_Hull.getTotalArea();

    // 输出结果
    std::cout << "体积: " << volume << std::endl;
    std::cout << "面积: " << area << std::endl;

    return true;
}

int main(int argc, char *argv[])
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_concave(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1)
    {
        std::cout << "read error " << std::endl;
        return -1;
    }
    // 凸包
    std::vector<pcl::Vertices> polygons; // 保存的是凸包多边形的顶点坐标
    // pcl::PolygonMesh model;

    computeConvexHull(cloud, cloud2, polygons);
    // 凹包
    pcl::ConcaveHull<PointT>
        concave_Hull;
    concave_Hull.setInputCloud(cloud);
    concave_Hull.setAlpha(0.1);
    concave_Hull.reconstruct(*cloud_concave);
    //  Viewer
    pcl::visualization::PCLVisualizer viewer("viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_convex(cloud2, 255, 0, 0);
    viewer.addPointCloud(cloud2, color_convex, "point");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_concave(cloud2, 0, 255, 0);
    viewer.addPointCloud(cloud_concave, color_concave, "point2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");

    viewer.spin();

    return 0;
}