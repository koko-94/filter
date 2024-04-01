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
    pcl::PolygonMesh model;
    pcl::ConvexHull<pcl::PointXYZ> convex_Hull;
    convex_Hull.setInputCloud(cloud);
    convex_Hull.setDimension(3);
    convex_Hull.setComputeAreaVolume(true);
    convex_Hull.reconstruct(*cloud2, polygons);
    volume = convex_Hull.getTotalVolume();
    area = convex_Hull.getTotalArea();
    std::cout
        << "体积: " << volume << std::endl;
    std::cout
        << "面积: " << area << std::endl;
    cout << "surface size :" << cloud2->size() << endl;

    // 凹包
    pcl::ConcaveHull<PointT> concave_Hull;
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