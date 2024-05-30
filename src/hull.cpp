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
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d.h> //特征库

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

pcl::PolygonMesh bossion_surface(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int Search)
{
    //-------------法线估计--------------------------- , PlaneParameter P
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;                         // 法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>); // 存储估计的法线
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(Search);
    n.compute(*normals);
    // for (int i = 0; i < normals->size(); i++)
    // {
    //     if (normals->points[i].normal_x * P.A + normals->points[i].normal_y * P.B + normals->points[i].normal_z * P.C < 0)
    //     {
    //         normals->points[i].normal_x *= -1.0;
    //         normals->points[i].normal_y *= -1.0;
    //         normals->points[i].normal_z *= -1.0;
    //     }
    // }
    //------------连接法线和坐标-----------------------
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //------------泊松重建-----------------------------
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    // pcl::io::savePCDFileASCII("C:/Users/12875/Desktop/数据/point_cloud_volume/normals.pcd", *cloud_with_normals);
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setSearchMethod(tree2);
    pn.setDegree(2); // 设置参数degree[1,5],值越大越精细，耗时越久。
    pn.setInputCloud(cloud_with_normals);
    pn.setDepth(6); // 设置将用于表面重建的树的最大深度
    pn.setMinDepth(2);
    pn.setScale(1.0); // 设置用于重建的立方体的直径与样本的边界立方体直径的比值

    pn.setIsoDivide(3);        // 设置块等表面提取器用于提取等表面的深度
    pn.setSamplesPerNode(3.0); // 设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
    // pn.setSamplesPerNode(12);//设置每个八叉树节点上最少采样点数目
    // pn.setSolverDivide(3);//设置块高斯-塞德尔求解器用于求解拉普拉斯方程的深度。
    pn.setSolverDivide(8);       // 设置求解线性方程组的Gauss-Seidel迭代方法的深度
    pn.setConfidence(false);     // 设置置信标志，为true时，使用法线向量长度作为置信度信息，false则需要对法线进行归一化处理
    pn.setManifold(false);       // 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
    pn.setOutputPolygons(false); // 设置是否输出为多边形(而不是三角化行进立方体的结果)。

    //----------------保存重建结果---------------------
    pcl::PolygonMesh mesh;
    pn.performReconstruction(mesh);
    //    pcl::io::savePLYFile(file_surface, mesh);
    return mesh;
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
    pcl::PolygonMesh model;

    // model = bossion_surface(cloud, 24);

    computeConvexHull(cloud, cloud2, polygons);
    // 凹包
    pcl::ConcaveHull<PointT> concave_Hull;
    concave_Hull.setInputCloud(cloud);
    concave_Hull.setAlpha(0.1);
    concave_Hull.reconstruct(*cloud_concave, polygons);
    //  Viewer
    pcl::visualization::PCLVisualizer viewer("viewer");

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_convex(cloud2, 255, 0, 0);
    // viewer.addPointCloud(cloud2, color_convex, "point");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");

    // viewer.addPointCloud(cloud, "inoputcloud");
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_concave(cloud2, 0, 255, 0);
    // viewer.addPointCloud(cloud_concave, color_concave, "point2");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");
    viewer.addPolygonMesh(model);
    // viewer.addPolygonMesh<pcl::PointXYZ>(cloud_concave, "concave_hull");

    viewer.spin();

    return 0;
}