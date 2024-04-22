#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>
#include <cmath>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>                           //pcl输入输出库
#include <pcl/filters/voxel_grid.h>                  //体素滤波库
#include <pcl/visualization/pcl_visualizer.h>        //可视化库
#include <pcl/filters/statistical_outlier_removal.h> //离群值滤波库
#include <pcl/filters/radius_outlier_removal.h>      //离群值滤波器
#include <pcl/surface/mls.h>                         //最小二乘法平滑滤波
#include <pcl/features/normal_3d.h>                  //特征库
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>

typedef pcl::PointXYZ PointT;
float radius = 0.1;

void voxelfilter(float size, const pcl::PointCloud<PointT>::Ptr &inputcloud, const pcl::PointCloud<PointT>::Ptr &outputcloud)
{
    pcl::VoxelGrid<PointT> sor; // 设置滤波器
    sor.setInputCloud(inputcloud);
    sor.setLeafSize(size, size, size); // 设置1m3的立方体
    sor.filter(*outputcloud);          // 保存滤波后点云

    // 计算点云数量进行比较
    std::cerr << "PointCloud before voxelfilter: " << inputcloud->width * inputcloud->height
              << " data points (" << pcl::getFieldsList(*inputcloud) << ")." << std::endl;
    std::cerr << "PointCloud after voxelfilter: " << outputcloud->width * outputcloud->height
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

// inline float signedVolumeOfTriangle(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
// {
//     float v321 = p3.x * p2.y * p1.z;
//     float v231 = p2.x * p3.y * p1.z;
//     float v312 = p3.x * p1.y * p2.z;
//     float v132 = p1.x * p3.y * p2.z;
//     float v213 = p2.x * p1.y * p3.z;
//     float v123 = p1.x * p2.y * p3.z;
//     return (1.0f / 6.0f) * (-v321 + v231 + v312 - v132 - v213 + v123);
// }

// float volumeOfMesh(pcl::PolygonMesh mesh)
// {
//     float vols = 0.0;
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
//     for (int triangle = 0; triangle < mesh.polygons.size(); triangle++)
//     {
//         pcl::PointXYZ pt1 = cloud->points[mesh.polygons[triangle].vertices[0]];
//         pcl::PointXYZ pt2 = cloud->points[mesh.polygons[triangle].vertices[1]];
//         pcl::PointXYZ pt3 = cloud->points[mesh.polygons[triangle].vertices[2]];
//         vols += signedVolumeOfTriangle(pt1, pt2, pt3);
//     }
//     return abs(vols);
// }

int main(int argc, char **argv)
{
    // pcl::PolygonMesh mesh;
    // pcl::io::loadOBJFile(argv[1], mesh);
    // std::cout << volumeOfMesh(mesh) << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Voxelfiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_RORfiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_SORfiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file inputcloud \n");
        return (-1);
    }
    // 应用体素滤波器
    voxelfilter(radius, cloud, cloud_Voxelfiltered);
    // ROR
    RORfilter(5, radius * 5, cloud_Voxelfiltered, cloud_RORfiltered);
    // SOR
    // SORfilter(50, 1.0f, cloud_Voxelfiltered, cloud_SORfiltered);

    // 转化
    pcl::copyPointCloud(*cloud_RORfiltered, *cloud_filtered);

    // 法线计算
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(radius * 5);
    ne.compute(*normals);
    // 和并
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_filtered, *normals, *cloud_with_normals);
    // 最小二乘法平滑处理
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud_filtered);
    mls.setSearchMethod(tree);
    mls.setPolynomialOrder(2);       // 2阶
    mls.setSearchRadius(radius * 3); // 用于拟合的K近邻半径。在这个半径里进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。

    // mls.setUpsamplingMethod(RANDOM_UNIFORM_DENSITY); // 增加密度较小区域的密度对于holes的填补却无能为力，具体方法要结合参数使用
    mls.process(*cloud_with_normals);
    pcl::copyPointCloud(*cloud_with_normals, *cloud_filtered);

    std::cerr << "PointCloud points after smooth: " << cloud_with_normals->width * cloud_with_normals->height
              << " data points (" << cloud_with_normals->points.size() << ")." << std::endl;

    // 定义搜索树对象
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals); // 点云搜索树
    //  4. 三角化
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3; // 定义三角化对象
    pcl::PolygonMesh triangles;                               // 定义最后网格模型
    gp3.setInputCloud(cloud_with_normals);                    // 输入
    gp3.setSearchRadius(radius * 15);                         // 搜索半径
    gp3.setMu(12.5);                                          // 设置被样本点搜索其最近邻点的最远距离，为了使用点云密度的变化
    gp3.setMaximumNearestNeighbors(100);                      // 样本点最大可搜索邻域个数
    gp3.setMaximumSurfaceAngle(M_PI / 4);                     // 设置最大表面角度，用于控制三角化结果的平滑程
    gp3.setMinimumAngle(M_PI / 18);                           // 设置最小角度，用于控制三角化结果的细节程度
    gp3.setMaximumAngle(2 * M_PI / 3);                        // 设置最大角度，用于控制三角化结果的细节程度
    // 以上都是默认参数，可以不设置

    gp3.setNormalConsistency(false); //  设置法向一致性，表示是否保持法向一致；如果设置为true，则生成三角化结果时会对法向进行一致性检查和调整

    gp3.setInputCloud(cloud_with_normals); // 设置带有法向信息的输入点云数据
    gp3.setSearchMethod(tree2);            // 设置搜索方法，用于三角化操作时的点云搜索
    gp3.reconstruct(triangles);            // 进行三角化操作，生成三角化结果

    // pcl::io::savePLYFile("output.ply", triangles);

    pcl::visualization::PCLVisualizer viewer("SOR_filter"); // 设置可视化窗口
    // v1 v2是视窗标识符
    // int v1(1);
    // int v2(2);
    // int v3(3);
    // int v4(4);
    // // createViewPort 参数是 double xmin, double ymin, double xmax, double ymax, int &viewport
    // viewer.createViewPort(0, 0, 0.5, 0.5, v1);
    // viewer.createViewPort(0.5, 0, 1, 0.5, v2);
    // viewer.createViewPort(0, 0.5, 0.5, 1, v3);
    // viewer.createViewPort(0.5, 0.5, 1, 1, v4);
    // pcl::visualization::PointCloudColorHandlerRGBField<PointTRGB> rgb(cloud); // rgb 显示本身颜色

    // viewer.addPointCloud(cloud, "cloud", v1);
    // viewer.addPointCloud(cloud_Voxelfiltered, "cloud_Voxelfiltered", v2);
    // viewer.addPointCloud(cloud_SORfiltered, "cloud_SORfiltered", v3);
    // viewer.addPointCloud(cloud_filtered, "cloud_filtered", v4);
    // viewer.addPointCloud(cloud_filtered, "cloud");
    viewer.addPolygonMesh(triangles);
    viewer.spin();

    return 0;
}
