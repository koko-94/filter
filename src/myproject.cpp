/**************************************************************************

Author: SHR

Date:2024.3.24

FileName: correspondence.cpp

Function:三维物体识别
**************************************************************************/
#include <pcl/io/pcd_io.h>   //io输入输出库
#include <pcl/point_cloud.h> //基本点云类库
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

// Algorithm params
bool show_keypoints_(false);
bool show_correspondences_(true);
bool use_cloud_resolution_(false);
bool use_hough_(true);
float model_ss_(0.01f);
float scene_ss_(0.03f);
float rf_rad_(0.015f);
float descr_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);

int main(int argc, char *argv[])
{

    pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());           // 模型
    pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>()); // 模型特征点
    pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());           // 场景
    pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>()); // 场景特征点
    pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>()); // 模型特征向量
    pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>()); // 场景特征向量
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

    //  Load clouds
    //
    if (pcl::io::loadPCDFile("../../data/new/line_zhixian.pcd", *model) < 0)
    // if (pcl::io::loadPCDFile("../../data/milk.pcd", *model) < 0)
    {
        std::cout << "Error loading model cloud." << std::endl;
        return (-1);
    }
    else
        std::cout << "read model success." << std::endl;
    if (pcl::io::loadPCDFile("../../data/new/whole_line.pcd", *scene) < 0)
    // if (pcl::io::loadPCDFile("../../data/milk_cartoon_all_small_clorox.pcd", *scene) < 0)
    {
        std::cout << "Error loading scene cloud." << std::endl;
        return (-1);
    }
    else
        std::cout << "read scene success." << std::endl;

    // 1 估计场景云的法线
    pcl::NormalEstimation<PointType, NormalType> norm_est;
    // norm_est.setKSearch(20);
    norm_est.setRadiusSearch(5); // 利用半径为3cm的球体中的所有邻点进行估计
    norm_est.setInputCloud(model);
    norm_est.compute(*model_normals);

    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);

    // 2 对每个云进行降采样以找到少量关键点
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(model);
    uniform_sampling.setRadiusSearch(2.0f); // 搜索半径
    // uniform_sampling.setRadiusSearch(0.01f);   // 搜索半径
    uniform_sampling.filter(*model_keypoints); // 存储降采样后的点云
    std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(2.0f);
    // uniform_sampling.setRadiusSearch(0.03f);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

    // 3 将关键点关联到SHOT特征描述符以执行模型aa和场景的关键点匹配并确定点对点对应关系。
    pcl::SHOTEstimation<PointType, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch(descr_rad_);

    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(model);
    descr_est.compute(*model_descriptors);

    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    // descr_est.setSearchSurface(scene);
    descr_est.setSearchSurface(scene);
    descr_est.compute(*scene_descriptors);

    // 基于欧氏距离找最相似的描述符，确定模型描述符和场景描述符之间的点对点对应关系
    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (std::size_t i = 0; i < scene_descriptors->size(); ++i) // 遍历场景描述子
    {
        std::vector<int> neigh_indices(1);                          // 初始化最近索引
        std::vector<float> neigh_sqr_dists(1);                      // 初始化最近平方距离
        if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) // skipping NaNs跳过非有限描述子
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists); // 搜索最近邻
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)                                                         //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

    // 根据描述符的对应关系进行聚类。
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize(cg_size_);
    gc_clusterer.setGCThreshold(20.0f); // min_num

    gc_clusterer.setInputCloud(model_keypoints);
    gc_clusterer.setSceneCloud(scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

    gc_clusterer.recognize(rototranslations, clustered_corrs);

    std::cout << "Model instances found: " << rototranslations.size() << std::endl;

    for (std::size_t i = 0; i < rototranslations.size(); ++i)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

        printf("\n");
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        printf("\n");
        printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
    }

    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(scene);
    viewer.addPointCloud(scene, rgb, "scene_cloud");

    pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
    pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
    viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
    viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
    for (std::size_t i = 0; i < rototranslations.size(); ++i)
    {
        pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
        viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

        for (std::size_t j = 0; j < clustered_corrs[i].size(); ++j)
        {
            std::stringstream ss_line;
            ss_line << "correspondence_line" << i << "_" << j;
            PointType &model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
            PointType &scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

            //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
            viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
        }
    }
    viewer.spin();

    return (0);
}
