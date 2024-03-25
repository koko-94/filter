/**************************************************************************

Author: SHR

Date:2024.3.24

FileName: range_image.cpp

Function:将点云转化我深度图像，并进行可视化，方便通过深度图像进行数据处理
**************************************************************************/

#include <iostream>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>