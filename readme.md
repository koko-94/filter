一.简易cpp文件功能介绍
1.main.cpp  pcl可视化 
使用pcl::visualization::PCLVisualizer函数，
不用pcl::visualization::CloudViewer，这个函数有bug可视化窗口无法保持。
PCLVisualizer 先创建点云变量，导入点云，设置可视化变量，导入点云数据，设置相关参数，添加进窗口，阻塞显示。

2.filter.cpp 滤波器
（1）passthrough直通滤波器，过滤某一个轴某一范围内数据。
（2）VoxelGird体素滤波，将某个区域的点云以重心表示为一个点云，减少数据量。
（3）SOR滤波器，根据每个点与其邻居之间的平均距离，去除标准差之外的点。
（4）ROR滤波器，设定一个搜索半径和一个邻近点数量的阈值，少于阈值即去除。

3.normalvector_estimate.cpp 法向量提取
根据kdtree提取点云模型法向量。
贪婪三角化模型对点云进行三角化

4.segementation.cpp RANSAC算法拟合模型
使用RANSAC算法拟合球体，平面等模型，去除模型之外的点云。

5.montecarlo.cpp 蒙特卡罗法
蒙特卡罗法填充点云表面模型内部点

6.subtraction.cpp 点云比较
将两副点云模型进行比较，得到损伤模型

7.hull.cpp 凸包法
凸包法得到凸点对图模型进行重建与数据计算

8.cylinder.cpp 圆柱模型拟和
从点云中提去出圆柱型点云

9.cal_mesh.cpp 三角化
三角化可视化重建模型

10.correspondence.cpp 三维物体识别

二.工程使用
本工程主要为进行三维结构物进行损伤模型的损伤重建，并对损伤属性进行计算。
1.首先使用filter.cpp进行滤波处理
2.然后使用RANSAC算法拟合模型去除平面模型
3.然后聚类分割出目标模型（在segementation.cpp中实现）
4.调用subtraction.cpp进行点云相减得到损伤模型
5.使用hull.cpp进行凸包法计算损伤数据。
