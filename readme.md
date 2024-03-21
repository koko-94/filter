1.pcl可视化 
使用pcl::visualization::PCLVisualizer函数，
不用pcl::visualization::CloudViewer，这个函数有bug可视化窗口无法保持。
PCLVisualizer 先创建点云变量，导入点云，设置可视化变量，导入点云数据，设置相关参数，添加进窗口，阻塞显示。

2.滤波器
（1）passthrough直通滤波器，过滤某一个轴某一范围内数据。
（2）VoxelGird体素滤波，将某个区域的点云以重心表示为一个点云，减少数据量。
（3）SOR滤波器，根据每个点与其邻居之间的平均距离，去除标准差之外的点。
（4）ROR滤波器，设定一个搜索半径和一个邻近点数量的阈值，少于阈值即去除。

3.法向量提取
根据kdtree提取点云模型法向量。

4.RANSAC算法拟合模型
使用RANSAC算法拟合球体，平面等模型，去除模型之外的点云。