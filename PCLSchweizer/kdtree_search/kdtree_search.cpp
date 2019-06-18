#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char**argv) {
	srand(time(NULL));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	//点云生成
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f* rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f* rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f* rand() / (RAND_MAX + 1.0f);
	}
	//创建kdtree对象
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	kdtree.setInputCloud(cloud);//设置搜索空间
	pcl::PointXYZ searchPoint;//定义查询点并随机赋值
	searchPoint.x = 1024.0f* rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f* rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f* rand() / (RAND_MAX + 1.0f);
	// k近邻搜索
	int K = 10;
	std::vector<int>pointIdxNKNSearch(K);//存储查询点近邻索引
	std::vector<float>pointNKNSquaredDistance(K);//存储近邻点对应平方距离
	std::cout << "K nearest neighbor search at (" << searchPoint.x//打印相关信息
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with K=" << K << std::endl;

	if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)//查询临近的K个点，并保存信息
	{
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	// 在半径r内搜索近邻
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = 256.0f* rand() / (RAND_MAX + 1.0f);
	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;
	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
	return 0;
}