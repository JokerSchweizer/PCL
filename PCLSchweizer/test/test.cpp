#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <iostream>
#include <vector>
#include <ctime>

int main(int argc, char**argv) {
	std::cout << argc << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::io::loadPLYFile<pcl::PointXYZ>(argv[1],cloud);
	std::cout <<cloud->points[100].x 
			  << cloud->points[100].y 
		      << cloud->points[100]. z
		      << std::endl;

	
	return 0;
}