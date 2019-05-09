#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
int
  main (int argc, char** argv)
{
    //创建模板化PointCloud结构，每个点的类型被设定为pcl::PointXYZ
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data 随机点填充PointCloud结构
  cloud.width    = 5;
  cloud.height   = 2;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
//将PointCloud数据保存到磁盘上，名为 test_pcd.pcd.
  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

  for (size_t i = 0; i < cloud.points.size (); ++i)
    std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
  std::cout <<"====================================================" << std::endl;
  std::cout << cloud <<std::endl;
  
  return (0);
}