#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include"E:\Schweizer\PCLBuild\globalRegistration\closeLoop.h"
#include "E:\Schweizer\PCLBuild\globalRegistration\optimize.h"

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//声明一个结构
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// 定义新的点表示方法 < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    nr_dimensions_ = 4;
  }

  // 覆写copyToFloatArray方法来定义我们的特征向量
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

//载入数据
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // 判断文件名是否合理
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //检查是否是.pcd后缀
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // 把点云数据加载到全局中
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //去除NAN点
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}


//参数为原点云、目标点云、输出点云、旋转矩阵、是否下采样
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
 //对大数据下采样
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.5, 0.5, 0.5);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // 计算曲面法线和曲率
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  // 实力化点结构
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  
  reg.setMaxCorrespondenceDistance (0.6);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;

  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (100);
  for (int i = 0; i <100; ++i)
  {
   // PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
	//这里*reg_result是对齐后的数据结果
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();
  }

	//由于上面都是src在边tgt不变，所以要变成target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);


  //add the source to the transformed target
  *output += *cloud_src;
  
  //final_transform = targetToSource;
  final_transform = Ti;
  cout << "旋转矩阵为===========================================" << endl;
  cout<< targetToSource<< endl;
  cout << "=====================================================" << endl;

 }


/* ---[ */
int main (int argc, char** argv)
{
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // 一次加载数据到一个vector中，现在可能比较费空间，但是时间还可以。以后要是不行再用地址每次用哪一帧就加载。
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);

  // 检查是否加载了数据
  if (data.empty())
  {
	  PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
	  PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
	  return (-1);
  }
  PCL_INFO("Loaded %d datasets.\n", (int)data.size());

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //定义全局的两两之间转换矩阵
  //std::vector<Eigen::Matrix4f>transformALL;
  std::vector<std::vector<Eigen::Matrix4f>>transformALL;
  transformALL.resize(argc-1);
  for (size_t i = 0; i < transformALL.size(); i++)
  {
	  transformALL[i].resize(argc-1,Eigen::Matrix4f::Identity());
  }
  for (size_t i = 0; i < transformALL.size(); i++)//对角线自我为单位矩阵
  {
	  transformALL[i][i] = Eigen::Matrix4f::Identity();
  }
  //std::cout << transformALL.size() << std::endl;;

  //定义图的连接关系
  std::vector<std::vector<int>>gVec;
  gVec.resize(argc-1);
  for (size_t i = 0; i < gVec.size(); i++)
  {
	  gVec[i].resize(argc-1);
  }
  for (size_t i = 0; i < gVec.size(); i++)//对角线自我关系
  {
	  gVec[i][i] = 1;
  }
  //cout << gVec.size() << endl;

  //定义 环信息存储
  std::vector<std::vector<int>>loopInf;

  //定义沿着环后旋转积累
  Eigen::Matrix4f loopR = Eigen::Matrix4f::Identity();

  //定义标志判断是否在环中出现过或者是孤立点,初始化为FALSE。
  std::vector<bool>loopFlag;
  loopFlag.resize(argc - 1);
  for (size_t i = 0; i < loopFlag.size(); i++)
  {
	  loopFlag[i] = FALSE;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  
	//两两之间配准，并找出对应的旋转矩阵
	//for (size_t i = 1; i < data.size(); ++i)
  for (size_t i = 0; i < data.size (); ++i)
  {
	  //for (size_t j = 0; j < i; j++)
	  for (size_t j = 0; j < data.size(); j++)
	  {
		  source = data[j].cloud;
		  target = data[i].cloud;

		  PointCloud::Ptr temp(new PointCloud);
		  PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[j].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());

		  //这里返回了的pairTransform 是target to source
		  pairAlign(source, target, temp, pairTransform, true);

		  ////transform current pair into the global transform
		  //pcl::transformPointCloud(*temp, *result, GlobalTransform);

		  ////update the global transform
		  //GlobalTransform *= pairTransform;

		  //std::cout << "=================" << endl;
		  ////save aligned pair, transformed into the first cloud's frame
		  //std::stringstream ss;
		  //ss << i <<"_"<<j<< ".pcd";
		  //pcl::io::savePCDFile(ss.str(), *result, true);
		  cout << i << "  to  " << j << endl;
		  cout << pairTransform << endl;
		  
		  transformALL[j][i] = pairTransform;
		 // transformALL[i][j] = pairTransform.inverse();//原先觉得直接求逆后不再和原来格式一样，没有那种旋转和平移的内部物理意义，但是整体还是有物理意义的，就这样吧。

		  if (pairTransform!=Eigen::Matrix4f::Identity())
		  {
			 // gVec[i][j] = gVec[j][i] = 1;
			  gVec[j][i] = 1;
		  }
		  else {
			  //gVec[i][j] = gVec[j][i] = 0;
			  gVec[j][i] = 0;
		  }
		  cout << "========================================================================================================" << endl;


	  }
  }
  //测试输出两两之间的旋转矩阵
  for (size_t i = 0; i < transformALL.size(); i++)
  {
	  for (size_t j = 0; j < transformALL[i].size(); j++)
	  {
		  cout << i << "to" << j << endl;
		  cout << transformALL[i][j] << endl;
	  }

  }

  //测试输出邻接矩阵
  cout << "==============================================在主程序里==========================================================" << endl;
  int count;//计数边的条数来判断是否为孤立点。
  for (size_t i = 0; i < gVec.size(); i++)
  {
	  count = 0;
	  for (size_t j = 0; j < gVec[i].size(); j++)
	  {
		  cout << gVec[i][j] << " ";
		  if (gVec[i][j]!=0)//边数加一
		  {
			  count++;
		  }
	  }
	  if (count<3)//如果边小于三条，那么就不可能出现在环里（其中自己有一条）
	  {
		  loopFlag[i] = TRUE;
	  }
	  cout << endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	loopInf	=closeLoop(argc, gVec);
	//测试是否有环回来
	cout << "==============================================在主程序里==========================================================" << endl;
	int size = loopInf.size();


	for (size_t k = 3; k < 100; k++)
	{
		for (size_t i = 0; i < loopInf.size(); i++)
		{
			if (loopInf[i].size()==k)
			{
				for (size_t j = 0; j < loopInf[i].size()-1; j++)
				{
					cout << loopInf[i][j] << "->";
					int ri = loopInf[i][j];
					loopFlag[ri - 1] = TRUE;
					int rj = loopInf[i][j + 1];
					//cout << transformALL[ri-1][rj-1] << endl;;
					loopR *= transformALL[ri - 1][rj - 1];//返回来的都是坐标，所以要记得减一
				}
				cout << loopInf[i][0];
				cout << endl;
				cout << loopR << endl;
				optimize(loopInf[i], loopR, transformALL);//调用优化函数
				loopR = Eigen::Matrix4f::Identity();
			}
		}
		if (k > 4)//在三阶环和四阶环后，如果都出现了，就不要优化了
		{
			bool F = TRUE;
			for (size_t i = 0; i < loopFlag.size(); i++)
			{
				if (loopFlag[i] == FALSE)
				{
					F = FALSE;
				}
			}
			if (F)
			{
				break;
			}
		}
	}

	for (size_t i = 0; i < data.size(); i++)
	{
		pcl::transformPointCloud(*data[i].cloud, *result, transformALL[0][i]);
		std::stringstream ss;
		ss << i <<"_"<< ".pcd";
		pcl::io::savePCDFile(ss.str(), *result, true);
	}
	

  system("pause");
}
/* ]--- */