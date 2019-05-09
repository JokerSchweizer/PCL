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

#include <pcl/visualization/pcl_visualizer.h>



using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
//==========================================================================================================
//可视化全局变量创建
// This is a tutorial so we can afford having global variables 
	//our visualizer
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;
//==========================================================================================================
//声明一个结构，方便处理点云对
//convenient structure to handle our pointclouds
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
//==========================================================================================================
//定义新的点表示
// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    //定义维度
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

//==========================================================================================================
////////////////////////////////////////////////////////////////////////////////
/** 在可视化程序的第一个视图端口上显示源和目标
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
 // p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/**  在可视化程序的第二个视图端口上显示源和目标
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

 // p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  //假设第一个参数是实际的测试模型
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    //至少需要5个长度，这里判断是否文件名合理
    if (fname.size () <= extension.size ())
      continue;

	/**下面是把文件名全部变小写？？？进行(int(*)(int))tolower操作
	*	std::vector<double> deg_C {21.0, 30.5, 0.0, 3.2, 100.0};
		std::vector<double> deg_F(deg_C.size());
		std::transform(std::begin(deg_C), std::end(deg_C), std:rbegin(deg_F),[](double temp){ return 32.0 + 9.0*temp/5.0; });
		//Result 69.8 86.9 32 37.76 212
	*/
    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //检查参数是否是pcd文件，三个参数是开始地址、结束地址、和对比对象
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // 加载云并将其保存到模型的全局列表中
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //从云中移除NAN点
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
   //向下采样
	  PointCloud::Ptr src (new PointCloud);
	  PointCloud::Ptr tgt (new PointCloud);
	  pcl::VoxelGrid<PointT> grid;
	  if (downsample)
	  {
			grid.setLeafSize (0.01, 0.01, 0.01);
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

	  //
	  // 实例化我们的自定义点表示(上面定义的)…
	  MyPointRepresentation point_representation;
	  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	  point_representation.setRescaleValues (alpha);

	  //
	  // Align
	  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
	  reg.setTransformationEpsilon (1e-6);
	  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
	  // Note: adjust this based on the size of your datasets
	  reg.setMaxCorrespondenceDistance (0.1);  
	  // Set the point representation
	  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	  reg.setInputSource (points_with_normals_src);
	  reg.setInputTarget (points_with_normals_tgt);


	  //
	  // Run the same optimization in a loop and visualize the results
	  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
	  reg.setMaximumIterations (100);

	  for (int i = 0; i < 100; ++i)
	  {
			PCL_INFO ("Iteration Nr. %d.\n", i);

			// 保存点云用来可视化
			points_with_normals_src = reg_result;

			// 估计
			reg.setInputSource (points_with_normals_src);
			reg.align (*reg_result);

			//在每次迭代之间积累转换
			Ti = reg.getFinalTransformation () * Ti;

				//if the difference between this transformation and the previous one
				//is smaller than the threshold, refine the process by reducing
				//the maximal correspondence distance
			if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			  reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
			prev = reg.getLastIncrementalTransformation ();

			// visualize current 右侧显示
			showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	  }

		//
	  // Get the transformation from target to source
	  targetToSource = Ti.inverse();

	  //
	  // Transform target back in source frame
	  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

	  p->removePointCloud ("source");
	  p->removePointCloud ("target");

	  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
	  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
	  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
	  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	  PCL_INFO ("Press q to continue the registration.\n");
	 // p->spin ();

	  p->removePointCloud ("source"); 
	  p->removePointCloud ("target");

	  //add the source to the transformed target
	  *output += *cloud_src;
  
	  final_transform = targetToSource;
	  cout << "=============================================================================================" << endl;
	  cout << targetToSource << endl;
	  cout << "=============================================================================================" << endl;
 }


/* ---[ */
int main (int argc, char** argv)
{
	  // Load data
	  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
	  loadData (argc, argv, data);

	  // Check user input
	  if (data.empty ())
	  {
			PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
			PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
			return (-1);
	  }
	  PCL_INFO ("Loaded %d datasets.", (int)data.size ());
  
	  // Create a PCLVisualizer object
	  /*
	  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
	  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);
	  */
		PointCloud::Ptr result (new PointCloud), source, target;
	  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  
	  for (size_t i = 1; i < data.size (); ++i)
	  {
		  cout << i << endl;
		source = data[i-1].cloud;
		target = data[i].cloud;

		// Add visualization data
		showCloudsLeft(source, target);

		PointCloud::Ptr temp (new PointCloud);
		PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str (), source->points.size (), data[i].f_name.c_str (), target->points.size ());
		//pairAlign(source, target, temp, pairTransform);
		pairAlign (source, target, temp, pairTransform, true);

		//transform current pair into the global transform
		pcl::transformPointCloud (*temp, *result, GlobalTransform);

		//update the global transform
		GlobalTransform = GlobalTransform * pairTransform;

			//save aligned pair, transformed into the first cloud's frame
		std::stringstream ss;
		ss << i << ".pcd";
		pcl::io::savePCDFile (ss.str (), *result, true);

	  }
}
/* ]--- */