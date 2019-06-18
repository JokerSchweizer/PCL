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

//����һ���ṹ
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


// �����µĵ��ʾ���� < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    nr_dimensions_ = 4;
  }

  // ��дcopyToFloatArray�������������ǵ���������
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

//��������
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  for (int i = 1; i < argc; i++)
  {
    std::string fname = std::string (argv[i]);
    // �ж��ļ����Ƿ����
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //����Ƿ���.pcd��׺
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // �ѵ������ݼ��ص�ȫ����
      PCD m;
      m.f_name = argv[i];
      pcl::io::loadPCDFile (argv[i], *m.cloud);
      //ȥ��NAN��
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
}


//����Ϊԭ���ơ�Ŀ����ơ�������ơ���ת�����Ƿ��²���
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
 //�Դ������²���
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


  // �������淨�ߺ�����
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

  // ʵ������ṹ
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
	//����*reg_result�Ƕ��������ݽ��
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

	//�������涼��src�ڱ�tgt���䣬����Ҫ���target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);


  //add the source to the transformed target
  *output += *cloud_src;
  
  //final_transform = targetToSource;
  final_transform = Ti;
  cout << "��ת����Ϊ===========================================" << endl;
  cout<< targetToSource<< endl;
  cout << "=====================================================" << endl;

 }


/* ---[ */
int main (int argc, char** argv)
{
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // һ�μ������ݵ�һ��vector�У����ڿ��ܱȽϷѿռ䣬����ʱ�仹���ԡ��Ժ�Ҫ�ǲ������õ�ַÿ������һ֡�ͼ��ء�
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);

  // ����Ƿ����������
  if (data.empty())
  {
	  PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
	  PCL_ERROR("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
	  return (-1);
  }
  PCL_INFO("Loaded %d datasets.\n", (int)data.size());

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //����ȫ�ֵ�����֮��ת������
  //std::vector<Eigen::Matrix4f>transformALL;
  std::vector<std::vector<Eigen::Matrix4f>>transformALL;
  transformALL.resize(argc-1);
  for (size_t i = 0; i < transformALL.size(); i++)
  {
	  transformALL[i].resize(argc-1,Eigen::Matrix4f::Identity());
  }
  for (size_t i = 0; i < transformALL.size(); i++)//�Խ�������Ϊ��λ����
  {
	  transformALL[i][i] = Eigen::Matrix4f::Identity();
  }
  //std::cout << transformALL.size() << std::endl;;

  //����ͼ�����ӹ�ϵ
  std::vector<std::vector<int>>gVec;
  gVec.resize(argc-1);
  for (size_t i = 0; i < gVec.size(); i++)
  {
	  gVec[i].resize(argc-1);
  }
  for (size_t i = 0; i < gVec.size(); i++)//�Խ������ҹ�ϵ
  {
	  gVec[i][i] = 1;
  }
  //cout << gVec.size() << endl;

  //���� ����Ϣ�洢
  std::vector<std::vector<int>>loopInf;

  //�������Ż�����ת����
  Eigen::Matrix4f loopR = Eigen::Matrix4f::Identity();

  //�����־�ж��Ƿ��ڻ��г��ֹ������ǹ�����,��ʼ��ΪFALSE��
  std::vector<bool>loopFlag;
  loopFlag.resize(argc - 1);
  for (size_t i = 0; i < loopFlag.size(); i++)
  {
	  loopFlag[i] = FALSE;
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PointCloud::Ptr result (new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;
  
	//����֮����׼�����ҳ���Ӧ����ת����
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

		  //���ﷵ���˵�pairTransform ��target to source
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
		 // transformALL[i][j] = pairTransform.inverse();//ԭ�Ⱦ���ֱ��������ٺ�ԭ����ʽһ����û��������ת��ƽ�Ƶ��ڲ��������壬�������廹������������ģ��������ɡ�

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
  //�����������֮�����ת����
  for (size_t i = 0; i < transformALL.size(); i++)
  {
	  for (size_t j = 0; j < transformALL[i].size(); j++)
	  {
		  cout << i << "to" << j << endl;
		  cout << transformALL[i][j] << endl;
	  }

  }

  //��������ڽӾ���
  cout << "==============================================����������==========================================================" << endl;
  int count;//�����ߵ��������ж��Ƿ�Ϊ�����㡣
  for (size_t i = 0; i < gVec.size(); i++)
  {
	  count = 0;
	  for (size_t j = 0; j < gVec[i].size(); j++)
	  {
		  cout << gVec[i][j] << " ";
		  if (gVec[i][j]!=0)//������һ
		  {
			  count++;
		  }
	  }
	  if (count<3)//�����С����������ô�Ͳ����ܳ����ڻ�������Լ���һ����
	  {
		  loopFlag[i] = TRUE;
	  }
	  cout << endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	loopInf	=closeLoop(argc, gVec);
	//�����Ƿ��л�����
	cout << "==============================================����������==========================================================" << endl;
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
					loopR *= transformALL[ri - 1][rj - 1];//�������Ķ������꣬����Ҫ�ǵü�һ
				}
				cout << loopInf[i][0];
				cout << endl;
				cout << loopR << endl;
				optimize(loopInf[i], loopR, transformALL);//�����Ż�����
				loopR = Eigen::Matrix4f::Identity();
			}
		}
		if (k > 4)//�����׻����Ľ׻�������������ˣ��Ͳ�Ҫ�Ż���
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