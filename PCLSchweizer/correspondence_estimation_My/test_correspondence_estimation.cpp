/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/io/pcd_io.h>
#include<pcl/io/ply_io.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>


//////////////////////////////////////////////////////////////////////////////////////
void test2 (pcl::PointCloud<pcl::PointXYZ>::Ptr const &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr const &cloud2)
{
  

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ> ());
  tree1->setInputCloud (cloud1);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
  tree2->setInputCloud (cloud2);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, double> ce;
  ce.setInputSource (cloud1);
  ce.setInputTarget (cloud2);
  pcl::Correspondences corr_orig;
  ce.determineCorrespondences(corr_orig);

  ce.setSearchMethodSource (tree1, true);
  ce.setSearchMethodTarget (tree2, true);
  pcl::Correspondences corr_cached;
  ce.determineCorrespondences (corr_cached);

  std::cout << corr_orig.size() << "===" << corr_cached.size() << std::endl;
  for(size_t i = 0; i < corr_orig.size(); i++)
  {
	  std::cout <<"=================================================" << std::endl;
	  std::cout << i << std::endl;
	  std::cout  << corr_orig[i].index_query << "<->"  << corr_orig[i].index_match  << std::endl;
	  std::cout << corr_cached[i].index_query << "<->" <<  corr_cached[i].index_match << std::endl;
  }
  
}

void test(pcl::PointCloud<pcl::PointXYZ>::Ptr const &cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr const &cloud2) {

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;//法线估计
	ne.setInputCloud(cloud1);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//定义kdtree
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud1_normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(1);
	ne.compute(*cloud1_normals); 

	pcl::CorrespondencesPtr corr(new pcl::Correspondences);
	pcl::registration::CorrespondenceEstimationNormalShooting <pcl::PointXYZ, pcl::PointXYZ, pcl::Normal> ce;
	ce.setInputSource(cloud1);
	ce.setKSearch(1);
	ce.setSourceNormals(cloud1_normals);
	ce.setInputTarget(cloud2);
	ce.determineCorrespondences(*corr);

	//根据定义的数据，对应指标应为1 <-> 1,2 <-> 2,3 <-> 3等
	for (unsigned int i = 0; i < corr->size(); i++)
	{
		std::cout << (*corr)[i].index_query << "<->" << (*corr)[i].index_match << std::endl;
	}
	std::cout << corr->size() << std::endl;

}


/* ---[ */
int
  main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>cloud_src, cloud_tgt;
	if (pcl::io::loadPLYFile(argv[1], cloud_src) < 0)
	{
		std::cerr << "Failed to read test file. Please download `bun0.pcd` and pass its path to the test." << std::endl;
		return (-1);
	}
	if (pcl::io::loadPLYFile(argv[2], cloud_tgt) < 0)
	{
		std::cerr << "Failed to read test file. Please download `bun4.pcd` and pass its path to the test." << std::endl;
		return (-1);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
	cloud_source_ptr = cloud_src.makeShared();
	cloud_target_ptr = cloud_tgt.makeShared();
	test(cloud_source_ptr,cloud_target_ptr);
	return 0;
}
/* ]--- */
