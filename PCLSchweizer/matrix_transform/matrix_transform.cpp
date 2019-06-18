//#include <iostream>
//
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/point_cloud.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/transforms.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//// This function displays the help如果没有提供预期参数，此函数将显示帮助
//void
//showHelp(char * program_name)
//{
//  std::cout << std::endl;
//  std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
//  std::cout << "-h:  Show this help." << std::endl;
//}
//
//// This is the main function
//int
//main (int argc, char** argv)
//{
//  // Show help
//  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
//    showHelp (argv[0]);
//    return 0;
//  }
//
//
//  // Fetch point cloud filename in arguments | Works with PCD and PLY files
//  //在参数中找.ply或.pcd文件名，如果没有就终止程序。
//  std::vector<int> filenames;
//  bool file_is_pcd = false;
//
//  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
//
//  if (filenames.size () != 1)  {
//    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
//
//    if (filenames.size () != 1) {
//      showHelp (argv[0]);
//      return -1;
//    } else {
//      file_is_pcd = true;
//    }
//  }
//
//  // Load file | Works with PCD and PLY files
//  //现在加载PCD/PLY文件并检测文件是否成功加载，未成功终止程序。
//  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//
//		  if (file_is_pcd) {
//			if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
//			  std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
//			  showHelp (argv[0]);
//			  return -1;
//			}
//		  } else {
//			if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)  {
//			  std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
//			  showHelp (argv[0]);
//			  return -1;
//			}
//  }
//  //================================================================================
//  /* Reminder: how transformation matrices work :
//
//           |-------> This column is the translation
//    | 1 0 0 x |  \
//    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
//    | 0 0 1 z |  /
//    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)
//
//    METHOD #1: Using a Matrix4f
//    This is the "manual" method, perfect to understand but error prone !
//  */
// //生成4维单位方阵，左上三维为旋转，最后一列为平移，最后一行不使用。
//  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//
//  
//  float theta = M_PI/4; // The angle of rotation in radians
//   /* 这里是绕Z轴旋转45度，和X轴上平移:
//            |  cos(θ) -sin(θ)  0.0 |
//        R = |  sin(θ)  cos(θ)  0.0 |
//            |  0.0     0.0     1.0 |
//
//        t = < 2.5, 0.0, 0.0 >
//   */
//  transform_1 (0,0) = 1;
//  transform_1 (0,1) = 2;
//  transform_1(0, 2) = 3;
//  transform_1 (1,0) = 4;
//  transform_1 (1,1) = 5;
//  transform_1(1, 2) =6;
//  transform_1(2, 0) = 8;
//  transform_1(2, 1) = 10;
//  transform_1(2, 2) = 12;
//
//  //    (row, column)
//
//  // Define a translation of 2.5 meters on the x axis.X轴上平移2.5
//    transform_1 (0,3) = 2.5;
//
//  // Print the transformation
//  printf ("Method #1: using a Matrix4f\n");
//  std::cout << transform_1 << std::endl;
//  std::cout << "=======================================================================" << std::endl;
//  std::cout << transform_1.inverse() << std::endl;
////======================================================================================
//  /*  METHOD #2: Using a Affine3f 第二种方法
//    This method is easier and less error prone
//  */
//  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//
//  // Define a translation of 2.5 meters on the x axis.
//  transform_2.translation() << 2.5, 0.0, 0.0;
//
//  // The same rotation matrix as before; theta radians around Z axis
//  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
//
//  // Print the transformation
//  printf ("\nMethod #2: using an Affine3f\n");
//  std::cout << transform_2.matrix() << std::endl;
//  cout << "=====================================" << endl;
//  //transform_2.inverse();
//  cout << transform_2.inverse().matrix() << endl;
//  cout << "==========================================" << endl;
//
//
//
///*
//  Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
//  transform_3.translation() = -transform_2.translation();
//  cout << transform_3.matrix() << endl;
//  cout << "==============================" << endl;
//  cout << transform_3.inverse().matrix() << endl;
//	*/
//
//
////======================================================================================
//  // Executing the transformation
//  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>()); 
//  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
//  // You can either apply transform_1 or transform_2; they are the same 输入、输出、旋转参数
//  pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform_1);
//  pcl::transformPointCloud (*transformed_cloud, *transformed_cloud2, transform_1.inverse());
//
////=====================================================================================
//  // Visualization
//  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
//      "                        red  = transformed point cloud\n");
//  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
//
//   // Define R,G,B colors for the point cloud
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);//白
//  // We add the point cloud to the viewer and pass the color handler
//  std::string src = "E://data//source.pcd";
//  std::string srt2 = "E://data//transformed2.pcd";
//  std::string srt = "E://data//transformed.pcd";
//
//  pcl::io::savePCDFileBinary(src, *source_cloud);
//  pcl::io::savePCDFileBinary(srt2, *transformed_cloud2);
//  pcl::io::savePCDFileBinary(srt, *transformed_cloud);
//  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
//
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud2, 230, 20, 20); // Red
//  viewer.addPointCloud (transformed_cloud2, transformed_cloud_color_handler, "transformed_cloud2");
//
//  viewer.addCoordinateSystem (1.0, "cloud", 0);
//  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud2");
//  //viewer.setPosition(800, 400); // Setting visualiser window position
//
//  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
//    viewer.spinOnce ();
//  }
//
//  return 0;
//}