#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
//  pcl::PCLPointCloud2 pcl_pc2;
//  pcl_conversions::toPCL(cloud_msg, pcl_pc2);
//  pcl::PointCloud<pcl::PointXYZ> cloud2;
//  pcl::fromPCLPointCloud2(pcl_pc2, cloud2);
//  pcl::PointCloud<pcl::PointXYZ> cloud2;
//  pcl::fromROSMsg (*cloud_msg, cloud2);
//  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
//  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
//  pcl_conversions::toPCL(*cloud_msg, *cloud);
     //sensor_msgs::PointCloud2 transformed_cloud;
  // Perform the actual filtering
//  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//  sor.setInputCloud (cloudPtr);
//  sor.setLeafSize (0.1, 0.1, 0.1);
//  sor.filter (cloud_filtered);
 //  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
 //   float theta = M_PI/4;
 //   transform_1.translation() << 1, 0, 0.0;
//    transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    
 //   Eigen::Matrix4f M;
 //   M = transform_1.matrix();
 //   pcl_ros::transformPointCloud(M, *cloud_msg, transformed_cloud);
  //  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  //  pcl::fromPCLPointCloud2(*cloud, *pcl_pc);
 //   pcl::transformPointCloud (cloud2, *transformed_cloud, transform_1);
  // Convert to ROS data type
  //pcl::PCLPointCloud2 pcl_pc2;
  //pcl::toROSMsg(transformed_cloud, pcl_pc2)
 // sensor_msgs::PointCloud2 output2;
 // output2 = transformed_cloud;
 // pcl::fromPCLPointCloud2(transformed_cloud, output2);
  //pcl_conversions::fromPCL(pcl_pc2, output2);
  //pcl::toROSMsg(*transformed_cloud, output2);
   pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*cloud_msg, pc);
  for (int i=0; i<pc.size(); i++)
  {
    printf("x %f y %f z %f \n", pc[i].x, pc[i].y, pc[i].z);
  }
  sleep(10);
  // Publish the data
  pub.publish (*cloud_msg);
}
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "transform_ros");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/voxel_transform/output", 1);

  // Spin
  ros::spin ();
}
