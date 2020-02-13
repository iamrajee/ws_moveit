#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include "std_msgs/Float32MultiArray.h"


std::vector<float> object_centroid, dimension, rpy;


typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;

float getOrientationPosDimension(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, float box_dim[]);
void getDimension(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, Eigen::Vector4f pcaCentroid, 
                  Eigen::Matrix3f eigenVectorsPCA,float box_dim[]);

