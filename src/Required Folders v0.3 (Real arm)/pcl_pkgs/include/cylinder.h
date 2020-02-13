#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include "std_msgs/Float32MultiArray.h"



float camera_position[3] = {0};
float object_dimension[2] = {0};
float cam_Y_Offset = 0.032;
std::vector<float> object_centroid, dimension;

typedef pcl::PointXYZ PointT;
struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector towards the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };

  AddCylinderParams cylinder_parameters;

void extractLocationHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, AddCylinderParams* cylinder_params);
