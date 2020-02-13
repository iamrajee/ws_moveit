#include "cylinder.h"


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
  pcl::PCDWriter writer;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*input, *cloud);

  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1);
  pass.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

  // writer.write ("cloud_filtered0.pcd", *cloud_filtered, false);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
  writer.write ("cloud_plane.pcd", *cloud_plane, false);

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);

  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // writer.write ("cloud_filtered2.pcd", *cloud_filtered2, false);


  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
	  writer.write ("cloud_cylinder.pcd", *cloud_cylinder, false);
  }

  // AddCylinderParams* cylinder_params;
  cylinder_parameters.radius = coefficients_cylinder->values[6];
  extractLocationHeight(cloud_cylinder, &cylinder_parameters);

  Eigen::Vector3f obj_centroid;
  obj_centroid(0) = camera_position[0] - cylinder_parameters.center_pt[0];
  obj_centroid(1) = camera_position[1] + cylinder_parameters.center_pt[2];
  obj_centroid(2) = (camera_position[2] - cam_Y_Offset) - cylinder_parameters.center_pt[1];
  
  object_centroid.clear();
  object_centroid.push_back(obj_centroid(0));
  object_centroid.push_back(obj_centroid(1));
  object_centroid.push_back(obj_centroid(2));

  dimension.clear();
  dimension.push_back(cylinder_parameters.radius); 
  dimension.push_back(cylinder_parameters.height);


  std::cout << "Centroid: (" << object_centroid[0] << ", " << object_centroid[1] << ", " << object_centroid[2] 
            << ")\nDimension: (" << dimension[0] << ", " << dimension[1] << ")" << std::endl;

}

int main (int argc, char** argv)
{
  ros::Publisher pub_dim;
  ros::Publisher pub_cen;

  camera_position[0] = atof(argv[2]); 
  camera_position[1] = atof(argv[3]);
  camera_position[2] = atof(argv[4]);

  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  pub_dim = nh.advertise<std_msgs::Float32MultiArray>("dimension", 10);
  pub_cen = nh.advertise<std_msgs::Float32MultiArray>("centroid", 10);

  std::cout << "PCL_VERSION: " << PCL_VERSION << std::endl;

  std_msgs::Float32MultiArray msg_cen, msg_dim;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    msg_dim.data.clear();
    msg_cen.data.clear();
    
    msg_dim.data = dimension;
    pub_dim.publish(msg_dim);

    msg_cen.data = object_centroid;
    pub_cen.publish(msg_cen);
   
    ros::spinOnce();
    loop_rate.sleep();
  }
}


void extractLocationHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, AddCylinderParams* cylinder_params)
  {
    double max_angle_y = 0.0;
    double min_angle_y = std::numeric_limits<double>::infinity();

    double lowest_point[3];
    double highest_point[3];

    // Loop over the entire pointcloud.
    for( int i=0; i<cloud->points.size(); i++)
    {
      /* Find the coordinates of the highest point */
      if(atan2(cloud->points[i].z, cloud->points[i].y) < min_angle_y)
      {
        min_angle_y = atan2(cloud->points[i].z, cloud->points[i].y);
        lowest_point[0] = cloud->points[i].x;
        lowest_point[1] = cloud->points[i].y;
        lowest_point[2] = cloud->points[i].z;
      }

      /* Find the coordinates of the lowest point */
      else if (atan2(cloud->points[i].z, cloud->points[i].y) > max_angle_y)
      {
        max_angle_y = atan2(cloud->points[i].z, cloud->points[i].y);
        highest_point[0] = cloud->points[i].x;
        highest_point[1] = cloud->points[i].y;
        highest_point[2] = cloud->points[i].z;
      }
    }

    /* Store the center point of cylinder */
    cylinder_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
    cylinder_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
    cylinder_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2 + cylinder_params->radius;
    /* Store the height of cylinder */
    cylinder_params->height =
        sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
             pow((lowest_point[2] - highest_point[2]), 2));

    // std::cout << "cylinder_params: Center: (" << cylinder_params->center_pt[0] << ", " << cylinder_params->center_pt[1] << ", "
    //           << cylinder_params->center_pt[2] << ") , \n Radius: " << cylinder_params->radius 
    //           << ", height: "  << cylinder_params->height << "\nHigh: (" << highest_point[0] << ", " 
    //           << highest_point[1] << ", " << highest_point[2] << ")\nLow: (" << lowest_point[0] 
    //           << ", " << lowest_point[1] << ", " << lowest_point[2] << ")" << std::endl;

  }

