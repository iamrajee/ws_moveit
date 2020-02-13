
#include "cube_dimension.h"

#define MAX_NO_PLANES 5


float camera_position[3] = {0};


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

  std::cout << "PCL_VERSION: " << PCL_VERSION << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> planes[MAX_NO_PLANES];
  Eigen::Vector4f planeCentroid;
  int no_planes = 0;
  int nr_points = (int) cloud_filtered->points.size ();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::fromROSMsg (*input, *cloud);

  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  std::cout << "FRAME ID: " << cloud->header.frame_id << std::endl;
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  // pcl::VoxelGrid<pcl::PointXYZ> vg;

  // vg.setInputCloud (cloud);
  // vg.setLeafSize (0.001f, 0.001f, 0.001f);
  // vg.filter (*cloud_filtered);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1);
  pass.filter (*cloud_filtered);
  
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 
  cloud->clear();

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PCDWriter writer;

  seg.setOptimizeCoefficients (true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE ); //only want points perpendicular to a given axis
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (500);
  seg.setDistanceThreshold (0.005);

  Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);  //z axis
  seg.setAxis(axis);
  seg.setEpsAngle( 0.001);  
  
  while (cloud_filtered->points.size () > 500)//0.05 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter(planes[no_planes]);  //(*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << planes[no_planes].points.size() //cloud_plane->points.size() 
    << " out of " << cloud_filtered->points.size()
    << " data points." << std::endl;

    // std::stringstream sp;
    // sp << "plane_" << no_planes << ".pcd";
    // pcl::io::savePCDFileASCII(sp.str(), planes[no_planes] );  //*cloud_plane);

    // planes[no_planes] = *cloud_plane;


    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter(*cloud); // (*cloud_rest);
    *cloud_filtered = *cloud;  //*cloud_rest;
    std::cout << "Extracted component: " << cloud->points.size () << " data points." << std::endl;
    no_planes++;
  }

  // int no_planes = i;
  Eigen::Vector4f centroid, box_cen(0,0,0,0);
  float box_dim[3] = {0};
  float angle = 0.0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_segment (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::compute3DCentroid(planes[0], planeCentroid);
  // std::cout << "Plane Centroid: " << planeCentroid << std::endl;
  for (int i=1; i < no_planes; i++)
  {
    pcl::compute3DCentroid(planes[i], centroid);
    // std::cout << "Plane-- Centroid: " << centroid << std::endl;
    *plane_segment = planes[i];
    if( (planeCentroid[2] - centroid[2]) > 0.02 )   // planes lying above the table plane
    {
      pcl::io::savePCDFileASCII("plane.pcd", *plane_segment);

      // std::cout << "FRAME ID: " << plane_segment->header.frame_id << std::endl;
      angle = getOrientationPosDimension(plane_segment, box_dim);

      // std::cout << "Angle from rot mat: " << angle << std::endl;

      if(angle > 2)
        angle -= M_PI;
      else if(angle < -1.14)
        angle += M_PI;
      
      std::cout << "Angle from rot mat: " << angle << std::endl;
      box_dim[2] = planeCentroid[2] - centroid[2];
      
      if (centroid == Eigen::Vector4f(0,0,0,0))
        box_cen = centroid;
      else
      {
        box_cen[0] = camera_position[0] - centroid[1] ;
        box_cen[1] = camera_position[1] - centroid[0] ;
        box_cen[2] = camera_position[2] - (centroid[2]+box_dim[2]/2) ;
      }
  
    }
  }

  object_centroid.clear();
  object_centroid.push_back(box_cen[0]);
  object_centroid.push_back(box_cen[1]);
  object_centroid.push_back(box_cen[2]);

  dimension.clear();
  dimension.push_back(box_dim[0]) ;
  dimension.push_back(box_dim[1]) ;
  dimension.push_back(box_dim[2]) ;

  rpy.clear();
  rpy.push_back(0) ;
  rpy.push_back(0) ;
  rpy.push_back(angle) ;

  std::cout << "Centroid: (" << object_centroid[0] << ", " << object_centroid[1] << ", " << object_centroid[2] 
            << ")\nDimension: (" << dimension[0] << ", " << dimension[1] << ", " << dimension[2] 
            << ")\nAngle: " << angle << std::endl;

}

int main (int argc, char** argv)
{
  ros::Publisher pub_dim;
  ros::Publisher pub_cen;
  ros::Publisher pub_rpy;

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
  pub_rpy = nh.advertise<std_msgs::Float32MultiArray>("rpy", 10);

  std_msgs::Float32MultiArray msg_cen, msg_dim, msg_rpy;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    msg_dim.data.clear();
    msg_cen.data.clear();
    msg_rpy.data.clear();
    
    msg_dim.data = dimension;
    pub_dim.publish(msg_dim);

    msg_cen.data = object_centroid;
    pub_cen.publish(msg_cen);

    msg_rpy.data = rpy;
    pub_rpy.publish(msg_rpy);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}

float getOrientationPosDimension(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, float box_dim[])
{
  Eigen::Vector4f pcaCentroid;
  Eigen::Matrix3f covariance;
  pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);
  computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  
  std::cout << "eigenVectorsPCA: \n" << eigenVectorsPCA << std::endl;
 
  getDimension(cloudSegmented, pcaCentroid, eigenVectorsPCA, box_dim);

  float roll, pitch, yaw, Angle ;
  pitch = -asin(eigenVectorsPCA(0,2));
  roll = atan2(eigenVectorsPCA(2,2)/cos(pitch), eigenVectorsPCA(1,2)/cos(pitch));
  yaw = atan2(eigenVectorsPCA(0,0)/cos(pitch), eigenVectorsPCA(0,1)/cos(pitch));

  std::cout << " roll: " << roll  << ", pitch: " << pitch << ", yaw: " << yaw 
            << "\nPitch in degree: " << pitch*180/M_PI << std::endl;

  if(round(roll) == 0)
    Angle = pitch - M_PI/2;
  else
    Angle = pitch + M_PI/2;

  std::cout << "\nAngle: " <<  Angle << ", " << fabs(Angle) << "; " << Angle*180/M_PI << std::endl;

  return Angle;            

}

void getDimension(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented, Eigen::Vector4f pcaCentroid, 
                  Eigen::Matrix3f eigenVectorsPCA,float box_dim[])
{
  eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                  ///    the signs are different and the box doesn't get correctly oriented in some cases.
  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);

  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);

  box_dim[0] = maxPoint.y - minPoint.y;
  box_dim[1] = maxPoint.z - minPoint.z;
}

