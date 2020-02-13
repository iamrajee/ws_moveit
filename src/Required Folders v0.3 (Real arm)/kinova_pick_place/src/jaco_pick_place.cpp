#include <pick_place_.h>

static const std::string PLANNING_GROUP = "arm";
using namespace kinova;

void dimensionCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
		dim[i] = *it;
	//	ROS_INFO("Dimension[%d]: [%f]",i , dim[i]);
		i++;
		
	}

	return;
}

void poseCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
		pose[i] = *it;
	//	ROS_INFO("Position[%d]: [%f]",i, pose[i]);
		i++;
		
	}

	return;
}

void rpyCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it) {
		rpy[i] = *it;
	//	ROS_INFO("Position[%d]: [%f]",i, pose[i]);
		i++;
		
	}

	return;
}

PickPlace::PickPlace(ros::NodeHandle &nh):
    nh_(nh)
{
    group_ = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface_ = new moveit::planning_interface::PlanningSceneInterface;
    
    group_->setPlanningTime(100.0);
    clear_target();
 
    for(int i=0;i<3;i++) ROS_INFO("Dimension [%c]: [%f]",axis[i] , dim[i]);  
    for(int i=0;i<3;i++)ROS_INFO("Position  [%c]: [%f]",axis[i] , pose[i]);
    for(int i=0;i<3;i++)ROS_INFO("rpy  [%c]: [%f]",_rpy[i] , rpy[i]);
    addCollisionObjects();

    close_offset = (dim[1]/0.05)-0.6;
    group_->setMaxAccelerationScalingFactor(0.1); // TO-DO : Real Arm -> tune the value b/w (0, 1] 
    group_->setMaxVelocityScalingFactor(0.1);     // TO-DO : Real Arm -> tune the value b/w (0, 1] 

    ros::WallDuration(1.0).sleep(); 
    
    check_constrain();

    pick();

    ros::WallDuration(1.0).sleep();

    place();
}


PickPlace::~PickPlace()
{    
    delete group_;
    delete planning_scene_interface_;
}

void PickPlace::openGripper(trajectory_msgs::JointTrajectory& posture)
{
 
  posture.joint_names.resize(3);
  posture.joint_names[0] = "j2s6s300_joint_finger_1";
  posture.joint_names[1] = "j2s6s300_joint_finger_2";
  posture.joint_names[2] = "j2s6s300_joint_finger_3";

  posture.points.resize(1);
  posture.points[0].positions.resize(3);
  posture.points[0].effort.resize(3);
  posture.points[0].positions[0] = -0.0045;
  posture.points[0].positions[1] = -0.0045;
  posture.points[0].positions[2] = -0.0045;
  posture.points[0].effort[0] = 1000.0;			// TO-DO : Real Arm -> Check the effect of this parameter
  posture.points[0].effort[1] = 1000.0;			// TO-DO : Real Arm -> Check the effect of this parameter
  posture.points[0].effort[2] = 1000.0;			// TO-DO : Real Arm -> Check the effect of this parameter
  posture.points[0].time_from_start = ros::Duration(1.0);
	
}

void PickPlace::closedGripper(trajectory_msgs::JointTrajectory& posture)
{

  posture.joint_names.resize(3);
  posture.joint_names[0] = "j2s6s300_joint_finger_1";
  posture.joint_names[1] = "j2s6s300_joint_finger_2";
  posture.joint_names[2] = "j2s6s300_joint_finger_3";

  float close_init = 1.0;
  ROS_INFO("CLOSE ANGLE : %f", close_init - close_offset);
  posture.points.resize(1);	
  posture.points[0].positions.resize(3);
  posture.points[0].effort.resize(3);
  posture.points[0].positions[0] = 0.9;
  posture.points[0].positions[1] = 0.9;	
  posture.points[0].positions[2] = 0.9;
  posture.points[0].effort[0] = 20.0;			// TO-DO : Real Arm -> Check the effect of this parameter
  posture.points[0].effort[1] = 20.0;			// TO-DO : Real Arm -> Check the effect of this parameter
  posture.points[0].effort[2] = 20.0;			// TO-DO : Real Arm -> Check the effect of this parameter
  posture.points[0].time_from_start = ros::Duration(1.0);
  
}

void PickPlace::pick()
{
  
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  int count = 1;
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  
  grasps[0].grasp_pose.header.frame_id = "object";
  
  //tf2::Quaternion orientation;
  // orientation.setRPY(-0.111641, 0.009703, 1.615731);
  // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  
  grasps[0].grasp_pose.pose.position.x = 0.0;
  grasps[0].grasp_pose.pose.position.y = 0.0;
  grasps[0].grasp_pose.pose.position.z = 0.0;
  grasps[0].grasp_pose.pose.orientation.x = -0.0233387;
  grasps[0].grasp_pose.pose.orientation.y = 0.734995;
  grasps[0].grasp_pose.pose.orientation.z = 0.676809;
  grasps[0].grasp_pose.pose.orientation.w = -0.0341765;
  grasps[0].allowed_touch_objects = std::vector<std::string>{"object"};
  grasps[0].max_contact_force = 100;		// TO-DO : Real Arm -> Check the effect of this parameter
  
  
  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
 
  grasps[0].pre_grasp_approach.direction.header.frame_id = "world";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.0;
  grasps[0].pre_grasp_approach.desired_distance = 0.5;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++

  grasps[0].post_grasp_retreat.direction.header.frame_id = "world";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.0;
  //~ grasps[0].post_grasp_retreat.desired_distance = 0.15;
  grasps[0].post_grasp_retreat.desired_distance = 0.0;

  // Setting posture of eef before grasp	
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  
  // Set support surface as table1.
  // group_->setSupportSurfaceName("table_top");
  // move_group.setNumPlanningAttempts(8);
  // Call pick to pick up the object using the grasps given
  // move_group.pick("object", grasps);
 
 do{
 success = ( group_->pick("object", grasps) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
 ROS_INFO("PICK: Visualizing plan %d  %s", count, success ? "SUCCESS" : "FAILED");
 ros::WallDuration(2.0).sleep();
 count++;
 }while(success == 0);

}


void PickPlace::place()
{
 
  //  Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  
  int count = 1;
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "world";
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, 0.0);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
  
  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0.05;
  place_location[0].place_pose.pose.position.y = 0.45;
  place_location[0].place_pose.pose.position.z = 0.1;
  place_location[0].allowed_touch_objects = std::vector<std::string>{"object"};
  
  // Setting pre-place approach
  // ++++++++++++++++++++++++++
 
  place_location[0].pre_place_approach.direction.header.frame_id = "world";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance =0.0; 
  place_location[0].pre_place_approach.desired_distance = 0.115;
    
  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  
  place_location[0].post_place_retreat.direction.header.frame_id = "world";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;
 
  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  
  openGripper(place_location[0].post_place_posture);
  
  //group_->setSupportSurfaceName("table_top");
  
 do{
 success = ( group_->place("object", place_location) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
 ROS_INFO("PLACE: Visualizing plan %d  %s", count, success ? "SUCCESS" : "FAILED");
 ros::WallDuration(2.0).sleep();
 count++;
 }while(success == 0);
 
 
}

void PickPlace::addCollisionObjects()
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector  collision objects.
  // std::vector<moveit_msgs::CollisionObject> collision_objects;
  // collision_objects.resize(1);
  
  collision_objects.resize(2);
    
  
  // OBJECT

  // Define the object that we will be manipulating
  collision_objects[0].header.frame_id = "world";
  collision_objects[0].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);	
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[0].primitives[0].dimensions.resize(2);
  collision_objects[0].primitives[0].dimensions[0] = dim[1];//0.18;  //height 
  collision_objects[0].primitives[0].dimensions[1] = dim[0];//0.03;  //radius
  //~ collision_objects[0].primitives[0].dimensions[2] = 0.18;
  

  //~ tf2::Quaternion orientation;
  //~ orientation.setRPY(0, 0, rpy_z);

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = pose[0];
  collision_objects[0].primitive_poses[0].position.y = pose[1];
  collision_objects[0].primitive_poses[0].position.z = pose[2];
  //collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  //~ collision_objects[0].primitive_poses[0].orientation = tf2::toMsg(orientation);
  collision_objects[0].operation = collision_objects[0].ADD;
  
   //TABLE
   
  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "world";
  collision_objects[1].id = "table";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);	
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 1;
  collision_objects[1].primitives[0].dimensions[1] = 1.5;
  collision_objects[1].primitives[0].dimensions[2] = 0.03;
  

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.5;
  collision_objects[1].primitive_poses[0].position.y = 0.75;
  collision_objects[1].primitive_poses[0].position.z = -0.015;
  collision_objects[1].operation = collision_objects[1].ADD;
  
  planning_scene_interface_->applyCollisionObjects(collision_objects);
  
}

void PickPlace::clear_target()
{ 
  collision_objects.resize(1);
  collision_objects[0].id = "object";
  collision_objects[0].operation = moveit_msgs::CollisionObject::REMOVE;
  planning_scene_interface_->applyCollisionObjects(collision_objects);

}

void PickPlace::check_constrain()
{
    moveit_msgs::Constraints grasp_constrains = group_->getPathConstraints();
    bool has_constrain = false;
    ROS_INFO("check constrain result: ");
    if (!grasp_constrains.orientation_constraints.empty())
    {
        has_constrain = true;
        ROS_INFO("Has orientation constrain.");
    }
    if(!grasp_constrains.position_constraints.empty())
    {
        has_constrain = true;
        ROS_INFO("Has position constrain. ");
    }
    if(has_constrain == false)
    {
        ROS_INFO("No constrain. ");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_place_dev");
    ros::NodeHandle node;

    //~ object_x = atof(argv[1]);
    //~ object_y = atof(argv[2]);
    //~ object_z = atof(argv[3]);
    //~ rpy_z = atof(argv[4]);
    
    box_dim_sub = node.subscribe("dimension", 10, dimensionCallback);
    box_pose_sub = node.subscribe("centroid", 10, poseCallback);
    box_rpy_sub = node.subscribe("rpy", 10, rpyCallback);
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::WallDuration(1.0).sleep();

    kinova::PickPlace pick_place(node);

    ros::waitForShutdown();

    return 0;
}
