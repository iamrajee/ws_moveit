#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_task_constructor_demo/pick_place_task14.h> // MTC pick/place demo implementation

#include <iostream>
#include<string>

constexpr char LOGNAME[] = "moveit_task_constructor_demo";

// ======================== spawnObject ====================
void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object)) //return 0 if error
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

// ======================== createTable ====================
moveit_msgs::CollisionObject createTable(int i=0) {
	ros::NodeHandle pnh("~");
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::Pose pose;

	std::string table_name_t, table_reference_frame_t, table_dimensions_t, table_pose_t;
	if(!i){
		table_name_t = "table_name";
		table_reference_frame_t = "table_reference_frame";
		table_dimensions_t = "table_dimensions";
		table_pose_t = "table_pose";
	}else{
		table_name_t = "table"+std::to_string(i)+"_name";
		table_reference_frame_t = "table"+std::to_string(i)+"_reference_frame";
		table_dimensions_t = "table"+std::to_string(i)+"_dimensions";
		table_pose_t = "table"+std::to_string(i)+"_pose";
	}


	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_name_t, table_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_reference_frame_t, table_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_dimensions_t, table_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_pose_t, pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors); //exit if no. error > 0

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z -= 0.5 * table_dimensions[2];  // align top surface of table with ground !!! its minus
	object.primitive_poses.push_back(pose);
	return object;
}

// ======================== createObject ====================
moveit_msgs::CollisionObject createObject(int i=0) {
	ros::NodeHandle pnh("~");
	
	std::string object_name, object_reference_frame;
	std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;

	std::string object_name_t, object_reference_frame_t, object_dimensions_t, object_pose_t;
	if(!i){
		object_name_t = "object_name";
		object_reference_frame_t = "object_reference_frame";
		object_dimensions_t = "object_dimensions";
		object_pose_t = "object_pose";
	}else{
		object_name_t = "object"+std::to_string(i)+"_name";
		object_reference_frame_t = "object"+std::to_string(i)+"_reference_frame";
		object_dimensions_t = "object"+std::to_string(i)+"_dimensions";
		object_pose_t = "object"+std::to_string(i)+"_pose";
	}

	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_name_t, object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_reference_frame_t, object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_dimensions_t, object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, object_pose_t, pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	pose.position.z += 0.5 * object_dimensions[0]; //??
	object.primitive_poses.push_back(pose);
	return object;
}

// ======================== main ====================
int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init moveit_task_constructor_demo");
	ros::init(argc, argv, "moveit_task_constructor_demo");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// --------- Add table and object to planning scene -----------
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::NodeHandle pnh("~");
	// if (pnh.param("spawn_table", true)){
	// 	spawnObject(psi, createTable());
	// 	spawnObject(psi, createTable(2)); // <==== new
	// }
	spawnObject(psi, createObject());

	spawnObject(psi, createObject(2)); // <==== new

	// spawnObject(psi, createObject(3)); // <==== new

	// spawnObject(psi, createObject(4)); // <==== new

	// --------- RUN ---------
	//load param
	//init
	//plan
	//execute
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", nh);
	pick_place_task.loadParameters();
	pick_place_task.init();
	if (pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			pick_place_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");                     //<=== pull request
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}
	
	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}