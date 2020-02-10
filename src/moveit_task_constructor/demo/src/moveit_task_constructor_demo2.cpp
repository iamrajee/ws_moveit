#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_task_constructor_demo/pick_place_task2.h> // MTC pick/place demo implementation

constexpr char LOGNAME[] = "moveit_task_constructor_demo";

// ======================== spawnObject ====================
void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object)) //return 0 if error
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

// ======================== createTable ====================
moveit_msgs::CollisionObject createTable() {
	ros::NodeHandle pnh("~");
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors); //exit if no. error > 0

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world i.e place table on surface
	object.primitive_poses.push_back(pose);
	return object;
}

// ======================== createObject ====================
moveit_msgs::CollisionObject createObject() {
	ros::NodeHandle pnh("~");
	std::string object_name, object_reference_frame;
	std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
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
	if (pnh.param("spawn_table", true))
		spawnObject(psi, createTable());
	spawnObject(psi, createObject());

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