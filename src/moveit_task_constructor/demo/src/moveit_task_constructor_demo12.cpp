#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_task_constructor_demo/pick_place_task12.h> // MTC pick/place demo implementation

#include <iostream>
#include<string>

#include <geometric_shapes/mesh_operations.h>           //<============New
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_extents.h>
#include <shape_msgs/Mesh.h>

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

// ==================== Create mesh ===========================//

double computeMeshHeight(const shape_msgs::Mesh& mesh) {
	double x,y,z;
	geometric_shapes::getShapeExtents(mesh, x, y, z);
	return z;
}

void collisionObjectFromResource(moveit_msgs::CollisionObject& msg, const std::string& id, const std::string& resource) {
	msg.meshes.resize(1);

	// load mesh
	const Eigen::Vector3d scaling(1, 1, 1);
	shapes::Shape* shape = shapes::createMeshFromResource(resource, scaling);
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(shape, shape_msg);
	msg.meshes[0] = boost::get<shape_msgs::Mesh>(shape_msg);

	// set pose
	msg.mesh_poses.resize(1);
	msg.mesh_poses[0].orientation.w = 1.0;

	// fill in details for MoveIt
	msg.id = id;
	msg.operation = moveit_msgs::CollisionObject::ADD;
}

void createmesh() {
	
    const double table_height= 0;

	std::string bottle_name, bottle_reference_frame, bottle_mesh, glass_name, glass_reference_frame, glass_mesh,world_frame;
	std::vector<double> bottle_dimensions, glass_dimensions;
	geometry_msgs::Pose bottle_pose_only, glass_pose_only;

	ros::NodeHandle pnh("~");
	std::size_t errors = 0;

	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame);

	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "bottle_name", bottle_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "bottle_mesh", bottle_mesh);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "bottle_reference_frame", bottle_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "bottle_dimensions", bottle_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "bottle_pose_only", bottle_pose_only);

	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "glass_name", glass_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "glass_mesh", glass_mesh);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "glass_reference_frame", glass_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "glass_dimensions", glass_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "glass_pose_only", glass_pose_only);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	geometry_msgs::PoseStamped bottle_pose, glass_pose;
	bottle_pose.header.frame_id= world_frame;
    bottle_pose.pose= bottle_pose_only;
	glass_pose.header.frame_id= world_frame;
    glass_pose.pose= glass_pose_only;

    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::CollisionObject> objects;
    // mtc_pour::setupObjects(objs, bottle, glass, "package://mtc_pour/meshes/small_bottle.stl");

    {
        // add bottle
        objects.emplace_back();
        collisionObjectFromResource(
            objects.back(),
            bottle_name,
            bottle_mesh);
        objects.back().header= bottle_pose.header;
        objects.back().mesh_poses[0]= bottle_pose.pose;

        // The input pose is interpreted as a point *on* the table
        auto mesh_height = computeMeshHeight(objects.back().meshes[0]);
        objects.back().mesh_poses[0].position.z+= computeMeshHeight(objects.back().meshes[0])/2 + .002;
        ROS_INFO_STREAM("DEBUG: mesh height: "<< computeMeshHeight(objects.back().meshes[0]));
    }

	{
		// add glass
		objects.emplace_back();
		collisionObjectFromResource(
			objects.back(),
			glass_name,
			glass_mesh);
		objects.back().header= glass_pose.header;
		objects.back().mesh_poses[0]= glass_pose.pose;
		// The input pose is interpreted as a point *on* the table
		objects.back().mesh_poses[0].position.z+= computeMeshHeight(objects.back().meshes[0])/2 + .002;
	}

    psi.applyCollisionObjects(objects);
}


//==============================================================//

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
	// spawnObject(psi, createObject());

	// spawnObject(psi, createObject(2)); // <==== new

	// spawnObject(psi, createObject(3)); // <==== new

	// spawnObject(psi, createObject(4)); // <==== new

	createmesh();					  // <==== new

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