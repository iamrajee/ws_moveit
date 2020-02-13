#pragma once

#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_task_constructor_msgs/Solution.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_extents.h>

#include <shape_msgs/Mesh.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

namespace mtc_pour {

void executeSolution(const moveit_task_constructor_msgs::Solution& msg){
	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution", true);
	ac.waitForServer();

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal goal;
	goal.solution = msg;
	ac.sendGoal(goal);

	ac.waitForResult();
	ROS_INFO_STREAM("action returned: " << ac.getState().toString());
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

double computeMeshHeight(const shape_msgs::Mesh& mesh) {
	double x,y,z;
	geometric_shapes::getShapeExtents(mesh, x, y, z);
	return z;
}

void cleanup(){
	moveit::planning_interface::PlanningSceneInterface psi;

	{
		std::map<std::string, moveit_msgs::AttachedCollisionObject> attached_objects = psi.getAttachedObjects({"bottle"});
		if(attached_objects.count("bottle") > 0){
			attached_objects["bottle"].object.operation = moveit_msgs::CollisionObject::REMOVE;
			psi.applyAttachedCollisionObject(attached_objects["bottle"]);
		}
	}

	{
		std::vector<std::string> names = psi.getKnownObjectNames();

		std::vector<moveit_msgs::CollisionObject> objs;
		for(std::string& obj : std::vector<std::string> {"table", "glass", "bottle"}){
			if(std::find(names.begin(), names.end(), obj) != names.end()){
				objs.emplace_back();
				objs.back().id = obj;
				objs.back().operation = moveit_msgs::CollisionObject::REMOVE;
			}
		}

		psi.applyCollisionObjects(objs);
	}
}

void setupTable(std::vector<moveit_msgs::CollisionObject>& objects, const geometry_msgs::PoseStamped& tabletop_pose) {
	// add table
	moveit_msgs::CollisionObject table;
	table.id= "table";
	table.header= tabletop_pose.header;
	table.operation= moveit_msgs::CollisionObject::ADD;

	table.primitives.resize(1);
	table.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	table.primitives[0].dimensions.resize(3);
	table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = .5;
	table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.0;
	table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = .1;

	table.primitive_poses.resize(1);
	table.primitive_poses[0]= tabletop_pose.pose;
	table.primitive_poses[0].orientation.w= 1;
	table.primitive_poses[0].position.z-= (table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]/2);

	objects.push_back( std::move(table) );
}

void setupObjects(
	std::vector<moveit_msgs::CollisionObject>& objects,
	const geometry_msgs::PoseStamped& bottle_pose,
	const geometry_msgs::PoseStamped& glass_pose,
	const std::string& bottle_mesh= "package://mtc_pour/meshes/bottle.stl",
	const std::string& glass_mesh= "package://mtc_pour/meshes/glass.stl")
{

	{
		// add bottle
		objects.emplace_back();
		collisionObjectFromResource(
			objects.back(),
			"bottle",
			bottle_mesh);
		objects.back().header= bottle_pose.header;
		objects.back().mesh_poses[0]= bottle_pose.pose;

		// The input pose is interpreted as a point *on* the table
		objects.back().mesh_poses[0].position.z+= computeMeshHeight(objects.back().meshes[0])/2 + .002;
	}

	{
		// add glass
		objects.emplace_back();
		collisionObjectFromResource(
			objects.back(),
			"glass",
			glass_mesh);
		objects.back().header= glass_pose.header;
		objects.back().mesh_poses[0]= glass_pose.pose;
		// The input pose is interpreted as a point *on* the table
		objects.back().mesh_poses[0].position.z+= computeMeshHeight(objects.back().meshes[0])/2 + .002;
	}
}

} // namespace mtc_pour
