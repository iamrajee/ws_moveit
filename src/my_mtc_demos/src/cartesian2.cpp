/* Inspired from work from Dr. Robert Haschke */

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>

using namespace moveit::task_constructor;

Task createTask() {
	Task t;
	t.stages()->setName("Cartesian Path");

	const std::string group = "panda_arm";

	// create Cartesian interpolation "planner" to be used in stages
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());

	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "ready");
		auto fixed = std::make_unique<stages::FixedState>("initial state");// start from a fixed robot state
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.3", cartesian);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.3;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.2", cartesian);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.2;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{  // rotate about TCP
		auto stage = std::make_unique<stages::MoveRelative>("rz -45°", cartesian);
		stage->setGroup(group);

		geometry_msgs::TwistStamped twist; // <===
		twist.header.frame_id = "world";   // <===
		twist.twist.angular.z = - M_PI / 4.; // <===
		stage->setDirection(twist);

		t.add(std::move(stage));
	}

	{  // perform a Cartesian motion, defined as a relative offset in joint space
		auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian);
		stage->setGroup(group);

		std::map<std::string, double> offsets = { { "panda_joint1", M_PI / 6. }, { "panda_joint3", -M_PI / 6 } };
		stage->setDirection(offsets);

		t.add(std::move(stage));
	}

	{  // move from reached state back to the original state, using joint interpolation
		auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();
		stages::Connect::GroupPlannerVector planners = { { group, joint_interpolation } };
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		t.add(std::move(connect));
	}

	{  // final state is original state again
		auto fixed = std::make_unique<stages::FixedState>("final state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(1);
	spinner.start();

	auto task = createTask();

	try {
		if (task.plan())
			task.introspection().publishSolution(*task.solutions().front());
	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
