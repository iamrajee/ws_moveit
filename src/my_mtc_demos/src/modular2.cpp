/* Inspired from work from Dr. Robert Haschke */

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/container.h>

using namespace moveit::task_constructor;

std::unique_ptr<SerialContainer> createModule(const std::string& group) {
	auto c = std::make_unique<SerialContainer>("Cartesian Path");
	c->setProperty("group", group);

	// Cartesian & joint planner
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.2;
		stage->setDirection(direction);
		c->insert(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.3", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.3;
		stage->setDirection(direction);
		c->insert(std::move(stage));
	}

	{  // rotate about TCP
		auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT);
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = M_PI / 4.;
		stage->setDirection(twist);
		c->insert(std::move(stage));
	}

	{  // move back to ready pose
		auto stage = std::make_unique<stages::MoveTo>("moveTo ready", joint_interpolation);
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setGoal("ready");
		c->insert(std::move(stage));
	}
	return c;
}

Task createTask() {
	Task t;
	t.stages()->setName("Reusable Containers");
	t.add(std::make_unique<stages::CurrentState>("current"));

	const std::string group = "panda_arm";
	t.add(createModule(group));
	t.add(createModule(group));
	t.add(createModule(group));
	t.add(createModule(group));
	t.add(createModule(group));

	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	ros::AsyncSpinner spinner(1); // run an asynchronous spinner to communicate with the move_group node and rviz
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