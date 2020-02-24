// ROS
#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>




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

#include <string.h>

#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h> //<======== new
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

// #include <moveit_task_constructor_demo/pick_place_task12.h>

// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_;
// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_(); //execute_("execute_task_solution", true)
// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_("execute_task_solution", true);

// PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh) : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true) {}

using namespace moveit::task_constructor;

constexpr char LOGNAME[] = "cartisian_task_logname"; //<======== new

// ros::init(argc, argv, "mtc_tutorial3");
// ros::init("mtc_tutorial3");

Task createTask(std::string s, double d) {
	Task t(s);
    t.stages()->setName(s);

	const std::string group = "panda1_arm";

	// create Cartesian interpolation "planner" to be used in stages
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());

	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "home1");
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
		auto stage = std::make_unique<stages::MoveRelative>("y +0.2", cartesian);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		// direction.vector.y = 0.2;
		direction.vector.y = d;
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

		std::map<std::string, double> offsets = { { "panda1_joint1", M_PI / 6. }, { "panda1_joint3", -M_PI / 6 } };
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

Task createTask2(std::string s, double d) {
	Task t(s);
    t.stages()->setName(s);

	const std::string group = "panda2_arm";

	// create Cartesian interpolation "planner" to be used in stages
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
    
	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "home2");
		auto fixed = std::make_unique<stages::FixedState>("initial state");// start from a fixed robot state
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.3", cartesian);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = -0.3;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("y +0.2", cartesian);
		stage->setGroup(group);
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		// direction.vector.y = 0.2;
		direction.vector.y = d;
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

		std::map<std::string, double> offsets = { { "panda2_joint1", M_PI / 6. }, { "panda2_joint3", -M_PI / 6 } };
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

Task createTask3(std::string s, double d) {
	Task t(s);
    t.stages()->setName(s);
	const std::string group = "panda_arm";
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "ready");
		auto fixed = std::make_unique<stages::FixedState>("initial state");// start from a fixed robot state
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	// ====================== Open Hand ====================== //
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup("hand");
		stage->setGoal("open");
		t.add(std::move(stage));
	}

	return t;
}

// bool task_execute(const moveit::task_constructor::Task&);
// actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_

// bool task_execute(Task task_) {
bool task_execute(const moveit::task_constructor::Task& task_, actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>& execute_) {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	task_.solutions().front()->fillMessage(execute_goal.solution);
	// execute_.sendGoal(execute_goal);
	// execute_.waitForResult();
	// moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;
	std::cout<<"flag1\n";
	execute_.sendGoal(execute_goal); 	// move_action_client_
	std::cout<<"flag2\n";
	execute_.waitForResult();
	std::cout<<"flag3\n";
	moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		// ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
		return false;
	}else{
		ROS_INFO_NAMED(LOGNAME, "Task execution completed!");
	}

	return true;
}



int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc");

	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_("execute_task_solution", true);

	// run an asynchronous spinner to communicate with the move_group node and rviz
	// ros::AsyncSpinner spinner(1);
	ros::AsyncSpinner spinner(1);
	spinner.start();
    Task t,t2;
	
    std::string s;

	// /*
    s = "Task1";
	t = createTask(s,0.2);
	try {
		if (t.plan()){
            t.enableIntrospection();
			// t.introspection().publishSolution(*t.solutions().front());   
            // task_execute(t,execute_);
			moveit_msgs::MoveItErrorCodes execute_result = t.execute(*t.solutions().front());
            if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {ROS_INFO_NAMED(LOGNAME, "Task t execution Failed!");}else{ROS_INFO_NAMED(LOGNAME, "Task t execution completed!");}
        }
	} catch (const InitStageException& ex) {std::cerr << "planning failed with exception" << std::endl << ex << t;}
	// */
	
    s = "Task2";
	t2 = createTask2(s,-0.2);
	try {
		if (t2.plan()){
            t2.enableIntrospection();
			// t2.introspection().publishSolution(*t2.solutions().front());
			moveit_msgs::MoveItErrorCodes execute_result = t2.execute(*t2.solutions().front());
			if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) { ROS_INFO_NAMED(LOGNAME, "Task t2 execution Failed!"); }else{ ROS_INFO_NAMED(LOGNAME, "Task t2 execution completed!");}
        }
	} catch (const InitStageException& ex) {std::cerr << "planning failed with exception" << std::endl << ex << t2;}
	// */

    printf("Press ctrl+C to close.\n");
	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}