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

#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>

#include <string.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h> //<======== new
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>


using namespace moveit::task_constructor;
constexpr char LOGNAME[] = "cartisian_task_logname"; //<======== new

Task createTask(std::string s, double d) {
	Task t(s);
    t.stages()->setName(s);
	const std::string group = "panda1_arm";
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	Stage* current_state = nullptr;
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());

	// ====================== Current State ====================== //
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		t.add(std::move(_current_state));
	}
	// ====================== Move to Home ====================== //
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->setGroup(group);
		stage->setGoal("home1");
		stage->restrictDirection(stages::MoveTo::FORWARD);
		current_state = stage.get();
		t.add(std::move(stage));
	}
	return t;
}

Task createTask2(std::string s, double d) {
	Task t(s);
    t.stages()->setName(s);
	const std::string group = "panda2_arm";
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	Stage* current_state = nullptr;
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
    
	// ====================== Current State ====================== //
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		t.add(std::move(_current_state));
	}
	// ====================== Move to Home ====================== //
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->setGroup(group);
		stage->setGoal("home2");
		stage->restrictDirection(stages::MoveTo::FORWARD);
		current_state = stage.get();
		t.add(std::move(stage));
	}
	return t;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc");
	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> execute_("execute_task_solution", true);

	ros::AsyncSpinner spinner(1);
	spinner.start();
    Task t,t2;
    std::string s;

    s = "Task1";
	t = createTask(s,0.2);
	try {
		if (t.plan()){
            t.enableIntrospection();
			// t.introspection().publishSolution(*t.solutions().front());   
			moveit_msgs::MoveItErrorCodes execute_result = t.execute(*t.solutions().front());
            if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {ROS_INFO_NAMED(LOGNAME, "Task t execution Failed!");}else{ROS_INFO_NAMED(LOGNAME, "Task t execution completed!");}
        }
	} catch (const InitStageException& ex) {std::cerr << "planning failed with exception" << std::endl << ex << t;}
	
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

    printf("Press ctrl+C to close.\n");
	ros::waitForShutdown();
	return 0;
}