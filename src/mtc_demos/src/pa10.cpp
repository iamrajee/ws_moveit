#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/move_relative.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

void spawnObject(const planning_scene::PlanningScenePtr& scene, bool right) {
	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = right ? -0.3 : 0.3;
	o.primitive_poses[0].position.y = 0.23;
	o.primitive_poses[0].position.z = 0.12;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.23;
	o.primitives[0].dimensions[1]= 0.03;
	scene->processCollisionObjectMsg(o);
}

void plan(Task &t, bool right_side) {
	std::string side = right_side ? "right" : "left";
	std::string tool_frame = side.substr(0,1) + "h_tool_frame";
	std::string eef = side.substr(0,1) + "a_tool_mount";
	std::string arm = side + "_arm";

	t.loadRobotModel();
	Stage* initial_stage = nullptr;
	// create a fixed initial scene
	{
		auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues();
		state.setToDefaultValues(state.getJointModelGroup("left_arm"), "home");
		state.setToDefaultValues(state.getJointModelGroup("right_arm"), "home");
		state.update();
		spawnObject(scene, right_side);

		auto initial = std::make_unique<stages::FixedState>();
		initial->setState(scene);
		initial_stage = initial.get();
		t.add(std::move(initial));
	}

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{side + "_hand", pipeline}, {arm, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
	grasp_generator->setAngleDelta(.2);
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("closed");
	grasp_generator->setMonitoredStage(initial_stage);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));

	if (right_side)
		grasp->setIKFrame(Eigen::Translation3d(0,0,.05)*
		                  Eigen::AngleAxisd(+0.5*M_PI, Eigen::Vector3d::UnitY()),
		                  tool_frame);
	else
		grasp->setIKFrame(Eigen::Translation3d(0,0,.05)*
		                  Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY()),
		                  tool_frame);

	// pick container, using the generated grasp generator
	auto pick = std::make_unique<stages::Pick>(std::move(grasp));
	pick->cartesianSolver()->setProperty("jump_threshold", 0.0); // disable jump check, see MoveIt #773
	pick->setProperty("eef", eef);
	pick->setProperty("object", std::string("object"));
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = tool_frame;
	approach.twist.linear.z = 1.0;
	pick->setApproachMotion(approach, 0.05, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "frame";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	t.add(std::move(pick));

	auto move = std::make_unique<stages::MoveRelative>("twist object",
	                                                   std::make_shared<solvers::CartesianPath>());
	move->properties().set("group", arm);
	move->setMinMaxDistance(0.1, 0.2);
	move->properties().set("marker_ns", std::string("lift"));
	move->properties().set("link", tool_frame);

	geometry_msgs::TwistStamped twist;
	twist.header.frame_id = "object";
	twist.twist.linear.y = 1;
	twist.twist.angular.y = 2;
	move->setDirection(twist);
	t.add(std::move(move));

	t.plan();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pa10");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");
	std::string side;
	nh.param<std::string>("side", side, "both");

	Task L("left"), R("right");
	try {
		if (side == "right" || side == "both")
			plan(R, true);
		if (side == "left" || side == "both")
			plan(L, false);

		std::cout << "waiting for any key + <enter>\n";
		char ch;
		std::cin >> ch;
	}
	catch (const InitStageException &e) {
		std::cerr << e;
		return EINVAL;
	}

	return 0;
}
