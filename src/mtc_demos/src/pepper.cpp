#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace moveit::task_constructor;

void spawnObject(bool right) {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "odom";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.25;
	o.primitive_poses[0].position.y = right ? -0.18 : 0.18;
	o.primitive_poses[0].position.z = 0.91;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0]= 0.12;
	o.primitives[0].dimensions[1]= 0.02;
	psi.applyCollisionObject(o);
}

void plan(Task &t, bool right_side) {
	spawnObject(right_side);

	std::string side = right_side ? "right" : "left";
	std::string tool_frame = side.substr(0,1) + "_grasp_frame";
	std::string eef = side + "_hand";
	std::string arm = side + "_arm";

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	initial_stage = initial.get();
	t.add(std::move(initial));

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	auto grasp_generator = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
	grasp_generator->setAngleDelta(.2);
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("close");
	grasp_generator->setMonitoredStage(initial_stage);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::move(grasp_generator));

	if (right_side)
		grasp->setIKFrame(Eigen::Translation3d(0.0,0.03,0.0), tool_frame);
	else
		grasp->setIKFrame(Eigen::Translation3d(0.005,0.035,0.0) *
		                  Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()), tool_frame);

	// pick container, using the generated grasp generator
	auto pick = std::make_unique<stages::Pick>(std::move(grasp_generator));
	pick->cartesianSolver()->setProperty("jump_threshold", 0.0); // disable jump check, see MoveIt #773
	pick->setProperty("eef", eef);
	pick->setProperty("object", std::string("object"));
	geometry_msgs::TwistStamped approach;
	approach.header.frame_id = tool_frame;
	approach.twist.linear.x = 1.0;
	approach.twist.linear.y = 1.0;
	pick->setApproachMotion(approach, 0.03, 0.1);

	geometry_msgs::TwistStamped lift;
	lift.header.frame_id = "base_link";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	t.add(std::move(pick));

	t.plan();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pepper");
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
