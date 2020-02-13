#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gtest/gtest.h>
#include "test_utils.h"

using namespace moveit::task_constructor;
bool do_pause = false;

void spawnObject(double pos) {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = pos;
	o.primitive_poses[0].position.y = 0.23;
	o.primitive_poses[0].position.z = 0.12;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);
}

void fill(ParallelContainerBase &container, Stage* initial_stage, bool right_side) {
	std::string side = right_side ? "right" : "left";
	std::string tool_frame = side.substr(0,1) + "h_tool_frame";
	std::string eef = side.substr(0,1) + "a_tool_mount";
	std::string arm = side + "_arm";

	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = {{side + "_hand", pipeline}, {arm, pipeline}};
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);

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
	auto pick = std::make_unique<stages::Pick>(std::move(grasp), side);
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

	// twist motion
	auto move = std::make_unique<stages::MoveRelative>("twist object",
	                                                   std::make_shared<solvers::CartesianPath>());
	move->properties().set("group", arm);
	move->setMinMaxDistance(0.1, 0.2);
	move->setProperty("marker_ns", std::string("lift"));
	move->setIKFrame(tool_frame);

	geometry_msgs::TwistStamped twist;
	twist.header.frame_id = "object";
	twist.twist.linear.y = 1;
	twist.twist.angular.y = 2;
	move->setDirection(twist);
	pick->insert(std::move(move));

	pick->insert(std::move(connect), 0);
	container.insert(std::move(pick));
}

TEST(PA10, bimodal) {
	Task t;

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>();
	initial_stage = initial.get();
	t.add(std::move(initial));

	auto parallel = std::make_unique<Alternatives>();
	fill(*parallel, initial_stage, true);
	fill(*parallel, initial_stage, false);

	t.add(std::move(parallel));

	size_t failures = 0;
	size_t successes = 0;
	size_t solutions = 0;
	for (double pos = -0.8; pos <= 0.801; pos += 0.2) {
		SCOPED_TRACE("object at pos=" + std::to_string(pos));

		spawnObject(pos);
		try {
			t.plan();
		} catch (const InitStageException &e) {
			ADD_FAILURE() << "planning failed with exception" << std::endl << e << t;
		}

		auto num = t.solutions().size();
		if (!num) {
			++failures;
			std::cerr << "planning failed with object at " << pos << std::endl << t << std::endl;
		} else {
			++successes;
			solutions += num;

			EXPECT_GE(num, 1u);
			EXPECT_LE(num, 20u);
		}
	}
	EXPECT_LE((double)failures / (successes + failures), 0.2) << "failure rate too high";
	EXPECT_GE((double)solutions / successes, 5) << "avg number of solutions too small";

	if (do_pause) waitForKey();
}

int main(int argc, char** argv){
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "pa10");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	do_pause = doPause(argc, argv);
	return RUN_ALL_TESTS();
}
