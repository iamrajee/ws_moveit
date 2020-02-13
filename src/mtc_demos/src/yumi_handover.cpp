#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <gtest/gtest.h>
#include "test_utils.h"

using namespace moveit::task_constructor;
bool do_pause = false;

void spawnObjects() {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "yumi_body";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.5;
	o.primitive_poses[0].position.y = -0.4;
	o.primitive_poses[0].position.z = 0.1;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.2;
	o.primitives[0].dimensions[1] = 0.02;
	psi.applyCollisionObject(o);

	o.id = "table";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.44;
	o.primitive_poses[0].position.y = 0.0;
	o.primitive_poses[0].position.z = 0.07;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0] = 0.6;
	o.primitives[0].dimensions[1] = 1.2;
	o.primitives[0].dimensions[2] = 0.06;
	psi.applyCollisionObject(o);
}

Task createTask() {
	Task t;

	std::string tool_frame = "yumi_link_7_r";
	std::string eef = "right_hand";
	std::string arm = "right_arm";

	auto pipeline = std::make_shared<solvers::PipelinePlanner>();
	pipeline->setPlannerId("RRTConnectkConfigDefault");

	auto cartesian = std::make_shared<solvers::CartesianPath>();
	cartesian->setProperty("jump_threshold", 0.0);

	Stage* referenced_stage = nullptr;
	{
		// fetch initial state from move_group
		auto initial = std::make_unique<stages::CurrentState>("current state");
		referenced_stage = initial.get();
		t.add(std::move(initial));

		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));

		// grasp generator
		auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
		grasp_generator->setAngleDelta(.2);
		grasp_generator->setPreGraspPose("open");
		grasp_generator->setGraspPose("closed");
		grasp_generator->setMonitoredStage(referenced_stage);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
		grasp->setIKFrame(Eigen::Translation3d(0.0, -0.05, 0.13) *
		                  Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()),
		                  tool_frame);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp), "pick with right");
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = tool_frame;
		approach.twist.linear.z = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.1);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "world";
		lift.twist.linear.z = 1.0;
		pick->setLiftMotion(lift, 0.03, 0.05);
		t.add(std::move(pick));
	}
	{
		auto handover = std::make_unique<stages::MoveTo>("move to handover", cartesian);
		handover->setProperty("group", arm);

		// TODO: specify that attached object should move to a specific location
		geometry_msgs::PoseStamped target;
		target.header.frame_id = "yumi_body";
		target.pose.position.x =  0.40;
		target.pose.position.y = -0.12;
		target.pose.position.z =  0.1;
		target.pose.orientation.x = -0.70711;
		target.pose.orientation.w = 0.70711;
		handover->setGoal(target);
		referenced_stage = handover.get();
		t.add(std::move(handover));
	}

	/************************************************************************************/
	{
		// release object with right hand
		auto pose_generator = new stages::GenerateGraspPose("generate release pose");
		pose_generator->setAngleDelta(.2);
		pose_generator->setPreGraspPose("open");
		pose_generator->setGraspPose("closed");

		auto ungrasp = std::make_unique<stages::SimpleUnGrasp>(std::unique_ptr<MonitoringGenerator>(pose_generator));
		ungrasp->properties().configureInitFrom(Stage::PARENT, {"object"});
		ungrasp->setProperty("eef", eef);
		ungrasp->remove(-1);  // remove last stage (pose generator)

		// retract right hand
		auto retract = std::make_unique<stages::MoveRelative>("retract", cartesian);
		retract->restrictDirection(stages::MoveRelative::FORWARD);
		retract->setProperty("group", arm);
		retract->setIKFrame(tool_frame);
		retract->setProperty("marker_ns", std::string("retract"));
		geometry_msgs::TwistStamped motion;
		motion.header.frame_id = tool_frame;
		motion.twist.linear.z = -1.0;
		retract->setDirection(motion);
		retract->setProperty("min_distance", 0.05);
		retract->setProperty("max_distance", 0.1);
		ungrasp->insert(std::move(retract), -1);  // insert retract as last stage in ungrasp

		tool_frame = "yumi_link_7_l";
		eef = "left_hand";
		arm = "left_arm";

		// connect to pick
		stages::Connect::GroupPlannerVector planners = {{eef, pipeline}, {arm, pipeline}};
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		connect->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(connect));

		// grasp generator
		auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
		grasp_generator->setAngleDelta(.2);
		grasp_generator->setPreGraspPose("open");
		grasp_generator->setGraspPose("closed");
		grasp_generator->setMonitoredStage(referenced_stage);

		auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<Stage>(grasp_generator));
		grasp->setIKFrame(Eigen::Translation3d(0.0, 0.05, 0.13) *
		                  Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitX()),
		                  tool_frame);

		// insert ungrasp with right hand before attach (as second last stage)
		grasp->insert(std::move(ungrasp), -2);

		// pick container, using the generated grasp generator
		auto pick = std::make_unique<stages::Pick>(std::move(grasp), "pick with left");
		pick->setProperty("eef", eef);
		pick->setProperty("object", std::string("object"));
		geometry_msgs::TwistStamped approach;
		approach.header.frame_id = tool_frame;
		approach.twist.linear.z = 1.0;
		pick->setApproachMotion(approach, 0.03, 0.1);

		geometry_msgs::TwistStamped lift;
		lift.header.frame_id = "world";
		lift.twist.linear.z = 0.0;
		pick->setLiftMotion(lift, 0.0, 0.0);
		t.add(std::move(pick));
	}
	{
		auto place = std::make_unique<stages::MoveTo>("move to place", cartesian);
		place->setProperty("group", arm);

		// TODO: specify that attached object should move to a specific location
		geometry_msgs::PoseStamped target;
		target.header.frame_id = "yumi_body";
		target.pose.position.x = 0.45;
		target.pose.position.y = 0.45;
		target.pose.position.z = 0.151;
		target.pose.orientation.x = -0.5;
		target.pose.orientation.y = 0.5;
		target.pose.orientation.z = -0.5;
		target.pose.orientation.w = 0.5;
		place->setGoal(target);
		t.add(std::move(place));
	}
	return t;
}

TEST(Yumi, handover) {
	ros::Duration(1).sleep();
	Task t = createTask();
	spawnObjects();

	try {
		EXPECT_TRUE(t.plan()) << "planning failed" << std::endl << t;
	} catch (const InitStageException &e) {
		ADD_FAILURE() << "planning failed with exception" << std::endl << e << t;
	}

	auto num = t.solutions().size();
	EXPECT_GE(num, 30u);
	EXPECT_LE(num, 100u);

	if (do_pause) waitForKey();
}

int main(int argc, char** argv){
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "yumi");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	do_pause = doPause(argc, argv);
	return RUN_ALL_TESTS();
}
