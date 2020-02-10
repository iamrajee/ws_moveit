/*
planning_interface :: moveit_commander
gn: PLANNING_GROUP
g : move_group               : MoveGroupInterface
s : planning_scene_interface : PlanningSceneInterface
*/

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string gn = "panda_arm";
  moveit::planning_interface::MoveGroupInterface g(gn);
  moveit::planning_interface::PlanningSceneInterface s;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = g.getCurrentState()->getJointModelGroup(gn);

  //======== VISUALISATION ===========
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  // markersof type text, cylinders, and spheres
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger(); // Batch publishing

  //======PRINT BASIC INFO ==========
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", g.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", g.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(g.getJointModelGroupNames().begin(), g.getJointModelGroupNames().end(),std::ostream_iterator<std::string>(std::cout, ", "));

  // ============== Start ============
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // =============== Plan Pose goal ==============
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  g.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (g.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // =============== Visvualise plan ==============
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // ============= execute plan ==============
  // g.move(); //dont use here blocking call as previous one is uncomplete?

  // ============== Plan joint-space goal ==============
  moveit::core::RobotStatePtr current_state = g.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = -1.0;  // radians
  g.setJointValueTarget(joint_group_positions);
  success = (g.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // ========= Visualize plan =================
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // ========== Plan with Path Constraints ==========
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  g.setPathConstraints(test_constraints);

  // NOTE !! : CURRENT POSE SHOULD ALSO SATISFY CONSTRAINS ELSE CHANGE CURRENT POSE
  robot_state::RobotState start_state(*g.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  start_state.setFromIK(joint_model_group, start_pose2);
  g.setStartState(start_state);

  g.setPoseTarget(target_pose1);
  g.setPlanningTime(10.0); //increase time as planning with constraints can slow
  success = (g.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  // ========= Visualize plan =================
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(start_pose2, "start");
  visual_tools.publishAxisLabeled(target_pose1, "goal");
  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  g.clearPathConstraints(); // !!IMPORTANT

  // =========== Plan Cartesian Paths ==========
  std::vector<geometry_msgs::Pose> waypoints;
  
  geometry_msgs::Pose target_pose3 = start_pose2;
  waypoints.push_back(target_pose3);  //add current
  target_pose3.position.z -= 0.2;
  waypoints.push_back(target_pose3);  // down
  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // right
  target_pose3.position.z += 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // up and left

  g.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = g.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // ========= Visualize plan =================
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Adding/Removing Objects and Attaching/Detaching Objects
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = g.getPlanningFrame();
  collision_object.id = "box1";
  
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  s.addCollisionObjects(collision_objects);
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);// Show text in RViz of status
  visual_tools.trigger(); 
  //and wait a bit
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // =============== Plan Pose goal(will avoid obstacle) ==============
  g.setStartState(*g.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose.orientation.w = 1.0;
  another_pose.position.x = 0.4;
  another_pose.position.y = -0.4;
  another_pose.position.z = 0.9;
  g.setPoseTarget(another_pose);
  success = (g.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // ========= Visualize plan =================
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // ========= Attach =================
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  g.attachObject(collision_object.id);
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the robot");

  // ========= Detach =================
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  g.detachObject(collision_object.id);
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the robot");

  // ========= Remove =================
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  s.removeCollisionObjects(object_ids);
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

  //END
  ros::shutdown();
  return 0;
}