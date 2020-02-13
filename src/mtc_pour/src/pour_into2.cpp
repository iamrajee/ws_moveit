/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-2018, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Hamburg University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner, Henning Kayser
   Desc:    Pour from attached bottle into(onto) an object
*/

#include <moveit/task_constructor/stages/pour_into2.h>                                    //<=============== new

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometric_shapes/shape_extents.h>

#include <shape_msgs/SolidPrimitive.h>

#include <eigen_conversions/eigen_msg.h>

#include <rviz_marker_tools/marker_creation.h>

namespace {
/** Compute prototype waypoints for pouring.
  * This generates a trajectory that pours in the Y-Z axis.
  * The generated poses are for the bottle-tip and are relative to the container top-center */
void computePouringWaypoints(const Eigen::Isometry3d& start_tip_pose, double tilt_angle,
                             const Eigen::Translation3d& pouring_offset, EigenSTL::vector_Isometry3d& waypoints,
                             unsigned int nr_of_waypoints= 10) {
	Eigen::Isometry3d start_tip_rotation(start_tip_pose);
	start_tip_rotation.translation().fill(0);

	waypoints.push_back(start_tip_pose);

	for(unsigned int i= 1; i <= nr_of_waypoints; ++i){
		const double fraction= (double) i / nr_of_waypoints;
		const double exp_fraction= fraction*fraction;
		const double offset_fraction= std::pow(fraction, 1.0/5.0) + (-4.5*fraction*fraction + 4.5*fraction); // custom trajectory translating away from cup center

		// linear interpolation for tilt angle
		Eigen::Isometry3d rotation = Eigen::AngleAxisd(fraction*tilt_angle, Eigen::Vector3d::UnitX()) * start_tip_rotation;

		// exponential interpolation towards container rim + offset
		Eigen::Translation3d translation(
			start_tip_pose.translation() * (1-exp_fraction) +
			pouring_offset.translation() * exp_fraction
			);

		translation.y() = start_tip_pose.translation().y() * (1-offset_fraction) +
		                  pouring_offset.translation().y() * (offset_fraction);

		waypoints.push_back(translation*rotation);
	}
}

// check the CollisionObject types this stage can handle
inline bool isValidObject(const moveit_msgs::CollisionObject& o){
	return (o.meshes.size() == 1 && o.mesh_poses.size() == 1 && o.primitives.empty()) ||
	       (o.meshes.empty() && o.primitives.size() == 1 && o.primitive_poses.size() == 1 && o.primitives[0].type == shape_msgs::SolidPrimitive::CYLINDER);
}

/* compute height of the CollisionObject
   This is only useful for meshes, when they are centered */
inline double getObjectHeight(const moveit_msgs::CollisionObject& o){
	if( !o.meshes.empty() ){
		double x,y,z;
		geometric_shapes::getShapeExtents(o.meshes[0], x, y, z);
		return z;
	}
	else {
		// validations guarantees this is a cylinder
		return o.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT];
	}
}

} /* anonymous namespace */



namespace mtc_pour {

PourInto::PourInto(std::string name) :
	PropagatingForward(std::move(name))
{
	auto& p = properties();
	p.declare<std::string>("group", "name of planning group");

	p.declare<std::string>("bottle", "attached bottle-like object");
	p.declare<std::string>("container", "container object to be filled");

	p.declare<double>("tilt_angle", "maximum tilt-angle for the bottle");
	p.declare<double>("min_path_fraction", 0.9, "minimum valid fraction of the planned pouring path");
	p.declare<size_t>("waypoint_count", 10, "Number of Cartesian waypoints to approximate pouring trajectory");
	p.declare<Eigen::Vector3d>("pour_offset", "offset for the bottle tip w.r.t. container top-center during pouring");
	p.declare<geometry_msgs::Vector3Stamped>("pouring_axis", "Axis around which to pour");
	// p.declare<ros::Duration>("pour_duration", ros::Duration(1.0), "duration to stay in pouring pose");
    p.declare<ros::Duration>("pour_duration", ros::Duration(16.0), "duration to stay in pouring pose");        // shouldn't & doesn't work
	// p.declare<ros::Duration>("waypoint_duration", ros::Duration(0.5), "duration between pouring waypoints");
	p.declare<ros::Duration>("waypoint_duration", ros::Duration(2.0), "duration between pouring waypoints");  // shouldn't & doesn't work
}

/* MTC stage interface */
void PourInto::computeForward(const InterfaceState& from) {
	planning_scene::PlanningScenePtr to;
	SubTrajectory trajectory;

	compute(from, to, trajectory);
	sendForward(from, InterfaceState(to), std::move(trajectory));
	return;
}

void PourInto::compute(const InterfaceState& input, planning_scene::PlanningScenePtr& result, SubTrajectory& trajectory) {
	const auto& props= properties();

	const std::string& container_name= props.get<std::string>("container");
	const std::string& bottle_name= props.get<std::string>("bottle");

	const Eigen::Translation3d pour_offset( props.get<Eigen::Vector3d>("pour_offset") );
	const auto& tilt_angle= props.get<double>("tilt_angle");

	const auto& min_path_fraction= props.get<double>("min_path_fraction");

	const ros::Duration pour_duration( props.get<ros::Duration>("pour_duration") );
	const ros::Duration waypoint_duration( props.get<ros::Duration>("waypoint_duration") );

	const planning_scene::PlanningScene& scene= *input.scene();
	moveit::core::RobotModelConstPtr robot_model= scene.getRobotModel();
	const moveit::core::JointModelGroup* group= robot_model->getJointModelGroup(props.get<std::string>("group"));

	/* validate planning environment is prepared for pouring */
	moveit_msgs::CollisionObject container;
	if(!scene.getCollisionObjectMsg(container, container_name))
		throw std::runtime_error("container object '" + container_name + "' is not specified in input planning scene");
	if(!isValidObject(container))
		throw std::runtime_error("PourInto: container is neither a valid cylinder nor mesh.");

	moveit_msgs::AttachedCollisionObject bottle;
	if(!scene.getAttachedCollisionObjectMsg(bottle, bottle_name))
		throw std::runtime_error("bottle '" + bottle_name + "' is not an attached collision object in input planning scene");
	if(!isValidObject(bottle.object))
		throw std::runtime_error("PourInto: bottle is neither a valid cylinder nor mesh.");

	moveit::core::RobotState state(scene.getCurrentState());

	// container frame:
	// - top-center of container object
	// - rotation should coincide with the planning frame
	Eigen::Isometry3d container_frame=
		scene.getFrameTransform(container_name) *
		Eigen::Translation3d(Eigen::Vector3d(0,0, getObjectHeight(container)/2));
	container_frame.linear().setIdentity();

	/* compute pouring axis as one angle (tilt_axis_angle) in x-y plane */
	//// TODO: spawn many axis if this is not set
	const auto& pouring_axis= props.get<geometry_msgs::Vector3Stamped>("pouring_axis");
	Eigen::Vector3d tilt_axis;
	tf::vectorMsgToEigen(pouring_axis.vector, tilt_axis);
	tilt_axis= container_frame.inverse() * scene.getFrameTransform(pouring_axis.header.frame_id) * tilt_axis;
	// always tilt around axis in x-y plane
	tilt_axis.z()= 0.0;
	double tilt_axis_angle= std::atan2(tilt_axis.y(), tilt_axis.x());

	const Eigen::Isometry3d& bottle_frame= scene.getFrameTransform(bottle_name);

	// assume bottle tip as top-center of cylinder/mesh

	auto& attached_bottle_tfs= state.getAttachedBody(bottle_name)->getFixedTransforms();
	assert(attached_bottle_tfs.size() > 0 && "impossible: attached body does not know transform to its link");

	const Eigen::Translation3d bottle_tip(Eigen::Vector3d(0, 0, getObjectHeight(bottle.object)/2));
	const Eigen::Isometry3d bottle_tip_in_tool_link(attached_bottle_tfs[0]*bottle_tip);

	const Eigen::Isometry3d bottle_tip_in_container_frame=
		container_frame.inverse() *
		bottle_frame *
		bottle_tip;

	/* Cartesian waypoints for pouring motion */
	EigenSTL::vector_Isometry3d waypoints;

	/* generate waypoints in y-z plane */
	computePouringWaypoints(bottle_tip_in_container_frame, tilt_angle, pour_offset, waypoints, props.get<size_t>("waypoint_count"));

	/* rotate y-z plane so tilt motion is along the specified tilt_axis */
	for(auto& waypoint : waypoints)
		waypoint= Eigen::AngleAxisd(tilt_axis_angle, Eigen::Vector3d::UnitZ()) * waypoint * Eigen::AngleAxisd(-tilt_axis_angle, Eigen::Vector3d::UnitZ());

	// TODO: possibly also spawn alternatives:
	//for(auto& waypoint : waypoints)
	//	waypoint= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * waypoint * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

	/* transform waypoints to planning frame */
	for(auto& waypoint : waypoints)
		waypoint= container_frame*waypoint;

	for(auto waypoint : waypoints){
		geometry_msgs::PoseStamped p;
		p.header.frame_id= scene.getPlanningFrame();
		tf::poseEigenToMsg(waypoint, p.pose);

		rviz_marker_tools::appendFrame(trajectory.markers(), p, 0.1, markerNS());

		//visualization_msgs::Marker tip;
		//tip.ns= markerNS();
		//tip.header= p.header;
		//tip.pose= rviz_marker_tools::composePoses(p.pose, Eigen::Isometry3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(0,1,0))));
		//tip.color.r= .588;
		//tip.color.g= .196;
		//tip.color.b= .588;
		//tip.color.a= 1.0;
		//// TODO: rename or move this package! maybe move it in with moveit_visual_tools?
		//rviz_marker_tools::makeArrow(tip, .11, true);
		//trajectory.markers().push_back(tip);
	}

	/* specify waypoints for tool link, not for bottle tip */
	for(auto& waypoint : waypoints)
		waypoint= waypoint*bottle_tip_in_tool_link.inverse();

	std::vector<moveit::core::RobotStatePtr> traj;

	// TODO: this has to use computeCartesianPath because
	// there is currently no multi-waypoint callback in cartesian_planner
	double path_fraction= state.computeCartesianPath(
		group,
		traj,
		state.getLinkModel(bottle.link_name),
		waypoints,
		true /* global reference_frame */,
		.03 /* max step size */,
		2.0 /* jump threshold */,
		[&scene](moveit::core::RobotState* rs,
		         const moveit::core::JointModelGroup* jmg,
		         const double* joint_positions){
			rs->setJointGroupPositions(jmg, joint_positions);
			rs->update();
			return !scene.isStateColliding(*rs, jmg->getName());
			}
		);

	/* build executable RobotTrajectory (downward and back up) */
	auto robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);
	robot_trajectory::RobotTrajectory back_trajectory(robot_model, group);

	for(const auto& waypoint : traj){
		robot_trajectory->addSuffixWayPoint(waypoint, 0.0);
	}

	for(auto waypoint = traj.rbegin(); waypoint != traj.rend(); waypoint++){
		back_trajectory.addSuffixWayPoint(std::make_shared<robot_state::RobotState>(**waypoint), 0.0);
	}

	/* generate time parameterization */
	// TODO: revert code and interfaces to ISP / needs testing on hardware
	trajectory_processing::IterativeSplineParameterization isp;
	const double velocity_scaling= 1.0;
	const double acceleration_scaling= 1.0;
	{
		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		isp.computeTimeStamps(*robot_trajectory, velocity_scaling, acceleration_scaling);
	}
	{
		trajectory_processing::IterativeParabolicTimeParameterization iptp;
		isp.computeTimeStamps(back_trajectory, velocity_scaling, acceleration_scaling);
	}
	for(size_t i= 0; i < robot_trajectory->getWayPointCount(); ++i)                       //========= It was commented, but after uncommenting still doesn't work
		robot_trajectory->setWayPointDurationFromPrevious(i, (1-i/robot_trajectory->getWayPointCount())*waypoint_duration.toSec());  //
	for(size_t i= 0; i < back_trajectory.getWayPointCount(); ++i)                         //
		back_trajectory.setWayPointDurationFromPrevious(i, 0.2*waypoint_duration.toSec());    // --------------------------------------------------------------------//

	/* combine downward and upward motion / sleep pour_duration seconds between */
	robot_trajectory->append(back_trajectory, pour_duration.toSec());

	trajectory.setTrajectory(robot_trajectory);

	result= scene.diff();
	result->setCurrentState(robot_trajectory->getLastWayPoint());

	if ( path_fraction < min_path_fraction ){
		ROS_WARN_STREAM("PourInto only produced motion for " << path_fraction << " of the way. Rendering invalid");
		trajectory.setCost(std::numeric_limits<double>::infinity());
		trajectory.setComment("pouring axis angle " + std::to_string(tilt_axis_angle));
		return;
	}

	return;
}

}
