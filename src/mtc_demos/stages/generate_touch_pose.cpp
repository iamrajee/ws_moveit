/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bielefeld University
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
 *   * Neither the name of Bielefeld University nor the names of its
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

/* Authors: Robert Haschke */

#include "generate_touch_pose.h"
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

namespace moveit { namespace task_constructor { namespace stages {

GenerateTouchPose::GenerateTouchPose(const std::string& name)
   : GenerateGraspPose(name)
{
}

void GenerateTouchPose::compute()
{
	if (upstream_solutions_.empty())
		return;
	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	const auto& props = properties();
	const std::string& object_name = props.get<std::string>("object");
	const std::string& eef = props.get<std::string>("eef");

	geometry_msgs::PoseStamped pose;
	pose.header.frame_id = scene->getPlanningFrame();
	pose.pose.orientation.w = 1.0;

	// TODO: evaluate object shape
	collision_detection::World::ObjectConstPtr object
	      = scene->getWorld()->getObject(object_name);
	if (object->shape_poses_.size() != 1) {
		ROS_WARN_STREAM_NAMED("GenerateTouchPose", "object has " << object->shape_poses_.size() << " shapes");
		return;
	}
	// enable object collision with end-effector
	collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
	acm.setEntry(object_name, scene->getRobotModel()->getEndEffector(eef)
	             ->getLinkModelNamesWithCollisionGeometry(), true);

	tf::pointEigenToMsg(object->shape_poses_[0].translation(), pose.pose.position);

	double current_angle_ = 0.0;
	while (current_angle_ < 2.*M_PI && current_angle_ > -2.*M_PI) {
		// rotate object pose about z-axis
		Eigen::Quaterniond q(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()));
		current_angle_ += props.get<double>("angle_delta");

		InterfaceState state(scene);
		tf::quaternionEigenToMsg(q, pose.pose.orientation);
		state.properties().set("target_pose", pose);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setComment(std::to_string(current_angle_));

		// add frame at target pose
		rviz_marker_tools::appendFrame(trajectory.markers(), pose, 0.1, "touch frame");
		spawn(std::move(state), std::move(trajectory));
	}
}

} } }
