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

/* Authors: Michael Goerner, Henning Kayser
   Desc:    Pour from attached bottle into(onto) an object
*/

#pragma once

#include <moveit/task_constructor/stage.h>

// TODO make use of this?
//#include <moveit/task_constructor/solvers/planner_interface.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <Eigen/Dense>

namespace mtc_pour {
using namespace moveit::task_constructor;
//TODO: should this be factorized in containers + MoveTo with multi-points?

/** Perform a Pouring Motion for attached "bottle" into a "container" */
class PourInto : public PropagatingForward {
public:
	PourInto(std::string name);

	void computeForward(const InterfaceState& from) override;

	void setGroup(std::string group_name){
		setProperty("group", std::move(group_name));
	}

	void setBottle(std::string bottle_name){
		setProperty("bottle", std::move(bottle_name));
	}

	void setContainer(std::string container){
		setProperty("container", std::move(container));
	}

	void setPourOffset(Eigen::Vector3d offset){
		setProperty("pour_offset", std::move(offset));
	}

	void setTiltAngle(double tilt_angle){
		setProperty("tilt_angle", tilt_angle);
	}

	void setPouringAxis(geometry_msgs::Vector3Stamped axis){
		setProperty("pouring_axis", std::move(axis));
	}

	void setPourDuration(ros::Duration d){
		setProperty("pour_duration", d);
	}

	void setWaypointDuration(ros::Duration d){
		setProperty("waypoint_duration", d);
	}

	void setMinPathFraction(double min_path_fraction){
		setProperty("min_path_fraction", min_path_fraction);
	}

	void setWaypointCount(size_t cnt){
		setProperty("waypoint_count", cnt);
	}
protected:
	void compute(const InterfaceState& state, planning_scene::PlanningScenePtr &scene,
	             SubTrajectory &trajectory);
};

}
