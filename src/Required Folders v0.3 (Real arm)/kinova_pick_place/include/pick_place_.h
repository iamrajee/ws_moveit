#ifndef PICK_PLACE_H
#define PICK_PLACE_H

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float32MultiArray.h>


namespace kinova
{


    class PickPlace
    {
		
    public:
        PickPlace(ros::NodeHandle &nh);
        ~PickPlace();
    private:
        ros::NodeHandle nh_;

        moveit::planning_interface::MoveGroupInterface* group_;
        moveit::planning_interface::PlanningSceneInterface* planning_scene_interface_;

        moveit_msgs::AttachedCollisionObject attached_object;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        
        void pick();
        void place();
        void addCollisionObjects();
        void clear_target();
        void check_constrain();
        void openGripper(trajectory_msgs::JointTrajectory& posture);
        void closedGripper(trajectory_msgs::JointTrajectory& posture);
    };
}

ros::Subscriber box_dim_sub;
ros::Subscriber box_pose_sub;
ros::Subscriber box_rpy_sub;

bool success;
float dim[3], pose[3],rpy[3], _z;
char axis[] ={'x','y','z'};
char _rpy[] ={'r','p','y'};
float close_pose_1, close_pose_2;
float object_x, object_y, object_z,rpy_z;
float grasp_offset = -0.01;
float close_offset;


#endif
