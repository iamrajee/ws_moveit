#include <moveit_task_constructor_demo/pick_place_task8.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//---------------------------------------------------------------------------------------------------------------------------------------------//
namespace moveit_task_constructor_demo
{ 


constexpr char LOGNAME[] = "pick_place_task";

//ClassX::MethodY(int arg1, int arg2) : arg1_(arg1), arg2_(arg2) {}                                   //overloading member of class
PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh) : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true) {}


// overloading the function                                                                           // ???? <======
                                                                         
	                                            /****************************************************\
-----------------------------------------------*                loadParameters                        *-------------------------------------------
	                                             ****************************************************/
std::string object2_name_,object2_reference_frame_;
geometry_msgs::Pose place_pose2_;
std::vector<double> object2_dimensions_;
std::string arm2_group_name_, hand2_group_name_,eef2_name_,hand2_frame_,hand2_open_pose_,hand2_close_pose_,arm2_home_pose_;

void PickPlaceTask::loadParameters() {
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
	ros::NodeHandle pnh("~");
	size_t errors = 0;

	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", grasp_frame_transform_);


	
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_); // Planning group properties
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", hand_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", hand_open_pose_); // Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose", hand_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", arm_home_pose_);
	
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm2_group_name", arm2_group_name_); // Planning group properties
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand2_group_name", hand2_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef2_name", eef2_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand2_frame", hand2_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand2_open_pose", hand2_open_pose_); // Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand2_close_pose", hand2_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm2_home_pose", arm2_home_pose_);

	// Target object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame_);

	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object2_name", object2_name_);                       // <=== new
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object2_dimensions", object2_dimensions_);           // <=== new
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object2_reference_frame", object2_reference_frame_); // <=== new
		
	
	// errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", surface_link_);
	std::string table_name,table2_name;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table2_name", table2_name);                           // <=== new
	support_surfaces_ = { table_name, table2_name };
	
	std::cout<<" ************* surface_link_ : "<<table_name<<"**************\n";
	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_min_dist", approach_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_max_dist", approach_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_min_dist", lift_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_max_dist", lift_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", place_surface_offset_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", place_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose2", place_pose2_);                           // <=== new

	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}


	                       	               /****************************************************\
------------------------------------------*                         init                         *---------------------------------------------
	                       	                ****************************************************/

void PickPlaceTask::init() {
	
    ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
	// const std::string object = "object";                           //!!! <=====error
	const std::string object = object_name_;
	const std::string object2 = object2_name_;

	// Reset ROS introspection before constructing the new object
	// TODO(henningkayser): verify this is a bug, fix if possible
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel();

	// Sampling planner
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
	// sampling_planner->setStepSize(.001); //not available

	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScaling(1.0);
	cartesian_planner->setMaxAccelerationScaling(1.0);
	cartesian_planner->setStepSize(.01);
	// cartesian_planner->setStepSize(.001); //slow

	// Set task properties
	t.setProperty("group", arm_group_name_);
	t.setProperty("eef", eef_name_);
	t.setProperty("hand", hand_group_name_);
	t.setProperty("hand_grasping_frame", hand_frame_);
	t.setProperty("ik_frame", hand_frame_);

	// Set task properties
	t.setProperty("group2", arm2_group_name_);
	t.setProperty("eef2", eef2_name_);
	t.setProperty("hand2", hand2_group_name_);
	t.setProperty("hand2_grasping_frame", hand2_frame_);
	t.setProperty("ik_frame2", hand2_frame_);
	
	Stage* current_state = nullptr;           //Forward current_state on to grasp pose generator
	Stage* current_state0 = nullptr;	
	Stage* attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator
	Stage* attach_object_stage0 = nullptr;

    // ====================== Current State ====================== //
	{
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		// current_state = _current_state.get();
		t.add(std::move(_current_state));
	}





	/* // <====== comment
	// ====================== test container ====================== //
	{ 
		// auto test_container = std::make_unique<SerialContainer>("test container"); //calculate and run both serially
		auto test_container = std::make_unique<Alternatives>("test container"); //calculate both and possible run both seperatly 
		// auto test_container = std::make_unique<Fallbacks>("test container"); //calculate and run either of
		// auto test_container = std::make_unique<Merger>("test container");//calculate and run both same time

		t.properties().exposeTo(test_container->properties(), { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
		test_container->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });

		{ // ************************************************************************************************************************************************************************************ //
			auto test_container2 = std::make_unique<SerialContainer>("test container2");  //worked fine !!

			test_container->properties().exposeTo(test_container2->properties(), { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
			test_container2->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });

			// ====================== Move to Home ====================== //
			{
				auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
				stage->setGroup(arm_group_name_);
				stage->setGoal(arm_home_pose_);
				stage->restrictDirection(stages::MoveTo::FORWARD);
				current_state = stage.get();
				test_container2->insert(std::move(stage));
			}

			// ====================== Open Hand ====================== //
			{
				auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
				stage->setGroup(hand_group_name_);
				stage->setGoal(hand_open_pose_);
				stage->restrictDirection(stages::MoveTo::FORWARD);
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}
			// ====================== Move to Pick ====================== //
			{
				auto stage = std::make_unique<stages::Connect>("move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
				stage->setTimeout(5.0);
				stage->properties().configureInitFrom(Stage::PARENT);
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}
			attach_object_stage0 = nullptr;
    		// ====================== Pick Object ====================== //
			{ 
				auto grasp = std::make_unique<SerialContainer>("pick object"); //container in which below stages with be inserted
	
				test_container2->properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
				grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
	
    		    // ---------------------- Approach Object ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
					stage->properties().set("marker_ns", "approach_object");
					stage->properties().set("link", hand_frame_);
					stage->properties().configureInitFrom(Stage::PARENT, { "group" });
					stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);
					// Set hand forward direction
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = hand_frame_;
					vec.vector.z = 1.0;
					stage->setDirection(vec);
					grasp->insert(std::move(stage));
				}
	
    		    // ---------------------- Generate Grasp Pose ---------------------- //
				{
					// Sample grasp pose
					auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
					stage->properties().configureInitFrom(Stage::PARENT);
					stage->properties().set("marker_ns", "grasp_pose");
					stage->setPreGraspPose(hand_open_pose_);
					stage->setObject(object);
					stage->setAngleDelta(M_PI / 24);
					stage->setMonitoredStage(current_state);  // Hook into current state
					// Compute IK
					auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
					wrapper->setMaxIKSolutions(8);
					wrapper->setMinSolutionDistance(1.0);
					wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
					wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
					wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
					grasp->insert(std::move(wrapper));
				}
	
    		    // ---------------------- Allow Collision (hand object) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
					stage->allowCollisions(object, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
					grasp->insert(std::move(stage));
				}
	
    		    // ---------------------- Close Hand ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
					// stage->properties().property("hand").configureInitFrom(Stage::PARENT, hand_group_name_); 														//"group" => "hand"
					// stage->properties().configureInitFrom(Stage::PARENT, { "group" });
					stage->setGroup(hand_group_name_);
					stage->setGoal(hand_close_pose_);
					grasp->insert(std::move(stage));
				}
	
    		    // ---------------------- Attach Object ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
					stage->attachObject(object, hand_frame_);
					attach_object_stage0 = stage.get();
					grasp->insert(std::move(stage));
				}
	
    		    // ---------------------- Allow collision (object support) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
					stage->allowCollisions({ object }, support_surfaces_, true);
					grasp->insert(std::move(stage));
				}
	
    		    // ---------------------- Lift object ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
					stage->properties().configureInitFrom(Stage::PARENT, { "group" });
					stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
					stage->setIKFrame(hand_frame_);
					stage->properties().set("marker_ns", "lift_object");
					// Set upward direction
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = world_frame_;
					vec.vector.z = 1.0;
					stage->setDirection(vec);
					grasp->insert(std::move(stage));
				}
	
    		    // ---------------------- Forbid collision (object support) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
					stage->allowCollisions({ object }, support_surfaces_, false);
					grasp->insert(std::move(stage));
				}
	
				// Add grasp container to task
				// test_container2.add(std::move(grasp));
				test_container2->insert(std::move(grasp));
			}
			// ====================== Move to Place ====================== //
			{
				auto stage = std::make_unique<stages::Connect>( "move to place", stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}} );
				stage->setTimeout(30.0);
				stage->properties().configureInitFrom(Stage::PARENT);
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}

		    // ====================== Place Object ====================== //
			{
				auto place = std::make_unique<SerialContainer>("place object");
				test_container2->properties().exposeTo(place->properties(), { "eef", "hand", "group" });
				place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

				// ---------------------- Allow collision (object support) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
					stage->allowCollisions({ object }, support_surfaces_, true);
					place->insert(std::move(stage));
				}

		        // ---------------------- Lower Object ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
					stage->properties().set("marker_ns", "lower_object");
					stage->properties().set("link", hand_frame_);
					stage->properties().configureInitFrom(Stage::PARENT, { "group" });
					stage->setMinMaxDistance(.03, .13);
					// Set downward direction
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = world_frame_;
					vec.vector.z = -1.0;
					stage->setDirection(vec);
					place->insert(std::move(stage));
				}

		        // ---------------------- Generate Place Pose ---------------------- //
				{
					// Generate Place Pose
					auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
					stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
					stage->properties().set("marker_ns", "place_pose");
					stage->setObject(object);
					// Set target pose
					geometry_msgs::PoseStamped p;
					p.header.frame_id = object_reference_frame_;
					p.pose = place_pose_;
					p.pose.position.z += 0.5 * object_dimensions_[0] + place_surface_offset_; //placing com at half height of same object so that bottom surface touch ground
					// p.pose.position.z += 0.5 * object_dimensions_[0] - place_surface_offset_;
					stage->setPose(p);
					stage->setMonitoredStage(attach_object_stage0);  // Hook into attach_object_stage
					// stage->setMonitoredStage(current_state);
					// Compute IK
					auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
					wrapper->setMaxIKSolutions(2);
					wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
					wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
					wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
					place->insert(std::move(wrapper));
				}

		        // ---------------------- Detach Object ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
					stage->detachObject(object_name_, hand_frame_);
					place->insert(std::move(stage));
				}

		        // ---------------------- Open Hand ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
					// stage->properties().property("group").configureInitFrom(Stage::PARENT, hand_group_name_);																	//??
					stage->setGroup(hand_group_name_);
					stage->setGoal(hand_open_pose_);
					place->insert(std::move(stage));
				}

		        // ---------------------- Forbid collision (hand, object) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
					stage->allowCollisions(object_name_, t.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(), false);
					place->insert(std::move(stage));
				}

		        // ---------------------- Retreat Motion ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
					stage->properties().configureInitFrom(Stage::PARENT, { "group" });
					stage->setMinMaxDistance(.12, .25);
					stage->setIKFrame(hand_frame_);
					stage->properties().set("marker_ns", "retreat");
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = hand_frame_;
					vec.vector.z = -1.0;
					stage->setDirection(vec);
					place->insert(std::move(stage));
				}

				// ---------------------- Close Hand ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
					// stage->properties().property("group").configureInitFrom(Stage::PARENT, hand_group_name_);
					stage->setGroup(hand_group_name_);
					stage->setGoal(hand_close_pose_);
					// current_state0 = stage.get();              //<==doesn't work
					place->insert(std::move(stage));
				}

				// Add place container to task
				// test_container2.add(std::move(place));
				test_container2->insert(std::move(place));
			}



		    // ====================== Move to Home ====================== //
			{
				auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
				stage->properties().configureInitFrom(Stage::PARENT, { "group" });
				stage->setGoal(arm_home_pose_);
				stage->restrictDirection(stages::MoveTo::FORWARD);
				// current_state = stage.get();
				// current_state0 = stage.get();
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}

			test_container->insert(std::move(test_container2));
		} // ************************************************************************************************************************************************************************************ //
		

		  // ************************************************************************************************************************************************************************************ //
		  // ************************************************************************************************************************************************************************************ //
		  // ************************************************************************************************************************************************************************************ //
		  // ************************************************************************************************************************************************************************************ //
		  // ************************************************************************************************************************************************************************************ //
		
		
		{ // ************************************************************************************************************************************************************************************ //
			auto test_container2 = std::make_unique<SerialContainer>("test container2");  //worked fine !!
			
			test_container->properties().exposeTo(test_container2->properties(), { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
			test_container2->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });

			// ====================== Move to Home 2 ====================== //
			{
				auto stage = std::make_unique<stages::MoveTo>("move home2", sampling_planner);
				stage->setGroup(arm2_group_name_);
				stage->setGoal(arm2_home_pose_);
				stage->restrictDirection(stages::MoveTo::FORWARD);
				current_state = stage.get();
				test_container2->insert(std::move(stage));
			}

			// ====================== Open Hand ====================== //
			{
				auto stage = std::make_unique<stages::MoveTo>("open hand2", sampling_planner);
				stage->setGroup(hand2_group_name_);
				stage->setGoal(hand2_open_pose_);
				stage->restrictDirection(stages::MoveTo::FORWARD);
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}

			// ====================== Move to Pick ====================== //
			{
				auto stage = std::make_unique<stages::Connect>("move to pick2", stages::Connect::GroupPlannerVector{ { arm2_group_name_, sampling_planner } });
				stage->setTimeout(20.0);
				stage->properties().configureInitFrom(Stage::PARENT);
				// stage->setMonitoredStage(current_state0);
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}

		    attach_object_stage = nullptr;  // Forward attach_object_stage to place pose generator

		    // ====================== Pick Object ====================== //
			{ 
				auto grasp = std::make_unique<SerialContainer>("pick object2"); //container in which below stages with be inserted

				test_container2->properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
				grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
				// grasp->setMonitoredStage(current_state0);
		        // ---------------------- Approach Object ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("approach object2", cartesian_planner);
					stage->properties().set("marker_ns", "approach_object");
					stage->properties().set("link", hand2_frame_);
					stage->properties().configureInitFrom(Stage::PARENT, { "group", "group2" });
					stage->properties().property("group").configureInitFrom(Stage::PARENT, "group2");
					stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);
					// stage->setMonitoredStage(current_state0);
					// Set hand2 forward direction
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = hand2_frame_;
					vec.vector.z = 1.0;
					stage->setDirection(vec);
					grasp->insert(std::move(stage));
				}

		        // ---------------------- Generate Grasp Pose ---------------------- //
				{
					// Sample grasp pose
					auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose2");
					stage->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
					stage->properties().set("marker_ns", "grasp_pose");
					stage->setPreGraspPose(hand2_open_pose_);
					stage->setObject(object2);
					stage->setAngleDelta(M_PI / 12);
					stage->setMonitoredStage(current_state);  // Hook into current state

					// Compute IK
					auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK2", std::move(stage));
					wrapper->setMaxIKSolutions(8);
					wrapper->setMinSolutionDistance(1.0);
					wrapper->setIKFrame(grasp_frame_transform_, hand2_frame_);
					wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
					wrapper->properties().property("eef").configureInitFrom(Stage::PARENT, "eef2");
					wrapper->properties().property("group").configureInitFrom(Stage::PARENT, "group2");
					wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
					grasp->insert(std::move(wrapper));
				}

		        // ---------------------- Allow Collision (hand2 object2) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand2,object2)2");
					stage->allowCollisions(object2, t.getRobotModel()->getJointModelGroup(hand2_group_name_)->getLinkModelNamesWithCollisionGeometry(),true);
					grasp->insert(std::move(stage));
				}

		        // ---------------------- Close Hand ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveTo>("close hand2", sampling_planner);
					stage->setGroup(hand2_group_name_);
					stage->setGoal(hand2_close_pose_);
					grasp->insert(std::move(stage));
				}

		        // ---------------------- Attach Object ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object2");
					stage->attachObject(object2, hand2_frame_);
					attach_object_stage = stage.get();
					grasp->insert(std::move(stage));
				}

		        // ---------------------- Allow collision (object2 support) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object2,support)2");
					stage->allowCollisions({ object2 }, support_surfaces_, true);
					grasp->insert(std::move(stage));
				}

		        // ---------------------- Lift object2 ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("lift object2", cartesian_planner);
					stage->setGroup(arm2_group_name_);
					stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
					stage->setIKFrame(hand2_frame_);
					stage->properties().set("marker_ns", "lift_object");
					// Set upward direction
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = world_frame_;
					vec.vector.z = 1.0;
					stage->setDirection(vec);
					grasp->insert(std::move(stage));
				}

		        // ---------------------- Forbid collision (object2 support) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object2,surface)2");
					stage->allowCollisions({ object2 }, support_surfaces_, false);
					grasp->insert(std::move(stage));
				}

				// Add grasp container to task
				test_container2->insert(std::move(grasp));
			}

		    // ====================== Move to Place ====================== //
			{
				auto stage = std::make_unique<stages::Connect>( "move to place2", stages::Connect::GroupPlannerVector{{arm2_group_name_, sampling_planner}} );
				stage->setTimeout(5.0);
				stage->properties().configureInitFrom(Stage::PARENT);
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}

		    // ====================== Place Object ====================== //
			{
				auto place = std::make_unique<SerialContainer>("place object2");
				test_container2->properties().exposeTo(place->properties(), { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
				place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });

				// ---------------------- Allow collision (object2 support) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object2,support)2");
					stage->allowCollisions({ object2 }, support_surfaces_, true);
					place->insert(std::move(stage));
				}

		        // ---------------------- Lower Object ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("lower object2", cartesian_planner);
					stage->properties().set("marker_ns", "lower_object");
					stage->properties().set("link", hand2_frame_);
					stage->properties().configureInitFrom(Stage::PARENT, { "group", "group2" });
					stage->properties().property("group").configureInitFrom(Stage::PARENT, "group2");
					stage->setMinMaxDistance(.03, .13);
					// Set downward direction
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = world_frame_;
					vec.vector.z = -1.0;
					stage->setDirection(vec);
					place->insert(std::move(stage));
				}

		        // ---------------------- Generate Place Pose ---------------------- //
				{
					// Generate Place Pose
					auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose2");
					stage->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });
					stage->properties().set("marker_ns", "place_pose");
					stage->setObject(object2);
					// Set target pose
					geometry_msgs::PoseStamped p;
					p.header.frame_id = world_frame_;
					p.pose = place_pose2_;
					// p.pose.position.z += (2 * object_dimensions_[0] + 2*place_surface_offset_); //placing com at half height of same object2 so that bottom surface touch ground
					p.pose.position.z += place_surface_offset_; //+0.5 * object_dimensions_[0]
					stage->setPose(p);
					stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage
					// Compute IK
					auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK2", std::move(stage));
					wrapper->setMaxIKSolutions(2);
					wrapper->setIKFrame(grasp_frame_transform_, hand2_frame_);
					// wrapper->properties().configureInitFrom(Stage::PARENT, { "eef2", "group2" });
					wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame","eef2", "hand2", "group2", "ik_frame2" });

					wrapper->properties().property("eef").configureInitFrom(Stage::PARENT, "eef2");
					wrapper->properties().property("group").configureInitFrom(Stage::PARENT, "group2");
					wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
					place->insert(std::move(wrapper));
				}

		        // ---------------------- Detach Object ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object2");
					stage->detachObject(object2_name_, hand2_frame_);
					place->insert(std::move(stage));
				}

		        // ---------------------- Open Hand ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveTo>("open hand2", sampling_planner);
					stage->setGroup(hand2_group_name_);
					stage->setGoal(hand2_open_pose_);
					place->insert(std::move(stage));
				}

		        // ---------------------- Forbid collision (hand2, object2) ---------------------- //
				{
					auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand2,object2)2");
					stage->allowCollisions(object2_name_, t.getRobotModel()->getJointModelGroup(hand2_group_name_)->getLinkModelNamesWithCollisionGeometry(), false);
					place->insert(std::move(stage));
				}

		        // ---------------------- Retreat Motion ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveRelative>("retreat after place2", cartesian_planner);
					stage->properties().configureInitFrom(Stage::PARENT, { "group", "group2" });
					stage->properties().property("group").configureInitFrom(Stage::PARENT, "group2");
					stage->setMinMaxDistance(.12, .25);
					stage->setIKFrame(hand2_frame_);
					stage->properties().set("marker_ns", "retreat");
					geometry_msgs::Vector3Stamped vec;
					vec.header.frame_id = hand2_frame_;
					vec.vector.z = -1.0;
					stage->setDirection(vec);
					place->insert(std::move(stage));
				}

				// ---------------------- Close Hand ---------------------- //
				{
					auto stage = std::make_unique<stages::MoveTo>("close hand2", sampling_planner);
					stage->setGroup(hand2_group_name_);
					stage->setGoal(hand2_close_pose_);
					place->insert(std::move(stage));
				}

				// Add place container to task
				// t.add(std::move(place));
				test_container2->insert(std::move(place));
			}

		    // ====================== Move to Home ====================== //
			{
				auto stage = std::make_unique<stages::MoveTo>("move home2", sampling_planner);
				stage->setGroup(arm2_group_name_);
				stage->setGoal(arm2_home_pose_);
				stage->restrictDirection(stages::MoveTo::FORWARD);
				// t.add(std::move(stage));
				test_container2->insert(std::move(stage));
			}			

			test_container->insert(std::move(test_container2));
		}// ************************************************************************************************************************************************************************************ //

		t.add(std::move(test_container));
	}

	*/ // <====== comment

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Merger example ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //
	// /* // <====== comment
	
	// ====================== test container ====================== //
	{ 
		// auto test_container = std::make_unique<SerialContainer>("test container"); //calculate and run both serially
		// auto test_container = std::make_unique<Alternatives>("test container"); //calculate both and possible run both seperatly 
		// auto test_container = std::make_unique<Fallbacks>("test container"); //calculate and run either of
		auto test_container = std::make_unique<Merger>("test container");//calculate and run both same time

		// ====================== Move to Home ====================== //
		{
			auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
			stage->setGroup(arm_group_name_);
			stage->setGoal(arm_home_pose_);
			stage->restrictDirection(stages::MoveTo::FORWARD);
			test_container->insert(std::move(stage));
		}

		// ====================== Move to Home 2 ====================== //
		{
			auto stage = std::make_unique<stages::MoveTo>("move home2", sampling_planner);
			stage->setGroup(arm2_group_name_);
			stage->setGoal(arm2_home_pose_);
			stage->restrictDirection(stages::MoveTo::FORWARD);
			test_container->insert(std::move(stage));
		}

		t.add(std::move(test_container)); //t is task
	}

    //====================== test container ====================== //
	{ 
		// auto test_container = std::make_unique<SerialContainer>("test container"); //calculate and run both serially
		// auto test_container = std::make_unique<Alternatives>("test container"); //calculate both and possible run both seperatly 
		// auto test_container = std::make_unique<Fallbacks>("test container"); //calculate and run either of
		auto test_container = std::make_unique<Merger>("test container");//calculate and run both same time

		// ====================== Open Hand 1 ====================== //
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
			stage->setGroup(hand_group_name_);
			stage->setGoal(hand_open_pose_);
			stage->restrictDirection(stages::MoveTo::FORWARD);
			// t.add(std::move(stage));
			test_container->insert(std::move(stage));
		}

		// ====================== Open Hand 2 ====================== //
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand2", sampling_planner);
			stage->setGroup(hand2_group_name_);
			stage->setGoal(hand2_open_pose_);
			stage->restrictDirection(stages::MoveTo::FORWARD);
			// t.add(std::move(stage));
			test_container->insert(std::move(stage));	
		}

		t.add(std::move(test_container)); //t is task
	}


	// */ // <====== comment
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //

}


	                       	               /****************************************************\
------------------------------------------*                      plan                            *-----------------------------------------
	                       	                ****************************************************/
bool PickPlaceTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	ros::NodeHandle pnh("~");
	int planning_attempts = pnh.param<int>("planning_attempts", 10); // !! here 10 is only default value

	try {
		task_->plan(planning_attempts);
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false; 
	}
	if (task_->numSolutions() == 0) {
		ROS_ERROR_NAMED(LOGNAME, "Planning failed");
		return false;
	}
	return true;
}
 
	                        	           /****************************************************\
------------------------------------------*                      execute                         *-----------------------------------------------
	                       	                ****************************************************/
                                              
bool PickPlaceTask::execute() {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	task_->solutions().front()->fillMessage(execute_goal.solution);
	execute_.sendGoal(execute_goal);
	execute_.waitForResult();
	moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
		return false;
	}

	return true;
}



}
//---------------------------------------------------------------------------------------------------------------------------------------------//