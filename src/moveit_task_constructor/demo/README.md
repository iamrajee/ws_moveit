# moveit_task_constructor_demo

Description: A simple pick & place demo using MoveIt Task Constructor. This uses the Panda from Franka Emika

Developed by Henning Kayser & Simon Goldstein at [PickNik Consulting](http://picknik.ai/)

## Run

*************************************** Run demo ***************************************

======================================== PickPlace ========================================
Default by MTC
    roslaunch moveit_task_constructor_demo demo.launch
    roslaunch moveit_task_constructor_demo pickplace.launch

2 - Custom formating with same code
    roslaunch moveit_task_constructor_demo demo2.launch
    roslaunch moveit_task_constructor_demo pickplace2.launch

3 - panda : two object, two cylider

4 - TODO : panda : for abstract pick place for multiple object

5 - individually two_panda(comment various ways whichh didn't worked)

6 - individually two_panda(clearner)
 
7 - serial two_panda

8 - Parallelising two_panda simple (Merger)

9 - panda single arm pouring

10 - two_panda arm pouring ground 

11 - two_panda arm pouring, glass on hight, arm sysmetric about origin

12 - two_panda arm pouring(cleaned)

13 - two_panda arm pickplace for differnt object

14 - two_panda arm pickplace for using differnt arm for same object

15 - TODO : multi task two arm pick place

======================================== Cartesian ========================================
Default by MTC
    roslaunch moveit_task_constructor_demo demo.launch
    rosrun moveit_task_constructor_demo cartesian

2. formating

3. two task with differnt node name

=>  roslaunch moveit_task_constructor_demo demo.launch
    rosrun moveit_task_constructor_demo cartesian2
    rosrun moveit_task_constructor_demo cartesian3



======================================== Modular ========================================
Default by MTC
    roslaunch moveit_task_constructor_demo demo.launch
    rosrun moveit_task_constructor_demo modular

2. formating
=========================================================================================
Added "enforce_joint_model_state_space: true #added newenforce_joint_model_state_space: true #added new" in /home/rajendra/ws_moveit/src/two_panda_moveit_config/config/ompl_planning.yaml