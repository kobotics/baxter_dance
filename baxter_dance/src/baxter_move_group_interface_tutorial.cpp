/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta */

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <stdio.h>
#include <fstream>
#include <stdlib.h>

 #include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  // sleep(20.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  std::string group_name = "left_arm";
  moveit::planning_interface::MoveGroup group(group_name);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
  
  //std::vector<double> joint_values_vector;
  double joint_values_array[] = {-1.1435826760620118, 0.5246214288574219, -0.5851651633789063, -0.3985437554931641, -0.0038349519653320314, 0.8448399179626466, -0.29874275809936524};
  double joint_values_left[12] = {0,0,-1.1435826760620118, 0.5246214288574219, -0.5851651633789063, -0.3985437554931641, -0.0038349519653320314, 0.8448399179626466, -0.29874275809936524,0,0,0};

  std::vector<double> joint_values(joint_values_array, joint_values_array + sizeof(joint_values_array) / sizeof(double));
  //std::vector<double> joint_values;

   // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First, we will define the collision object message.
  moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = group.getPlanningFrame();

  // /* The id of the object is used to identify it. */
  // collision_object.id = "box1";

  // /* Define a box to add to the world. */
  // /* Define a box to add to the world. */
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 0.4;
  // primitive.dimensions[1] = 0.1;
  // primitive.dimensions[2] = 0.4;

  // /* A pose for the box (specified relative to frame_id) */
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x =  0.6;
  // box_pose.position.y = -0.4;
  // box_pose.position.z =  1.2;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;  
  // collision_objects.push_back(collision_object);  

  // // Now, let's add the collision object into the world
  // ROS_INFO("Add an object into the world");  
  // planning_scene_interface.addCollisionObjects(collision_objects);
  
  // /* Sleep so we have time to see the object in RViz */
  // sleep(5.0);



  // Random positions
  // distance in radians to set to random position
  double distance = 0.314;
  
  
  //Robot state
  robot_state::RobotStatePtr RobotStateTarget = group.getCurrentState();
  //RobotStateTarget.setToDefaultValues();
  
  // RobotStateTarget.setJointGroupPositions(RobotStateTarget.getRobotModel()->getJointModelGroup(group.getName()), joint_values_left);
  RobotStateTarget->setJointGroupPositions(RobotStateTarget->getJointModelGroup("left_arm"), joint_values);
  ROS_INFO("hi1\n");
  std::vector<double> joint_values_display;
  RobotStateTarget->copyJointGroupPositions(RobotStateTarget->getJointModelGroup("left_arm"), joint_values_display);
  // current_state.copyJointGroupPositions(joint_model_group_right, joint_values_right);
  for (std::vector<double>::const_iterator i = joint_values_display.begin();i != joint_values_display.end(); ++i)
    {
      ROS_INFO("%f",*i);
    }
   // for(std::size_t i = 0; i < 12; ++i)
   //  {
   //    ROS_INFO(" %f", joint_values_left[i]);
   //  }
// //Read 
  // robot_state::RobotState& RobotStateStart(*group.getCurrentState());
  // RobotStateStart.setJointGroupPositions(RobotStateStart.getRobotModel()->getJointModelGroup(group.getName()), joint_values_left);
  ROS_INFO("hi2\n");
  // const robot_model::JointModelGroup* joint_model_group =
  //   group.getCurrentState()->getJointModelGroup();
  // group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())
  
  // group.setStartState(RobotStateStart);
  
  RobotStateTarget->setToRandomPositionsNearBy(group.getCurrentState()->getJointModelGroup(group.getName()), *RobotStateTarget, distance);
  
  // RobotStateTarget.setToRandomPositions();
  std::vector<double> group_variable_values;
  RobotStateTarget->copyJointGroupPositions(RobotStateTarget->getJointModelGroup(group.getName()), group_variable_values);
  

  //assign to target
  group.setJointValueTarget(group_variable_values);
  
  
  for (std::vector<double>::const_iterator i = group_variable_values.begin();i != group_variable_values.end(); ++i) //group_variable_values.end(); ++i)
    {
      ROS_INFO("%f",*i);
    }
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  // ROS_INFO(my_plan.trajectory_);
  /* Sleep to give Rviz time to visualize the plan. */
    if (1)
  {
    ROS_INFO("Visualizing plan 2 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  }
  sleep(5.0);

  ros::shutdown();  
  return 0;
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = 0.28;
  // target_pose1.position.y = -0.7;
  // target_pose1.position.z = 1.0;
  // group.setPoseTarget(target_pose1);


  // // Now, we call the planner to compute the plan
  // // and visualize it.
  // // Note that we are just planning, not asking move_group 
  // // to actually move the robot.
  // moveit::planning_interface::MoveGroup::Plan my_plan;
  // bool success = group.plan(my_plan);

  // ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  // /* Sleep to give Rviz time to visualize the plan. */
  // sleep(5.0);

  // // Visualizing plans
  // // ^^^^^^^^^^^^^^^^^
  // // Now that we have a plan we can visualize it in Rviz.  This is not
  // // necessary because the group.plan() call we made above did this
  // // automatically.  But explicitly publishing plans is useful in cases that we
  // // want to visualize a previously created plan.
  // if (1)
  // {
  //   ROS_INFO("Visualizing plan 1 (again)");    
  //   display_trajectory.trajectory_start = my_plan.start_state_;
  //   display_trajectory.trajectory.push_back(my_plan.trajectory_);
  //   display_publisher.publish(display_trajectory);
  //   /* Sleep to give Rviz time to visualize the plan. */
  //   sleep(5.0);
  // }
  
  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active 
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is 
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.
 
  /* Uncomment below line when working with a real robot*/
  /* group.move() */

  // Planning to a joint-space goal 
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // First get the current set of joint values for the group.
  
  // for (std::vector<double>::const_iterator i = group_variable_values.begin();i != group_variable_values.end(); ++i)
  //   {
  //     ROS_INFO("%f",*i);
  //   }
  // std::vector<std::string> group_variable_names_model = group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName())
  // std::vector<std::string> group_variable_names = group.getJointValueTarget().getJointModelGroup(group.getName())->getVariableNames();
  // for (std::vector<double>::const_iterator i = group_variable_values.begin(); i != group_variable_values.end(); ++i)
  //   {
  //     ROS_INFO("%f",*i);
  //   }
    // for (std::vector<std::string>::const_iterator i = group_variable_names.begin(); i != group_variable_names.end(); ++i)
    // {
    //   // std::string s = "Hi";
    //   // std::cout << s << std::endl;
    //  //const char *cstr = *i.c_str();
    //   std::string s = *i;
    //   ROS_INFO("%s",s.c_str());
    //   //printf("%s\n", s);
    // }
    // for (std::vector<std::string>::const_iterator i = group_variable_names_model.begin(); i != group_variable_names_model.end(); ++i)
    // {
    //   ROS_INFO("%s",*i);
    // }
  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan.
  // double joint_values_array[] = {-1.1435826760620118, 0.5246214288574219, -0.5851651633789063, -0.3985437554931641, -0.0038349519653320314, 0.8448399179626466, -0.29874275809936524,0,0,0};
  // ,0,0.9568205153503418, 2.077009984423828, 0.2538738201049805, -1.282024442010498, -0.7528010707946777, 0.019941750219726564, 0.5648884244934083,0,0,0};
  
  // group_variable_values = joint_values;  
  group.setJointValueTarget(group_variable_values);
  // for (std::vector<double>::const_iterator i = group_variable_values.begin();i != group_variable_values.end(); ++i)
  //   {
  //     ROS_INFO("%f",*i);
  //   }
  // geometry_msgs::PoseStamped pose = group.getPoseTarget();


  // for (std::vector<double>::const_iterator i = group_variable_values.begin();i != group_variable_values.end(); ++i)
  //   {
  //     ROS_INFO("%f",*i);
  //   }

  // const std::vector<std::string> &joint_names_left = joint_model_group_left->getJointModelNames();
  // const std::vector<std::string> &joint_names_right = joint_model_group_right->getJointModelNames();

  // for(std::size_t i = 0; i < joint_names_left.size(); ++i)
  //   {
  //     ROS_INFO("Joint %s: %f", joint_names_left[i].c_str(), joint_values_left[i]);
  //   }
  // const std::vector<std::string> &joint_names_left = joint_model_group_left->getJointModelNames();
  // const std::vector<std::string> &joint_names_right = joint_model_group_right->getJointModelNames();

  // for(std::size_t i = 0; i < joint_names_left.size(); ++i)
  //   {
  //     ROS_INFO("Joint %s: %f", joint_names_left[i].c_str(), joint_values_left[i]);
  //   }
  // moveit::planning_interface::MoveGroup::Plan my_plan;
  // bool success = group.plan(my_plan);
  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  // ROS_INFO(my_plan.trajectory_);
  /* Sleep to give Rviz time to visualize the plan. */
    if (1)
  {
    ROS_INFO("Visualizing plan 2 (again)");    
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);
  }
  sleep(5.0);


  ros::shutdown();  
  return 0;

  std::string line;
  // bool success = false;
  std::ifstream file("/home/kim/baxter/ros_ws/src/baxter_dance/baxter_dance/src/simple2.txt");
  ROS_INFO_STREAM(file.is_open());
  int i =0,line_no =0;
  if(file.is_open())
  {
    ROS_INFO_STREAM("Open");
  }
  std::getline(file,line);
  
  while(!file.eof() && std::getline(file,line))
  {
  line_no++;
  //ROS_INFO_STREAM(line_no);
    std::stringstream   linestream(line);
    std::string         value;
    double temp;
    i =0;
    joint_values.clear();
    //collision_result.clear();
    joint_values.push_back(0);
    joint_values.push_back(0);
    while(getline(linestream,value,','))
    {
      //ROS_INFO_STREAM(i);
        if((i>0 && i<8) || (i>8 && i<16))
        {
          temp = atof(value.c_str());
          joint_values.push_back(temp);
        }
        else if(i==8)
        {
          joint_values.push_back(0);
          joint_values.push_back(0);
          joint_values.push_back(0);
          joint_values.push_back(0);
        }
        else if(i==16)
        {
          joint_values.push_back(0);
          joint_values.push_back(0);
          joint_values.push_back(0);
        }
        i++;
    }
    group.setJointValueTarget(joint_values);
    success = group.plan(my_plan);
    ROS_INFO("Visualizing plan %d (joint space goal) %s",line_no,success?"success":"FAILED");
    
  }
  file.close();

  ros::shutdown();  
  return 0;
  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  moveit_msgs::OrientationConstraint ocm;  
  ocm.link_name = "left_hand";  
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  
  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);  
  group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already 
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose. 
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);
  
  // Now we will plan to the earlier pose target from the new 
  // start state that we have just created.
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(10.0);

  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints 
  // for the end-effector to go through. Note that we are starting 
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list.
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose3 = start_pose2;
  target_pose3.position.x += 0.2;
  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up and out

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z -= 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // down and right (back to start)

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);    
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(15.0);


  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First, we will define the collision object message.
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = group.getPlanningFrame();

  // /* The id of the object is used to identify it. */
  // collision_object.id = "box1";

  // /* Define a box to add to the world. */
  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 0.4;
  // primitive.dimensions[1] = 0.1;
  // primitive.dimensions[2] = 0.4;

  // /* A pose for the box (specified relative to frame_id) */
  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x =  0.6;
  // box_pose.position.y = -0.4;
  // box_pose.position.z =  1.2;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // std::vector<moveit_msgs::CollisionObject> collision_objects;  
  // collision_objects.push_back(collision_object);  

  // // Now, let's add the collision object into the world
  // ROS_INFO("Add an object into the world");  
  // planning_scene_interface.addCollisionObjects(collision_objects);
  
  // /* Sleep so we have time to see the object in RViz */
  // sleep(2.0);

  // Planning with collision detection can be slow.  Lets set the planning time
  // to be sure the planner has enough time to plan around the box.  10 seconds
  // should be plenty.
  group.setPlanningTime(10.0);


  // Now when we plan a trajectory it will avoid the obstacle
  group.setStartState(*group.getCurrentState());
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
    success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(10.0);
  

  // Now, let's attach the collision object to the robot.
  ROS_INFO("Attach the object to the robot");  
  group.attachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(4.0);


  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");  
  group.detachObject(collision_object.id);  
  /* Sleep to give Rviz time to show the object detached. */
  sleep(4.0);


  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the object from the world");  
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);  
  planning_scene_interface.removeCollisionObjects(object_ids);
  /* Sleep to give Rviz time to show the object is no longer there. */
  sleep(4.0);


  // Dual-arm pose goals
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First define a new group for addressing the two arms. Then define 
  // two separate pose goals, one for each end-effector. Note that 
  // we are reusing the goal for the right arm above
  moveit::planning_interface::MoveGroup two_arms_group("arms");

  two_arms_group.setPoseTarget(target_pose1, "left_hand");

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.7;
  target_pose2.position.y = 0.15;
  target_pose2.position.z = 1.0;

  two_arms_group.setPoseTarget(target_pose2, "left_hand");

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroup::Plan two_arms_plan;
  two_arms_group.plan(two_arms_plan);
  sleep(4.0);

// END_TUTORIAL

  ros::shutdown();  
  return 0;
}
