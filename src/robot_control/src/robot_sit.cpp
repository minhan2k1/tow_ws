#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <string>
#include <map>
#include <vector>

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_sit");

    std::map<std::string, double> target_position;
    
    target_position["j_flhr"] = 0;
    target_position["j_flhp"] = 0;
    target_position["j_flkp"] = 0;
    target_position["j_rlhr"] = 0;
    target_position["j_rlhp"] = 0;
    target_position["j_rlkp"] = 0;
    target_position["j_frhr"] = 0;
    target_position["j_frhp"] = 0;
    target_position["j_frkp"] = 0;
    target_position["j_rrhr"] = 0;
    target_position["j_rrhp"] = 0;
    target_position["j_rrkp"] = 0;
    


    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string PLANNING_GROUP = "leg_all";

    moveit::planning_interface::MoveGroupInterface group_leg_all(PLANNING_GROUP);

    const moveit::core::JointModelGroup* joint_model_group = group_leg_all.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    //moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

    //visual_tools.trigger();
    //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    std::vector<std::string> joint_names;
    joint_names = group_leg_all.getJoints();

    group_leg_all.setStartStateToCurrentState();

    group_leg_all.setMaxVelocityScalingFactor(1.0);

    for(int i = 0; i < joint_names.size(); i++){
        if(target_position.count(joint_names[i]) > 0){
            ROS_INFO_STREAM("\t" << joint_names[i] << " goal position: " << target_position[joint_names[i]]);
            group_leg_all.setJointValueTarget(joint_names[i], target_position[joint_names[i]]);
        }
    } 

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    group_leg_all.setPlanningTime(5.0);

    bool success = (group_leg_all.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if ( !success )
    throw std::runtime_error("No plan found");

    ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

    /*visual_tools.deleteAllMarkers();
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();*/

    ros::Time start = ros::Time::now();

    group_leg_all.move();

    ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

    spinner.stop();

    return EXIT_SUCCESS;
}
