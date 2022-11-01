# tow_ws
ros workspace containing packages used at work

copy package folders and run (in workspace's directory):
  
    catkin_make
  
to make the workspace then (in workspace's directory):

    source devel/setup.bash

description_v2 package: contains robot urdf and launch files which launches the robot in a gazebo simulation along with rgbd camera and controllable joints

description_v2_moveit package: using moveit to motion plan and control the robot (launch demo_gazebo.launch to launch rviz and gazebo and control the robot there)

robot_control package: contains 2 scripts which once run after demo_gazebo.launch has been executed will get the robot to sit or stand.

launching:
  launch robot with camera and moveit planner:
  
    roslaunch description_v2_moveit demo_gazebo.launch
    
(optional) then:

    rosrun robot_control robot_stand
to stand

    rosrun robot_control robot_sit
    
to sit
in rviz add image and pointcloud2 with corresponding topics to view camera inputs
    
launch robot in gazebo with controllable joints but without moveit 

    roslaunch description_v2 gazebo.launch
    
  in rviz add image and pointcloud2 with corresponding topics to view camera inputs
  
  in rviz add marker array with its topic to view the map generated by octomap
