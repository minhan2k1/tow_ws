# tow_ws
ros workspace containing packages used at work

description_v2 package: contains robot urdf and launch files which launches the robot in a gazebo simulation along with rgbd camera and controllable joints

description_v2_moveit: using moveit to motion plan and control the robot (launch demo_gazebo.launch to launch rviz and gazebo and control the robot there)

robot_control: contains 2 scripts which once run after demo_gazebo.launch has been executed will get the robot to sit or stand.
