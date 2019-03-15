# Robotic Arm Pick and Place #
#### Robotics Software Engineering ####
#### Term 1 ####
#### Project 2 ####

_Concepts/Skills Learned:_
  * ROS: Turtlesim, Catkin workspace, packages, nodes
  * Linux command line
  * VMWare (Workstation Player)
  * Forward and inverse kinematics
  * Gazebo
  * Rviz
  * URDF for robot model creation
  * Moveit!

---

_Project Description:_

   This project began by providing a thorough introduction to the Robot Operating System (ROS), specifically how to establish a Catkin workspace, import/build packages, as well as create and link nodes. These concepts were further illustrated via exploration of Turtlesim.  In order to use ROS, a Linux machine was desired. Therefore, the Workstation Player from VMWare was implemented, using a pre-constructed Ubuntu Linux environment, with ROS already installed. Linux command line manipulation was the primary method learned in the use of said Ubuntu Linux environment.
   
   Next, the project itself was discussed, the overall goal being to use a model of the KUKA KR-210 robotic arm in a virtual environment (Gazebo), programming it to locate and grab a cylindrical object, thereafter placing it in a designated bin. In order to accomplish this, lectures on forward and inverse kinematics were provided, focusing primarily on the motion of robotic arms. Afterwards, the basic workings of Gazebo was discussed, as well as the implementation of Rviz in visualizing incoming sensory data. Additionally, the URDF file format was discussed in the creation of robot models, specifically using XML format, applied in this project to the KR-210 robot arm. 
   
   With the robot arm modeled and inverse kinematics detailed, the Moveit! application was used to simplify the process of path and motion planning, allowing primary focus to be placed on the optimization of the inverse kinematics Python file.
     
   For a full report of how this was accomplished, see the included write-up: 
   
   [Robotic Arm Pick and Place Write-Up](https://github.com/akompaniyets/Robotic-Arm-Pick-and-Place/blob/master/Robotic%20Arm%20Pick%20and%20Place%20Write-Up.pdf)
