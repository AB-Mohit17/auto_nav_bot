### Hi there 👋, My name is Mohit
#### Programming and Robotics
![Programming and Robotics](https://anivizindia.in/wp-content/uploads/2022/03/3d-animation.jpg)

My self Mohit from India. Currenlty an undergraduate student in IIT Roorkee and a Tech ensthusiast with a great passion for programming and Robotics

Skills: Java / C++ / Fortran / C / Python / Mysql / ROS



# Project ::




# Autonomous Navigation of Ground Robot
![Lines of code](https://img.shields.io/tokei/lines/github/AB-Mohit17/auto_nav_bot)

![GitHub repo size](https://img.shields.io/github/repo-size/AB-Mohit17/auto_nav_bot)

Making use of ROS (Robotics Operating System) to run Simulation of  ROBOT Navigating Automatically in a map of some environment from one point to another by choosing the shortest path , dodging the obstacles using a path planning with some predefined packages in ROS.
## Demo

https://drive.google.com/file/d/12U-ZFfdfukMCP64Ul8YYy5Yr2LTFvG-g/view?usp=sharing
## Features

- Uses AMCL for Path Planning
- Not Much Need of Python
- Provides pre-build packages for variouos tasks like mappping,localization,navigation
- Uses Great Simulation and Visualization Tools Like Gazebo and RVIZ 
- Not Much Need of Progamming is Required


## Tech Requirements :

- Ubuntu 18.04 LTS 
- ROS-melodic-desktop-full Version
- Git

## Installation : 

- Install ROS from the official website  
  https://wiki.ros.org/melodic/Installation/Ubuntu


## Packages required to be cloned : 

- turtlebot3               
    https://github.com/ROBOTIS-GIT/turtlebot3

- turtlebot3_simulations     
    https://github.com/ROBOTIS-GIT/turtlebot3_simulations

- turtlebot3_msgs   
    https://github.com/ROBOTIS-GIT/turtlebot3_msgs    

- Navigation containing AMCL,move_base        
    https://github.com/ROBOTIS-GIT/turtlebot3_simulations

## Approach to the Project : 

- Running Simulation in gazebo using teleop 
- Calculating Odometry of the bot using a python script
- Mapping of the environment of the bot slam_gmapping 
- Localization of bot in the Map using amcl 
- Navigation of bot using move_base 

## Steps for Building the Project : 

- Create a workspace for ur project    
    http://wiki.ros.org/catkin/Tutorials/create_a_workspace
      
- Follow the turotial for the Trtlebot3 Simulation 
    https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/ 

