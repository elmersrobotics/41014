# Sensors and Control - Project 7:  Turtlebot robot following each other
By *Elmer Junior Caballero, Jithin Abraham, Shayer Shah*

This project uses a MATLAB application and simulated ROS Gazebo environment to demonstrate a 
turtlebot3 following another turtlebot3 using different sensing and control techniques.

__*Software Used:*__
- Ubuntu 18.04 (Virtual Machine)
- VMWare Workstation 16 Player
- MATLAB
- ROS melodic

__*MATLAB Toolboxes Used:*__
- ROS Toolbox
- Image Processing Toolbox

__*Packages Used:*__
- turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3.git
- turtlebot3_simulation - https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
- turtlebot3_msgs - https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

## Project Overview
The assigned project ‘Turtlebot robot following each other’ requires the group to use learnt and state-of-the-art control algorithms to manipulate a 
Turtlebot to follow the path of a guider Turtlebot. Both RGD and depth images can be used for the control and specially designed artificial markers, 
or patterns can be used on the back of the guider Turtlebot. The proposed approach to delivering the assigned project is to demonstrate the control of a 
simulated Turtlebot that will follow a simulated guider Turtlebot within a Gazebo simulator environment (running on a Linux virtual machine).

The guider Turtlebot will have a specially designed marker behind it which can be observed and recognized by the follower Turtlebot. Basic operattion mode uses a 
simple RGB colour marker, while advanced operation mode uses a 'King of Hearts' card as a marker.
A MATLAB program that is running on the host PC connects to the ROS master on the virtual machine and subscribes to the required topics to receive
onboard camera and sensor data from the simulated follower Turtlebot. When running, the MATLAB program analyses the data from the camera feed, determinse the required 
movements to ‘follow’ the artificial marker, and subsequently publishes a message to the follower Turtlebot to request movement. The program also analyses laser scan data
from the follower Turtlebot to perform collision avoidance during operation. 

## MATLAB Program Structure
__*MATLAB GUI*__
- A GUI has been developed for the program to allow the user to select the target to be followed (Basic: RGB, Advanced: King of Hearts) and provides convenient start/stop controls.

__*MATLAB CLASS*__
- TurtleBotTask function - This constructor is the program's main function that will connect to the virtual machine's ROS master and subscribe to the required image
and laser scan data. This function initialises the project parameters and runs a loop that will continuously call the image processing functions and movement calculation
functions to prepare and publish control messages to the follower Turtlebot. The function will also display an onboard camera view of the follower Turtlebot as it
operates.
- processImageBasic function - [BASIC mode] This function analyses the given image data and determines if the desired colour can be detected in the current camera view.
If the selected colour is found, it will calculate and return the pixel size of the matched object and return its centre coordinates (x,y) in the frame.
- processImageAdvanced function - [ADVANCED mode] This function analyses the given image data and determines if the 'King of Hearts' pattern is detected in the current
camera view. If the card object is found, it will calculate the average x,y position of the generated point cloud to estimate the location of the object in the frame.
- calculateMovement function - This function will determine what linear and angular velocity messages to send to the follower turtlebot depending on the values are returned
by the image processing functions. A proportional control method is used in the function to decrease the measured error and will attempt to keep the detected target
in the centre of the camera view. The function also uses the gathered laser scan data to determine if an object is in front of the follower turtlebot and will stop
the Turtlebot's forward movement if a potential collision is found.

## Contribution:
- Elmer Junior Caballero: *50%*
- Jithin Abraham: *25%*
- Shayer Shah: *25%*


