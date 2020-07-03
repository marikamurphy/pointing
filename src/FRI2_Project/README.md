# FRI2_Project

## About
This project is a research project for the Freshman Research Initiative at the University of Texas at Austin. The goal of this project is ultimately to create a software program that will enable a robot to follow the direction a person is pointing and determine the object that person is pointing at. [Here](https://www.overleaf.com/8178441511hhgghwzctwtj) is a link to edit the Overleaf document for this project's proposal for more information, and [here](https://www.overleaf.com/read/prmwvtgfdbkz) is a link to view it.

## Approach
We hope to achieve this through a simple process:
1. Determine the length of a person's arm using [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)
2. Take a picture of a person pointing using a Kinect
3. Detect where their arm is in space using a combination of technologies: OpenPose is used on the HD RGB image to find the u,v coordinates in the plane of the image, the Kinect provides a depth image to find the rough depth at those points, and a registered image mapping function can translate those points into x, y, and z
4. Create a vector in space which begins at the person's elbow and extends towards their fingers, and scale it into space infinitely
5. Find where that vector first intersects an object in space
6. Run object detection (i.e. [YOLO](https://pjreddie.com/darknet/yolo/)) to determine what the user is pointing at

In order to detect various joints on a person's body, we're using OpenPose (as seen above). We're integrating OpenPose into ROS using [this repository](https://github.com/firephinx/openpose_ros). You can attempt to set it up by following the instructions on that repo, but we've found setting up OpenPose to be exceedingly difficult and only have it running on the FRI CSRES Kane computer.

We would like for the robot to have fairly high prediction accuracy, and in order to compare its performance we want to set up an experiment where the robot and a human participant both try to deduce what a second human participant is pointing at. The accuracy of the two predictions can then be compared.

## Setup
No matter what you do, this needs to be run on Kane. This is because OpenPose is only installed correctly on Kane, and the repository we're using is also only set up on Kane.

To initialize this repository, you should run the following process:
* `source /opt/ros/kinetic/setup.bash`
* `mkdir my_catkin_ws`
* `cd my_catkin_ws`
* `catkin_init_workspace`
* `mkdir src`
* `catkin build`
* `cd src`
* `git clone https://github.com/RG8452/FRI2_Project.git`
* `cd ..`
* `catkin build`

This should create a workspace and clone the existing repository into it. To run correctly, you should also set up [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) and download the [ROS Wrapper](https://github.com/firephinx/openpose_ros). The ROS Wrapper should be cloned into the source directory of the catkin workspace.

## Initialization
Before doing this, ensure that you've either set up OpenPose and the ROS Wrapper or are on Kane. There is a working directory of this repo located in /home/fri/Documents/pointing_ws and you can get to the machine remotely by running `ssh fri@kane.csres.utexas.edu`.

In order to productively run code, you'll need to have several processes running through terminal. If you open multiple terminals for these processes, each terminal **must** have sourced devel/setup.bash (from this directory). Here is a list of the commands that should be running, in the order that they need to be run:

* `source devel/setup.bash` (Use from the catkin workspace)
* `roslaunch freenect_launch freenect.launch` (This also runs roscore, provided it isn't running already)
  * `roslaunch bwi_launch v2.launch` (I'm told something like this exists for the robot but idk)
* `roslaunch openpose_ros openpose_ros.launch` This launches the OpenPose wrapper.
* `rosrun rviz rviz`
* `rosrun fri2_pkg <file name>`

All source code implementations should be in the src directory, and all source code interfaces (the .h files) should be put in the include/fri2 directory. The OpenPose ROS Wrapper can be configured by editing files within that openpose_ros directory; specifically, the launch file (openpose_ros/openpose_ros/launch/openpose_ros.launch) or the flags file (openpose_ros/openpose_ros/src/gflags_options.cpp).

### Adding files
All of the logic in this package should be abstracted into classes or other files as much as possible.If you want to add a file to handle some piece of logic, you should follow these steps:
1. Create a header file that specifies an interface under the folder `include/fri2/<file>.h`
2. Create a source file that provides an implementation under the folder `src/<file>.h`
3. Put the source file into the CMakeList.txt under the executable that needs it

There's currently a test file that does nothing but will provide a reference for how to structure additions.

## YOLO
The YOLO package that we intend to use alongside our project is a custom ROS package located in the GitHub repo [leggedrobotics/darknet_ros](https://github.com/leggedrobotics/darknet_ros). For reference in using/subscribing to the topic that this repo publishes, you can reference Parth's repo [here](https://github.com/ParthChonkar/FRI_FinalProject). Specifically, the file [subscriber.py](https://github.com/ParthChonkar/FRI_FinalProject/blob/master/identification_protocol/src/subscriber.py) might serve as a useful reference.

The LeggedRobotics darknet_ros package must be in the **same workspace src folder** as fri2_pkg in order to run. This must be configured in the CMakeLists file in order to function properly. Furthermore, the path variable in the ImageHandler class (called imgFolderPath) so that its absolute path points to (and includes) the rospackage on your local machine.

roslaunch darknet_ros yolo_v3.launch

For more information regarding distance from a point to a line in 3D:
http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
