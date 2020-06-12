This is the pointing project README
Necessary tools:
openpose
ros

**Helpful commands**
*Do catkin build in the pointing workspace, not in point.  OpenPose will break*

* source devel/setup.bash
* roscore
* rosrun point point_node

* source devel/setup.bash
* roslaunch darknet_ros yolo_v3.launch
* roslaunch freenect_launch freenect.launch
* rosrun rviz rviz
* rosrun point_node <file name>
* rosbag play -l pointing.bag

* for python/pytorch: source pointing-env/bin/activate
* deactivate (leave virtual env)
* to run server for pytorch type python server.py

* For the simulation in gazebo: 
* roslaunch gazebo_ros empty_world.launch
* rosrun gazebo_ros spawn_model -file `rospack find hsr_description`/robots/hsrb4s.urdf -urdf -model MYROBOT
* todo: make launch file for hsr in gazebo
* For the simulation in rviz: roslaunch hsr_description display.launch

* https://github.com/stevenjj/openpose_ros
* 

// Result for BODY_25 (25 body parts consisting of COCO + foot)
// const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
//     {0,  "Nose"},
//     {1,  "Neck"},
//     {2,  "RShoulder"},
//     {3,  "RElbow"},
//     {4,  "RWrist"},
//     {5,  "LShoulder"},
//     {6,  "LElbow"},
//     {7,  "LWrist"},
//     {8,  "MidHip"},
//     {9,  "RHip"},
//     {10, "RKnee"},
//     {11, "RAnkle"},
//     {12, "LHip"},
//     {13, "LKnee"},
//     {14, "LAnkle"},
//     {15, "REye"},
//     {16, "LEye"},
//     {17, "REar"},
//     {18, "LEar"},
//     {19, "LBigToe"},
//     {20, "LSmallToe"},
//     {21, "LHeel"},
//     {22, "RBigToe"},
//     {23, "RSmallToe"},
//     {24, "RHeel"},
//     {25, "Background"}
// };
