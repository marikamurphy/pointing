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

* For the simulation in gazebo/rviz: 
* roslaunch hsr_gazebo hsr_apt.launch
* sim_mode then run python move.py from hsr_gazebo directory (TODO: use CallPython)

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


Camera calibration info

frame_id: "head_rgbd_sensor_rgb_frame"
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [554.3827128226441, 0.0, 320.5, -0.0, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False

