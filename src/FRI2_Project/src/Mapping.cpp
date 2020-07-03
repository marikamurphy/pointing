#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <Eigen/Dense>
/*

int main(int argc, char** argv){
  //Eigen::MatrixXf pointMatrix(10,3);
  Eigen::MatrixXd pointMatrix;
  int rowPointer = 0;
	// Find the transform between the depth frame and the rgb frame
  ros::init(argc, argv, "tf_stuff");
  ros::NodeHandle node;

  tf::TransformListener listener;
  
  while (node.ok()) {
    tf::StampedTransform transform;
    try{
      listener.waitForTransform("/camera_rgb_optical_frame", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(10.0));
                               
      listener.lookupTransform("/camera_rgb_optical_frame", "/camera_depth_optical_frame",  
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    // Grab rotation and translation and print
    tf::Quaternion rotation;
    rotation = transform.getRotation();
    tf::Vector3 translation;
    translation = transform.getOrigin();
    tf::Matrix3x3 m(rotation);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw<< "\n";
    double x, y, z, w;
    x = translation.x();
    y = translation.y();
    z = translation.z();
    w = translation.w();
 //   pointMatrix(rowPointer, 1) = x;
   // pointMatrix(rowPointer, 2) = y;
   // pointMatrix(rowPointer, 3) = z;
   // rowPointer++;
    std::cout << "x: " << x << " y: " << y << " z: " << z << " w: " << w << "\n";
  }


  //MatrixXd rigidTransform(4, 4);
  //rigidTransform << 1, 0, 0, -0.025 << 
	return 0;
	// camera_depth_optical_frame matrix
	// [575.8157348632812, 0.0, 314.5 ]
	// [0.0, [575.8157348632812,  235.5]
	// [0.0, 0.0, 1.0]
	// camer_rgb_optical_frame
	// [525.0, 0.0, 319.5]
	// [0.0, 525.0, 239.0]
	// [0.0, 0.0, 1.0]
	
	// [1, 0, 0 -.025]
	// [0, 1, 0 0]
	// [0, 0, 1, 0]
	// [0, 0, 0, 1]
};

*/
