#ifndef PROJECTIVE_UTIL_H
#define PROJECTIVE_UTIL_H

//#include "SubscribeToKinect.h"

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

/**/
extern Eigen::MatrixXf getPIdent();
/**/
extern Eigen::Matrix3f getCameraIntrinsicMatrix(float fx, float fy, float cx, float cy);
/* Takes a depth value and converts it to a 3D x, y, and z. */
extern void depth_to_3D(Eigen::Matrix3f cam_cal, int index, float &x, float &y, float &z, cv::Mat depth_image);
/* perform calculations to transform 3D coordinates to 2D using camera calibration*/
extern Eigen::MatrixXf get_2d_points(Eigen::Matrix3f cam_cal, Eigen::MatrixXf p_ident, Eigen::MatrixXf rotation_trans_mat, Eigen::MatrixXf raw_3d_coords);
/* Converts all of the depth values in depth_image and puts the x, y, and z into raw_3D_coords. */
extern void calculate_3D_coords();
/* Create the grid that holds the mapping.*/
extern int ** initialize_point_grid();
/* Given a pixel in the form x, y, return the 3D value corresponding to that location. */
extern geometry_msgs::Point get_3d_point(int x, int y);

/* Take the values of raw_2d_points and put them into a grid.
 * To retrieve a point of a pixel <x, y>, grab the index at <x, y> and 
 * index into raw_3d_points. 
 */
extern void build2Dto3DMap(Eigen::MatrixXf raw_2d_points, int **map2to3);

/* Adjust a x, y, depth point to reflect the camera calibration. */
extern geometry_msgs::Point transform_point(float x, float y, float depth_val);

// Find what's being pointed at out in space
extern std::vector<geometry_msgs::Point> extend_point (std::vector<geometry_msgs::Point> origin, int scalar); 
    

#endif