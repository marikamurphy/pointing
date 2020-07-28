#include "point/Client.h"
#include "point/TFBroadcastPR.h"
#include "tf2_ros/message_filter.h"//do we need?
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <openpose/headers.hpp> //Openpose dependencies
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#define ROOT_TRANSFORM "camera_rgb_optical_frame" //TODO: change for hsr
#define SCREEN_WIDTH    640
#define SCREEN_HEIGHT   480
