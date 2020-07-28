#include "point/TFBroadcastPR.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

TFBroadcastPR::TFBroadcastPR(std::string src, std::string dest) {
    _src = src;
    _dest = dest;
} 

void TFBroadcastPR::receivePose(geometry_msgs::Pose &pose) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = _src; //which is source and which is dest? TODO
    transformStamped.child_frame_id = _dest;
    transformStamped.transform.translation.x = pose.position.x;
    transformStamped.transform.translation.y = pose.position.y;
    transformStamped.transform.translation.z = pose.position.z;
    
    transformStamped.transform.rotation.x = pose.orientation.x;
    transformStamped.transform.rotation.y = pose.orientation.y;
    transformStamped.transform.rotation.z = pose.orientation.z;
    transformStamped.transform.rotation.w = pose.orientation.w;

    br.sendTransform(transformStamped);
}




/* #include "tf/transform_broadcaster.h"
#include "tf/tf.h"

TFBroadcastPR::TFBroadcastPR(std::string src, std::string dest) {
    _src = src;
    _dest = dest;
} 

void TFBroadcastPR::receivePose(geometry_msgs::Pose &pose) {
    static tf::TransformBroadcaster br;

    tf::Transform transform(
        tf::Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ),
        tf::Vector3(
            pose.position.x,
            pose.position.y,
            pose.position.z
        )
    );

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _src, _dest));
    
}
*/
