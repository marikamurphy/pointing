#include "point/TFBroadcastPR.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
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
