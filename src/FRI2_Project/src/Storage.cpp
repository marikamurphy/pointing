#include <ros/ros.h>
#include <fri2/Storage.h>

void Storage::add(openpose_ros_msgs::OpenPoseHumanList list) {
    if(list.num_humans != 0) {
	__index = (__index + 1) % 10; __data[__index] = list.human_list[0];
    }
}

void Storage::calibrate(const sensor_msgs::CameraInfo& msg) {
	fx = 1 / msg.K[0];	
	fy = 1 / msg.K[4];
	cx = msg.K[2];
	cy = msg.K[5];
}

openpose_ros_msgs::OpenPoseHuman Storage::get() {
    return __data[__index];
}
