#ifndef _POINT_MAKELINE_H_
#define _POINT_MAKELINE_H_

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>

class MakeLine {
public:
	ros::NodeHandle *n;

	MakeLine(ros::NodeHandle *n) : n(n) {
	    marker_pub = n->advertise<visualization_msgs::Marker>("visualization_marker", 10);
	}
	void pubPoints(geometry_msgs::Point, geometry_msgs::Point);
	ros::Publisher marker_pub;
private:
};

#endif
