#include "ros/ros.h"
#include "fri2/MakeLine.h"
#include <visualization_msgs/Marker.h>

void MakeLine::pubPoints(geometry_msgs::Point p, geometry_msgs::Point q) {

	//ros::Subscriber sub = n.subscribe("", 1000, pubPoints);
	// Can I do ros::init twice? I want to make a publisher in another method.
	//ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        //ros::Rate r(30);

	visualization_msgs::Marker line_strip;
	visualization_msgs::Marker points;
	// Setup information about line_strip
	line_strip.header.frame_id = "/camera_depth_optical_frame"; // Figure out the frame id!
	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "points_and_lines";
	line_strip.action = visualization_msgs::Marker::ADD;
	line_strip.pose.orientation.w = 1.0;
	line_strip.id = 1;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.scale.x = 0.05;
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;
	points.header.frame_id = "/camera_depth_optical_frame";
    	points.header.stamp =  ros::Time::now();
    	points.ns  = "points_and_lines";
    	points.action = visualization_msgs::Marker::ADD;
    	points.pose.orientation.w = 1.0;

 
 	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.05;
    	points.scale.y = 0.05;
    	points.id = 0;
	points.color.r = 1.0f;
	points.color.a = 1.0;
	points.points.push_back(p);
	points.points.push_back(q);
	// Put points into line strip
	line_strip.points.push_back(p);
	line_strip.points.push_back(q);
	while(ros::ok()) {
	marker_pub.publish(line_strip);
	//marker_pub.publish(points);
	}
}
