#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/OpenPoseHuman.h>
#include <openpose_ros_msgs/PointWithProb.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <cstring>
#include "fri2/ImageProcessor.h"
#include "fri2/Storage.h"
#include "fri2/MakeLine.h"
#include "fri2/point_math.h"
#include <set>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <openpose/header.hpp>

//TYPE DEFINITIONS
typedef struct {
	int x;
	int y;
} pixel2d;

//HACKY STUPID GLOBAL VARIABLES THAT SHOULDN"T EXIST
static darknet_ros_msgs::BoundingBox *g_boxes = nullptr;
static int g_numBoxes = -1;
static cv::Mat g_matrix;
static bool mat_read = false;

// forward declarations because we don't have a header
void mapDepthToMatrix(Eigen::MatrixXf &, Storage &);
void bressLine(int **, Eigen::MatrixXf &, pixel2d, pixel2d, float *, float *);
char * insideBoundingBox(pixel2d, int);
void build2Dto3DMap(int **, Eigen::MatrixXf &);

//methods that ryan didn't feel like moving
void store_boxes(const darknet_ros_msgs::BoundingBoxes &boxes) {
	if (g_numBoxes == -1) return; //Can't store data without this value
	if (g_boxes) delete[] g_boxes;
	g_boxes = new darknet_ros_msgs::BoundingBox[g_numBoxes];
	for (auto i = 0; i < g_numBoxes; i++)
		g_boxes[i] = boxes.bounding_boxes.at(i);
}

void store_box_count(const darknet_ros_msgs::ObjectCount &msg) {
	g_numBoxes = msg.count;
}

// Round to the nearest float
inline int nearest(float f) {
	if ((f - 0.5) > (int) f) return ((int) f + 1);
	return (int) f;
}

void depth_test_cb(const sensor_msgs::Image img) {
	g_matrix = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1)->image;
	mat_read = true; 
}

// actually relevant
int main(int argc, char **argv) { 
	ros::init(argc, argv, "hw5_node");
	ros::NodeHandle n;
	Storage s;

	//Human subscription
	ros::Subscriber openpose_sub = n.subscribe<openpose_ros_msgs::OpenPoseHumanList>(
																"/openpose_ros/human_list", 10, &Storage::add, &s); 
	//Calibrate subscription
	ros::Subscriber depth_camera_info = n.subscribe("/camera/depth/camera_info", 1, &Storage::calibrate, &s);
	
	//Image (points) subscription
	ros::Subscriber depth_image_test = n.subscribe("/camera/depth/image", 10, &depth_test_cb);
	ros::Subscriber boxes_counter = n.subscribe("/darknet_ros/found_object", 10, &store_box_count);
	ros::Subscriber boxes = n.subscribe("/darknet_ros/bounding_boxes", 10, &store_boxes);

	//Read in the data of humans and points
	openpose_ros_msgs::OpenPoseHuman h;
	while (!mat_read || (!g_boxes) || g_numBoxes == -1) {
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
	h = s.get();

	for (int i = 0; i < g_numBoxes; i++)
		ROS_INFO("Box %d: class %s, prob %f, x: [%ld,%ld] y: [%ld,%ld]", i,
				  g_boxes[i].Class.c_str(), g_boxes[i].probability,
		          g_boxes[i].xmin, g_boxes[i].xmax, g_boxes[i].ymin, g_boxes[i].ymax);

	// We want a 4 row, 480 * 640 column matrix that holds the 3D points from the depth camera
	// We multiply the 3 x 4 projective matrix by this matrix to get the camera points
	// Then, camera point (x, y) is the output matrix 640 * y + x / w
	// The resulting w represents the depth, and we always want the smaller w because it is
	// closer to the camera

	Eigen::MatrixXf pointMatrix(4, 307200);
	mapDepthToMatrix(pointMatrix, s); // Fill out point matrix
	// At this point, the pointMatrix has the following format:
	// [x0, x1, x2, ..., x307199]
	// [y0, y1, y2, ...]
	// [z0, z1, z2, ...]
	// [1,  1,  1,  ...]
	// Each column represents a homogenous spatial coordinate from the depth frame
	// The columns are ordered in the same order as the pixels in row-major order

	Eigen::MatrixXf rMat(3, 4); // Rigid Transformation Matrix (ignoring fourth row); 3 x 4
							// Notably, this is also the Pideal matrix
	rMat << 1, 0, 0, -0.025,
					0, 1, 0, 0.0,
					0, 0, 1, 0.0;

	Eigen::MatrixXf kMat(3, 3); // Constant matrix for adjusting values; 3 x 3
							// Should describe the color camera; TODO: verify
	kMat << 525.0, 0.0,   319.5,
					0.0,   525.0, 239.0,
					0.0,   0.0,   1.0;

	// Projective matrix: Formed from psuedoidentity rotation, plus translation, multiplied by constants
	// This is the real projective matrix, not the ideal projective matrix, and is 3 x 4
	// I *believe* that this matrix projects points from the RGB camera into the depth frame
	Eigen::MatrixXf pMat = kMat * rMat;

	Eigen::MatrixXf cameraCoords = pMat * pointMatrix; // 3 x 4 * 4 x 307200 yields 3 x 307200 points
	// The format of this matrix is this:
	// [x0, x1, x2, ..., x307199]
	// [y0, y1, y2, ...]
	// [w0, w1, w2, ...]
	// These are 2D camera coordinates (where each point falls into), and w represents the depth.
	// Any conflict where two 3D points fall into the same 2D point should hopefully resolve by
	// using the lower w. You can calculate the actual x and y by dividing by scaling factor w.

	// Build a mapping where every single camera pixel has an index into the point cloud for where
	// it is in space. For occluded or doubly-recognized points, we always register the one in front
	// (aka nearest z or smallest w)
	int **mappingData = (int **) malloc(sizeof(int *) * 480);
	for (int r = 0; r < 480; r++) {
		mappingData[r] = (int *) malloc(sizeof(int) * 640);
		for (int c = 0; c < 640; c++) {
			mappingData[r][c] = -1; // -1 initialized
		}
	}
	build2Dto3DMap(mappingData, cameraCoords);

	// All bresenham line calculations
	float slope = (h.body_key_points_with_prob[4].y - h.body_key_points_with_prob[3].y) /
	              (h.body_key_points_with_prob[4].x - h.body_key_points_with_prob[3].x);
	float posY = h.body_key_points_with_prob[4].y;
	int posX = (int) h.body_key_points_with_prob[4].x;
	int startX = posX;
	int startY = (int) posY;
	
	// If we are pointing left (from the camera's perspective)
	if (h.body_key_points_with_prob[4].x - h.body_key_points_with_prob[3].x < 0) {
		ROS_INFO("Pointing left, x: %d, y: %d", posX, (int)posY);
		//Pointing left, calculate endpoint of line (where it leaves the screen)
		for (; posX > 0; posX--) {
			posY -= slope;
			if (posY < 0 || posY > 480) break;
		}
		if (posY < 0) posY = 0;
		else if (posY > 480) posY = 480;
		else posX = 0;
		ROS_INFO("After adjustment: x: %d, y: %d", posX, (int) posY); //Found endpoint of bresenham

		ROS_INFO("Running Bresenham Algorithm");
		pixel2d sPixel { startX, startY };   // 2D color camera start pixel
		pixel2d ePixel { posX, (int) posY }; // 2D color camera end pixel (where the pixel goes out of bounds)

		// Build a vector that represents where the line starts (used in calculating distance to a line in 3D)
		int mIndex = -1;
		while (mIndex == -1) {
			mIndex = mappingData[sPixel.y][sPixel.x];
			if (mIndex == -1) sPixel.x++;
		}
		float vectorStart[3] { pointMatrix(0, mIndex), pointMatrix(1, mIndex), pointMatrix(2, mIndex) };

		// Build a vector that represents where the line ends by adding the vector from elbow to wrist 100 times
		// Basically just increases the length of a vector a bunch to make a line that is guaranteed to go out of frame
		mIndex = -1;
		int tempx = h.body_key_points_with_prob[3].x;
		while (mIndex == -1)
			mIndex = mappingData[(int) h.body_key_points_with_prob[3].y][tempx++];
		float vectorEnd[3];
		vectorEnd[0] = vectorStart[0] + (vectorStart[0] - pointMatrix(0, mIndex)) * 100;
		vectorEnd[1] = vectorStart[1] + (vectorStart[1] - pointMatrix(1, mIndex)) * 100;
		vectorEnd[2] = vectorStart[2] + (vectorStart[2] - pointMatrix(2, mIndex)) * 100;
		bressLine(mappingData, pointMatrix, sPixel, ePixel, vectorStart, vectorEnd); // Perform the algorithm and stop
	}

	for (int r = 0; r < 480; r++)
		free(mappingData[r]);
	free(mappingData);

	return 0;
}

/*
 * Create a maping that build a 2D array of indices
 * Each index in the output matrix represents the index into the depth matrix
 * where the point is closest to the camera for that pixel.
 * This is done by iterating all of the 2D coordinates of the camera and only
 * selecting the one that happens to be closest to the camera (aka min(w))
 */
void build2Dto3DMap(int **out, Eigen::MatrixXf &coords) {
	for (int point = 0; point < 480 * 640; point++) {
		float scale = coords(2, point);				// Find w
		if (std::isnan(scale)) continue;
		int px = (int) (coords(0, point) / scale);  // Calculate pixel coordinate
		int py = (int) (coords(1, point) / scale);
		if (out[py][px] == -1) out[py][px] = point; // If first visit, set the index mapping
		else if (coords(2, out[py][px]) > scale) // If the previous index was further from the camera than this one
			out[py][px] = point; // Store current index
	}
}

/* 
 * Calculate the x,y,z coordinates of a pixel from the depth camera perspective
 * Uses the g_matrix of depth coordinates and image registration math to determine
 * where the point is, stores result in float references
 */
void calculateCoords(Storage &s, int index, float &x, float &y, float &z) {
    int row = index / 640;
    int col = index % 640;
    z = g_matrix.at<float>(row, col);  // Find actual depth
    x = (col + 0.5 - s.cx) * s.fx * z; // Perform frustum calculations
    y = (row + 0.5 - s.cy) * s.fy * z;
}

/*
 * Store all of the depth points into a big Eigen matrix
 * Resulting matrix shall have 480 * 640 rows, each with 3 columns
 * The format of the columns is [x, y, z]
 * Requires a Storage object that has the camera constants
 */
void mapDepthToMatrix(Eigen::MatrixXf &pMatrix, Storage &s) {
	for (int index = 0; index < 480 * 640; index++) {
		float x, y, z;
		calculateCoords(s, index, x, y, z); // First, calculate the actual x, y, and z in question
		pMatrix(0, index) = x;   // Store these values
		pMatrix(1, index) = y;
		pMatrix(2, index) = z;
		pMatrix(3, index) = 1.0; // Store a 1 for w into the depth matrix (scales in the color 2D matrix)
	}
}

// Super dumb implementation of a bresenham line; checks along the line given
// Will look up and down a certain number of pixels away from the given points
// Note: This algorithm is similar, but not identical to the one on wikipedia
void bressLine(int **map, Eigen::MatrixXf &dpthMat, pixel2d start, pixel2d end, float *vStart, float *vEnd) {
	float dX = end.x - start.x;
	float dY = end.y - start.y;
	float m = dY/dX; // Slope of the bresenham line
	char *name = nullptr;  // Temporary pointer for the name of an object
	char *guess = nullptr; // Temporary pointer for what the program thinks you're pointing at
	float minDistToLine = 1000.0; // Best fit to the line found so far
	int const verticalRange = 2; //Check 2 up, 2 down from calculated pixel

	for (int tx = 0; tx < abs(end.x - start.x); tx++) {
		// Iterating x from the start x to the end x
		int sx = (end.x < start.x) ? -tx : tx; // Calculate the signed x (moving right or left)
		pixel2d p { sx + start.x, nearest(m * sx + start.y)}; // Calculate which pixel you're at
		if (p.x < 0 || p.x > 640 || p.y < 0 || p.y > 480) continue; // Break if you're out of bounds
		if (map[p.y][p.x] != -1 && (name = insideBoundingBox(p, verticalRange))) {
			// If your pixel has a depth and is inside a bounding box:
			int index = map[p.y][p.x];
			float pt3d[3] { dpthMat(0, index), dpthMat(1, index), dpthMat(2, index) }; // Find its 3D coordinates
			float distToLine = pointLineDistance3d(vStart, vEnd, pt3d); // Calculate its distance to the vector
			if (distToLine < minDistToLine) { // Store the best fit
				minDistToLine = distToLine;
				guess = name;
			}
		}
	}
	if (guess) // Print what the progrma found
		ROS_INFO("Guess: object %s, %0.2f from line.", guess, minDistToLine);
	else
		ROS_INFO("No object found");
}

// Check if the pixel is inside bounding box
char * insideBoundingBox(pixel2d pixel, int delta) {
	std::set<int> yPts;
	// Gather all x and y coordinates around the point
	yPts.insert(pixel.y);
	for (int i=0; i<delta; i++) {
		yPts.insert(pixel.y + i);
		yPts.insert(pixel.y - i);
	}
	// Iterate through the bounding boxes
	for (int i = 0; i < g_numBoxes; i++) {
	    // Check if there are any valid y values
	    auto ptr = yPts.begin();
			while(ptr != yPts.end()) { //Could substitute for loop here, but no real point
					if (*ptr >= g_boxes[i].ymin &&     // Four checks for inside of a bounding box
							*ptr <= g_boxes[i].ymax &&
							pixel.x >= g_boxes[i].xmin &&
							pixel.x <= g_boxes[i].xmax) {
							if (strcmp(g_boxes[i].Class.c_str(), "person") == 0) {
								// Skip any boxes that register as "person" becuase yolo will register you pointing at yourself
								ptr++;
								continue;
							}
							return (char *) g_boxes[i].Class.c_str();
					}
				ptr++;
			}
    }
	
    return nullptr;
}
