#ifndef OPENPOSE_H
#define OPENPOSE_H
// #include <openpose/headers.hpp> //Openpose dependencies

op::Wrapper& opWrapper;

// Adapted from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/01_body_from_image_default.cpp
/* Get the Datum with the body key points. */
extern std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> get_key_points();

/* Print all of the body keypoints for the first person in the list. */
extern void print_keypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumPtr);

// Publish the relevant keypoints for wrists and elbows.
extern void show_body_keypoints(const op::Array<float>& keyPoints);

// Locate limb positions 
extern std::vector<geometry_msgs::Point> find_end_points(const op::Array<float>& keyPoints);


#endif

