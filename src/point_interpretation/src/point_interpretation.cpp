#include <Client.h>
#include <SubscribeToKinect.h>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <unistd.h>
 
#define MAXNUM 999999 
using namespace cv;
using namespace Eigen;
using namespace std;

/* Creates the camera intrinsic matrix in an eigen matrix given parameters.
 * Alpha and beta are the focal length.  u0 and v0 are the principle point. */
MatrixXd computeCameraIntrinsicMatrix(double alpha, double beta, double u0, double v0) {
    MatrixXd camera_intrinsic = MatrixXd::Zero(3,3);
    camera_intrinsic(0,0) = alpha;
    camera_intrinsic(1,1) = beta;
    camera_intrinsic(0,2) = u0;
    camera_intrinsic(1,2) = v0;
    camera_intrinsic(2,2) = 1;
    return camera_intrinsic;
}

/* Creates the matrix used to project from 3d to 2d.
 * It's similar to the identity matrix, but it has an extra column of 0s. */
MatrixXd computeIdentityIdealProjection() {
    MatrixXd ideal_proj = MatrixXd::Zero(3,4);
    for(int i = 0; i < 3; i++)
        ideal_proj(i,i) = 1;
    return ideal_proj;
}

/* Takes a translation into account for an ideal_projection. This is how the camera has
 * moved between the two views, 2d and 3d.*/
MatrixXd computeIdealProjection(MatrixXd translation) {
    MatrixXd ideal_proj = MatrixXd::Zero(3,4);
    // Set rotation portion.
    for(int i = 0; i < 3; i++)
        ideal_proj(i,i) = 1;
    // Set translation portion.
    for(int i = 0; i < 3; i++)
        ideal_proj(i,3) = (translation(i, 0) / translation(3, 0));

    return ideal_proj;
}

/* Transforms in_mat from homogenous coordinates to cartesian */
MatrixXd computeCartesianFromHomogeneous(MatrixXd in_mat) {
    MatrixXd cart_point(in_mat.rows() - 1, in_mat.cols());
    for(int row = 0; row < in_mat.rows() - 1; row++) {
        for(int col = 0; col < in_mat.cols(); col++) {
            cart_point(row, col) = in_mat(row, col) / in_mat(in_mat.rows() - 1, col);
        }
    }
    return cart_point;
}

/* draws cross at x,y on given cv image mat */
void renderCrosshair(int x, int y, cv::Mat &in_mat, int rgb[]) {
    line(in_mat, Point(x - 5,y), Point(x+5,y), Scalar(rgb[0],rgb[1],rgb[2]));
    line(in_mat, Point(x,y-5), Point(x,y+5), Scalar(rgb[0],rgb[1],rgb[2]));
}

void renderCrosshair(MatrixXd in_pts, cv::Mat &in_mat, int rgb[]) {
    for(int i = 0; i < in_pts.cols(); i++)
        renderCrosshair(in_pts(0,i), in_pts(1,i), in_mat, rgb);
}

void renderVector(vector<Point> vec, Mat &in_mat, int rgb[]){
    for(int i = 0; i < vec.size(); i++){
        circle(in_mat, vec.at(i), 1, Scalar(rgb[0],rgb[1],rgb[2]), -1);
    }
}

/* creates arbitrary arm coordinate, will be replaced by openpose eventually */
MatrixXd makeArm() {
    MatrixXd arm(4, 2);
    // Elbow
    arm(0, 0) = 2;
    arm(1, 0) = 2;
    arm(2, 0) = 7;
    arm(3, 0) = 1;
    // Wrist
    arm(0, 1) = 1;
    arm(1, 1) = 1;
    arm(2, 1) = 5;
    arm(3, 1) = 1;
    return arm;
}

/* */
MatrixXd extendArm(MatrixXd arm, int scale){
    MatrixXd extendedArm(4, 2);
    extendedArm.col(0) = arm.col(0);
    extendedArm(0, 1) = arm(0,1) + scale * (arm(0,1) - arm(0,0)); // scale each point
    extendedArm(1, 1) = arm(1,1) + scale * (arm(1,1) - arm(1,0));
    extendedArm(2, 1) = arm(2,1) + scale * (arm(2,1) - arm(2,0));
    extendedArm(3, 1) = 1;
    return extendedArm;
}

/* Find the two intercepts of the line that are on screen. */
MatrixXd findIntercepts(MatrixXd arm_in_2d, int height, int width) {
    MatrixXd intercepts(2, 2);
    double m = (arm_in_2d(0, 0) - arm_in_2d(0, 1)) / (arm_in_2d(1, 0) - arm_in_2d(1, 1));
    int left = -m * arm_in_2d(0, 1) + arm_in_2d(1, 1); // 0, left
    int top = -arm_in_2d(1, 1) / m + arm_in_2d(0, 1); // top, 0
    int right = m * (width - arm_in_2d(0, 1)) + arm_in_2d(1, 1); // width, right
    int bottom = (height - arm_in_2d(1, 1)) / m + arm_in_2d(0, 1); // bottom, height
    //intersects the right/left side of image
    if (left >= 0 && left <= height && right >= 0 && right <= height) {
        // left
        intercepts(0, 0) = 0;
        intercepts(1, 0) = left;
        // right
        intercepts(0, 1) = width;
        intercepts(1, 1) = right;
    } else { //interscts the top/bottom of image
        // top
        intercepts(0, 0) = top;
        intercepts(1, 0) = 0;
        // bottom
        intercepts(0, 1) = bottom;
        intercepts(1, 1) = height;
    }
    return intercepts;
}

/* Show the boxes on the cv image. */
void printBoxes(Mat out_image, vector< Point> boxes, vector<string> labels) {
    cv::Scalar colorScalar = cv::Scalar(94, 206, 165) ;
    for(int i = 0; i < boxes.size(); i+=2){
        rectangle(out_image, boxes.at(i), boxes.at(i+1), colorScalar);
        putText(out_image, labels.at(i/2), boxes.at(i), FONT_HERSHEY_SIMPLEX, 1, colorScalar);
    }
    
}

/* Check if a point is inside a box and which boxes. */
vector<int> insideBox (vector<Point> boxes, Point cor){
    vector<int> ret;
    for(int i = 0; i < boxes.size(); i +=2) {
        Point tLeft = boxes.at(i);
        Point bRight = boxes.at(i+1);
        /* Point must be between the two x values and the two y values. */
        if (cor.x >= tLeft.x && cor.x <= bRight.x 
        && cor.y >= tLeft.y && cor.y <= bRight.y) {
            cout << "inside box: " << cor.x << " " << cor.y << endl;
            ret.push_back(i);
        }
    }
    return ret;
}

/* Get the pixels of a line going from first to second. */
vector<Point> bresenham(Point first, Point sec) 
{ 
    int dx = 1;
    int dy = 1;
    // Code to make it go either way
    // Are we going left or right?
    if (first.x > sec.x) {
        // We're going 
        dx = -1; 
    }
    if(first.y > sec.y){
        dy = -1;
    }
    vector<Point> ret;
    int m_new = 2 * (sec.y - first.y); 
    int slope_error_new = m_new - (sec.x - first.x); 
    int x = first.x;
    int y = first.y;
    while (x != sec.x)
    {
        Point pixel;
        pixel.x = x;
        pixel.y = y;
        ret.push_back(pixel);
        cout << "(" << x << "," << y << ")\n"; 
        // Add slope to increment angle formed 
        slope_error_new += m_new; 
        // Slope error reached limit, time to 
        // increment y and update slope error. 
        if (slope_error_new * dy >= 0) 
        { 
            y+=dy; 
            slope_error_new  -= 2 * (sec.x - first.x); 
        }
        x+=dx; 
    }
    return ret; 
} 

/* Decide which intercept is the endpoint based on the direction of the arm. */
Point findEndpoint (MatrixXd arm_in_2d) {
    // Create wrist and elbow as cv points so they can be used in bresenham
    Point wrist(arm_in_2d(0, 1), arm_in_2d(1, 1));
    Point elbow(arm_in_2d(0, 0), arm_in_2d(1, 0));
    Point myIntercept;
    MatrixXd intercepts = findIntercepts(arm_in_2d, 480, 640);
    /* If the X coordinate of the first point is 0, then it's at the top.
     * That means we are looking at the top and bottom pair.
     */
    if (intercepts(0, 0) == 0) {
        // Intercepts on the left and right
        if (wrist.x < elbow.x) {
            // Left
            myIntercept.x = intercepts(0, 0);
            myIntercept.y = intercepts(1, 0);
        } else {
            // Right
            myIntercept.x = intercepts(0, 1);
            myIntercept.y = intercepts(1, 1);
        }
    } else {
        // Intercepts on the top and bottom
        if (wrist.y < elbow.y) {
            // Top
            myIntercept.x = intercepts(0, 0);
            myIntercept.y = intercepts(1, 0);
        } else {
            // Bottom
            myIntercept.x = intercepts(0, 1);
            myIntercept.y = intercepts(1, 1);
        }
    }
    return myIntercept;
}

int main(int argc, char **argv) {
    Client *client = new Client();
    /* Send image will load the image and use the client server 
     * it to python for object recognition. */
    client->sendImage("./donut.png");
    vector<Point> boxes = client->getBoxes();
    vector<string> labels = client->getLabels();
    MatrixXd arm = makeArm();
    MatrixXd extendedArm = extendArm(arm, 5);
    // Create tranlation point
    MatrixXd point_trans(4,1);
    point_trans(0,0) = 0;
    point_trans(1,0) = 0;
    point_trans(2,0) = 0;
    point_trans(3,0) = 1;
    // Create matrices for projecting into 2d
    namedWindow("out_image");    
    MatrixXd camera_intrinsic = computeCameraIntrinsicMatrix(304, 305, 320, 240);
    MatrixXd camera_projective_matrix = camera_intrinsic * computeIdealProjection(point_trans);
    // Project into 2D
    MatrixXd new_arm = camera_projective_matrix * arm;
    MatrixXd new_extended_arm = camera_projective_matrix * extendedArm;
    MatrixXd arm_in_2d = computeCartesianFromHomogeneous(new_arm);
    Point endpoint = findEndpoint(arm_in_2d);
    Point wrist(arm_in_2d(0, 0), arm_in_2d(1, 0));
    vector<Point> bres = bresenham(wrist, endpoint);
    //Mat out_image q= Mat::zeros(480, 640, CV_8UC3);
    Mat out_image = imread("./donut.png");   // Read the file
    printBoxes(out_image, boxes, labels);
    int red_rgb[3] = {0, 0, 255};
    for (Point p : bres) {
        vector<int> matchingBoxes = insideBox(boxes, p);
        if (matchingBoxes.size() != 0) {
            renderCrosshair(p.x, p.y, out_image, red_rgb);
        }
    }
    renderCrosshair(endpoint.x, endpoint.y, out_image, red_rgb);
    int green_rgb[3] = {0, 255, 0};
    renderCrosshair(arm_in_2d, out_image, green_rgb);
    int blue_rgb[3] = {255, 0, 0};
    renderCrosshair(computeCartesianFromHomogeneous(new_extended_arm), out_image, blue_rgb);
    renderVector(bres, out_image, green_rgb);
    imshow("out_image", out_image);
    waitKey(0);
    return 0;
}