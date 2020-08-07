#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <unistd.h>
 
using namespace cv;
using namespace Eigen;
using namespace std;

 MatrixXd computeCameraIntrinsicMatrix(double alpha, double beta, double u0, double v0) {
    MatrixXd camera_intrinsic = MatrixXd::Zero(3,3);
    camera_intrinsic(0,0) = alpha;
    camera_intrinsic(1,1) = beta;
    camera_intrinsic(0,2) = u0;
    camera_intrinsic(1,2) = v0;
    camera_intrinsic(2,2) = 1;
    return camera_intrinsic;
 }

 MatrixXd computeIdentityIdealProjection() {
    MatrixXd ideal_proj = MatrixXd::Zero(3,4);
    for(int i = 0; i < 3; i++)
        ideal_proj(i,i) = 1;
    return ideal_proj;
 }

 MatrixXd computeIdealProjection(MatrixXd translation) {
    MatrixXd ideal_proj = MatrixXd::Zero(3,4);
    for(int i = 0; i < 3; i++)
        ideal_proj(i,i) = 1;

    for(int i = 0; i < 3; i++)
        ideal_proj(i,3) = (translation(i, 0) / translation(3, 0));

    return ideal_proj;
 }

MatrixXd computeCartesianFromHomogeneous(MatrixXd in_mat) {
    MatrixXd cart_point(in_mat.rows() - 1, in_mat.cols());
    for(int row = 0; row < in_mat.rows() - 1; row++) {
        for(int col = 0; col < in_mat.cols(); col++) {
            cart_point(row, col) = in_mat(row, col) / in_mat(in_mat.rows() - 1, col);
        }
    }
    return cart_point;
}

void renderCrosshair(int x, int y, cv::Mat &in_mat, int rgb[]) {
    line(in_mat, Point(x - 5,y), Point(x+5,y), Scalar(rgb[0],rgb[1],rgb[2]));
    line(in_mat, Point(x,y-5), Point(x,y+5), Scalar(rgb[0],rgb[1],rgb[2]));
}

void renderCrosshair(MatrixXd in_pts, cv::Mat &in_mat, int rgb[]) {
    for(int i = 0; i < in_pts.cols(); i++)
        renderCrosshair(in_pts(0,i), in_pts(1,i), in_mat, rgb);
}

MatrixXd simpleCube(double half_width) {
    int ctr = 0;
    MatrixXd point_hom(4,8);

    point_hom(0,ctr) = -half_width;
    point_hom(1,ctr) = -half_width;
    point_hom(2,ctr) = -half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    point_hom(0,ctr) = -half_width;
    point_hom(1,ctr) = half_width;
    point_hom(2,ctr) = -half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    point_hom(0,ctr) = half_width;
    point_hom(1,ctr) = -half_width;
    point_hom(2,ctr) = -half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    point_hom(0,ctr) = half_width;
    point_hom(1,ctr) = half_width;
    point_hom(2,ctr) = -half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    point_hom(0,ctr) = -half_width;
    point_hom(1,ctr) = -half_width;
    point_hom(2,ctr) = half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    point_hom(0,ctr) = -half_width;
    point_hom(1,ctr) = half_width;
    point_hom(2,ctr) = half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    point_hom(0,ctr) = half_width;
    point_hom(1,ctr) = -half_width;
    point_hom(2,ctr) = half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    point_hom(0,ctr) = half_width;
    point_hom(1,ctr) = half_width;
    point_hom(2,ctr) = half_width;
    point_hom(3,ctr) = 1;
    ctr++;

    return point_hom;
}

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

MatrixXd extendArm(MatrixXd arm, int scale){
    MatrixXd extendedArm(4, 2);
    extendedArm.col(0) = arm.col(0);
    extendedArm(0, 1) = arm(0,1) + scale * (arm(0,1) - arm(0,0)); // scale each point
    extendedArm(1, 1) = arm(1,1) + scale * (arm(1,1) - arm(1,0));
    extendedArm(2, 1) = arm(2,1) + scale * (arm(2,1) - arm(2,0));
    extendedArm(3, 1) = 1;
    return extendedArm;
}

Vector2d intersectionOfTwoLines(Vector2d A, Vector2d B, Vector2d C, Vector2d D) { 
    Vector2d intersection();
    // Line AB represented as a1x + b1y = c1 
    double a1 = B(1) - A(1); 
    double b1 = A(0) - B(0); 
    double c1 = a1*(A(0)) + b1*(A(1)); 
    
    // Line CD represented as a2x + b2y = c2 
    double a2 = D(1) - C(1); 
    double b2 = C(0) - D(0); 
    double c2 = a2*(C(0))+ b2*(C(1)); 
    
    double determinant = a1*b2 - a2*b1; 
    
    if (determinant == 0) 
    { 
        // The lines are parallel. This is simplified 
        // by returning a pair of FLT_MAX 
        intersection(0) = Double.MAX_VALUE;
        intersection(1) = Double.MAX_VALUE;
        return intersection; 
    } 
    else
    { 
        intersection(0) = (b2*c1 - b1*c2)/determinant; 
        intersection(1) = (a1*c2 - a2*c1)/determinant; 
        return intersection; 
    } 
} 

int main(int argc, char **argv) {
    //MatrixXd point_hom = simpleCube(5);
    MatrixXd arm = makeArm();
    MatrixXd extendedArm = extendArm(arm, 1);

    MatrixXd point_trans(4,1);
    point_trans(0,0) = 0;
    point_trans(1,0) = 0;
    point_trans(2,0) = 0;
    point_trans(3,0) = 1;

    namedWindow( "out_image", WINDOW_AUTOSIZE);
    int key;
    for(double z = 0; z < 10; z+= 0.001) {
        extendedArm = extendArm(arm, z);
        
        MatrixXd camera_intrinsic = computeCameraIntrinsicMatrix(304, 305, 320, 240);
        MatrixXd camera_projective_matrix = camera_intrinsic * computeIdealProjection(point_trans);

        MatrixXd new_arm = camera_projective_matrix * arm;
        MatrixXd new_extended_arm = camera_projective_matrix * extendedArm;

        Mat out_image = Mat::zeros(480, 640, CV_8UC3);
        int green_rgb[3] = {0, 255, 0};
        MatrixXd arm_in_2d = computeCartesianFromHomogeneous(new_arm);
        double m = (arm_in_2d(0, 0) - arm_in_2d(0, 1)) / (arm_in_2d(1, 0) - arm_in_2d(1, 1));
        
        double x_left = -arm_in_2d(1, 1) / m + arm_in_2d(0, 1); // 
        double y_top = -m * arm_in_2d(0, 1) + arm_in_2d(1, 1);
        double x_right = (480 - arm_in_2d(1, 1)) / m + arm_in_2d(0, 1);
        double y_bottom = m * (640 - arm_in_2d(0, 1)) + arm_in_2d(1, 1);
        // y1 - y = dy/dx(x1 - x)
        renderCrosshair(arm_in_2d, out_image, green_rgb);
        int red_rgb[3] = {255, 0, 0};
        renderCrosshair(computeCartesianFromHomogeneous(new_extended_arm), out_image, red_rgb);

        imshow("out_image", out_image);
        //usleep(50);
        waitKey(2);
        // key = waitKey(100);
        // if(key == 27){
        //     break;
        // }
    }

    return 0;
}