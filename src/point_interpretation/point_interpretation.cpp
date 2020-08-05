#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>
 
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

void renderCrosshair(int x, int y, cv::Mat &in_mat) {
    line(in_mat, Point(x - 5,y), Point(x+5,y), Scalar(0,255,0));
    line(in_mat, Point(x,y-5), Point(x,y+5), Scalar(0,255,0));
}

void renderCrosshair(MatrixXd in_pts, cv::Mat &in_mat) {
    for(int i = 0; i < in_pts.cols(); i++)
        renderCrosshair(in_pts(0,i), in_pts(1,i), in_mat);
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

int main(int argc, char **argv) {
    MatrixXd point_hom = simpleCube(5);

    MatrixXd point_trans(4,1);
    point_trans(0,0) = 0;
    point_trans(1,0) = 0;
    point_trans(2,0) = 50;
    point_trans(3,0) = 1;

    namedWindow( "out_image", WINDOW_AUTOSIZE);

    for(double z = 0; z < 1000.0; z+= 0.01) {
        point_trans(0,0) = z;
        
        MatrixXd camera_intrinsic = computeCameraIntrinsicMatrix(304, 305, 320, 240);
        MatrixXd camera_projective_matrix = camera_intrinsic * computeIdealProjection(point_trans);

        MatrixXd new_point_hom = camera_projective_matrix * point_hom;

        Mat out_image = Mat::zeros(480, 640, CV_8UC3);
        renderCrosshair(computeCartesianFromHomogeneous(new_point_hom), out_image);

        imshow("out_image", out_image);

        waitKey(1);
    }

    return 0;
}