#include <Image.h>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <unistd.h>
 
using namespace Eigen;
using namespace std;

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