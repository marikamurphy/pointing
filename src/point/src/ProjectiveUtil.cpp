#include "ProjectiveUtil.h"

/*  */
Eigen::MatrixXf getPIdent() {
    Eigen::MatrixXf pIdent = Eigen::ArrayXXf::Zero(3, 4);
    for(int i = 0; i < 3; i++)
        pIdent(i,i) = 1.0f;
    return pIdent;
}

/**/
Eigen::Matrix3f getCameraIntrinsicMatrix(float fx, float fy, float cx, float cy) {
    Eigen::Matrix3f cameraIntrinsic = Eigen::Matrix3f::Identity(3, 3);
    cameraIntrinsic(0,0) = fx;
    cameraIntrinsic(1,1) = fy;
    cameraIntrinsic(0,3) = cx;
    cameraIntrinsic(1,3) = cy;
    return cameraIntrinsic;
}

/* Takes a depth value and converts it to a 3D x, y, and z. */
void depth_to_3D(Eigen::Matrix3f cam_cal, int index, float &x, float &y, float &z, cv::Mat depth_image) {
    int row = index / 640;
    int col = index % 640;
    z = depth_image.at<float>(row, col);  // Find actual depth
    x = (col + 0.5 - cam_cal(0, 0)) * cam_cal(0, 2) * z; // Perform frustum calculations
    y = (row + 0.5 - cam_cal(1, 1)) * cam_cal(1, 2) * z; // TODO make sure this works.
}

/* perform calculations to transform 3D coordinates to 2D using camera calibration*/
Eigen::MatrixXf get_2d_points(Eigen::Matrix3f cam_cal, Eigen::MatrixXf p_ident, Eigen::MatrixXf rotation_trans_mat, Eigen::MatrixXf raw_3d_coords) {
    Eigen::MatrixXf p_ideal = p_ident * rotation_trans_mat;
    Eigen::MatrixXf p_real = p_ideal * cam_cal;
    Eigen::MatrixXf raw_2d_points = p_real * raw_3d_coords;
    return raw_2d_points;
}

/* Converts all of the depth values in depth_image and puts the x, y, and z into raw_3D_coords. */
void calculate_3D_coords(Eigen::MatrixXf raw_3d_coords, Eigen::Matrix3f cam_cal, cv::Mat depth_image) {
    raw_3d_coords(4, 307200);
    for (int index = 0; index < 480 * 640; index++) {
        float x, y, z;
        depth_to_3D(cam_cal, index, x, y, z, depth_image); // First, calculate the actual x, y, and z in question
        raw_3d_coords(0, index) = x;   // Store these values
        raw_3d_coords(1, index) = y;
        raw_3d_coords(2, index) = z;
        raw_3d_coords(3, index) = 1.0; // Store a 1 for w into the depth matrix (scales in the color 2D matrix)
    }
}

/* Create the grid that holds the mapping.*/
int ** initialize_point_grid() {
    int **map2to3 = (int **) malloc(sizeof(int *) * 480);
    for (int r = 0; r < 480; r++) {
        map2to3[r] = (int *) malloc(sizeof(int) * 640);
        for (int c = 0; c < 640; c++) {
            map2to3[r][c] = -1; // -1 initialized
        }
    }
    return map2to3;
}

/* Given a pixel in the form x, y, return the 3D value corresponding to that location. */
geometry_msgs::Point get_3d_point(int x, int y, int **map2to3) {
    int index = map2to3[x][y];
    geometry_msgs::Point ret;
    ret.x = (0, index);
    ret.y = (1, index);
    ret.z = (2, index);
    return ret;
}

/* Take the values of raw_2d_points and put them into a grid.
    * To retrieve a point of a pixel <x, y>, grab the index at <x, y> and 
    * index into raw_3d_points. */
void build2Dto3DMap(Eigen::MatrixXf raw_2d_points, int **map2to3) {
    for (int point = 0; point < 480 * 640; point++) {
        float scale = raw_2d_points(2, point);				// Find w
        if (std::isnan(scale)) continue;
        int px = (int) (raw_2d_points(0, point) / scale);  // Calculate pixel coordinate
        int py = (int) (raw_2d_points(1, point) / scale);
        if (map2to3[py][px] == -1) map2to3[py][px] = point; // If first visit, set the index mapping
        else if (raw_2d_points(2, map2to3[py][px]) > scale) // If the previous index was further from the camera than this one
            map2to3[py][px] = point; // Store current index
    }
}