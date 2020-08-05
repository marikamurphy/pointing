#include "ProjectiveUtil.h"

/*  Creates a 3x4 identity matrix for use in camera calibration calcs */
Eigen::MatrixXf getPIdent() {
    Eigen::MatrixXf pIdent = Eigen::ArrayXXf::Zero(3, 4);
    for(int i = 0; i < 3; i++)
        pIdent(i,i) = 1.0f;
    return pIdent;
}

/* Creates the camera intrinsic matrix*/
Eigen::Matrix3f getCameraIntrinsicMatrix(float fx, float fy, float cx, float cy) {
    Eigen::Matrix3f cameraIntrinsic = Eigen::Matrix3f::Identity(3, 3);
    cameraIntrinsic(0,0) = fx;
    cameraIntrinsic(1,1) = fy;
    cameraIntrinsic(0,2) = cx;
    cameraIntrinsic(1,2) = cy;
    return cameraIntrinsic;
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

/* Takes a depth value and converts it to a 3D x, y, and z. Called by calculate_3D_coords. */
void depth_to_3D(Eigen::Matrix3f cam_cal, int index, float &x, float &y, float &z, cv::Mat depth_image) {
    int row = index / 640;
    int col = index % 640;
    z = depth_image.at<float>(row, col);  // Find actual depth
    x = (col + 0.5 - cam_cal(0, 0)) * cam_cal(0, 2) * z; // Perform frustum calculations
    y = (row + 0.5 - cam_cal(1, 1)) * cam_cal(1, 2) * z; // TODO make sure this works.
}

/* perform calculations to transform 3D coordinates to 2D using camera calibration*/
Eigen::MatrixXf get_2d_points(Eigen::Matrix3f cam_cal, Eigen::MatrixXf rotation_trans_mat, Eigen::MatrixXf raw_3d_coords) {
    /* 3x3 * 3x4 * 4x307200 = 3x307200 */
    return cam_cal * rotation_trans_mat * raw_3d_coords;
}

/* Create the grid that holds the mapping.*/
int **initialize_point_grid() {
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

/* Adjust a x, y, depth point to reflect the camera calibration. */
geometry_msgs::Point transform_point(float x, float y, float depth_val, Eigen::Matrix3f cam_cal) {
    geometry_msgs::Point point;
    point.x = (x + 0.5 - cam_cal(0, 2)) * cam_cal(0, 0) * depth_val;
    point.y = (y + 0.5 - cam_cal(1, 2)) * cam_cal(1, 1) * depth_val;
    point.z = depth_val;
    return point;
}

// Find what's being pointed at out in space
std::vector<geometry_msgs::Point> extend_point (std::vector<geometry_msgs::Point> origin, int scalar) {
    geometry_msgs::Point vec;
    // Create vector
    vec.x = origin[1].x - origin[0].x;
    vec.y = origin[1].y - origin[0].y;
    vec.z = origin[1].z - origin[0].z;
    // Extend vector
    geometry_msgs::Point end_vec;
    end_vec.x = vec.x * scalar;
    end_vec.y = vec.y * scalar;
    end_vec.z = vec.z * scalar;
    std::vector<geometry_msgs::Point> ret;
    // Create end point
    geometry_msgs::Point end_pt;
    end_pt.x = origin[0].x + end_vec.x;
    end_pt.y = origin[0].y + end_vec.y;
    end_pt.z = origin[0].z + end_vec.z;
    ret.push_back(origin[0]);
    ret.push_back(end_pt);
    return ret;
}