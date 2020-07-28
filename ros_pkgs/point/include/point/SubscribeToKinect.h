#ifndef SUBSCRIBE_TO_KINECT_H
#define SUBSCRIBE_TO_KINECT_H


class SubscribeToKinect {
private:
    /* Struct to store camera parameters */
    Eigen::Matrix3f cam_cal;
    Eigen::MatrixXf p_ideal; // Might be wrong /* p_ident * rotation_trans_mat = p_ideal */
    Eigen::MatrixXf p_real; // Also might not have correct sizing /* p_ideal * cam_cal = p_real*/
    Eigen::MatrixXf raw_2d_points;
    /* Depth values turned into 3D coordinates.  Each column is a point and 
     * the rows are x, y, z, and w. */ 
    Eigen::MatrixXf raw_3d_coords;
    /* All of the depth values at each location.  Given in terms of the distance from the camera*/
    cv::Mat depth_image;
    cv::Mat color_image;
    int **map2to3;
    op::Wrapper& opWrapper;
    geometry_msgs::Point elbow;
    geometry_msgs::Point wrist;

public:
    ros::Publisher marker_pub;

    /* Takes a depth value and converts it to a 3D x, y, and z. */
    void depth_to_3D(Eigen::Matrix3f cam_cal, int index, float &x, float &y, float &z) {
        int row = index / 640;
        int col = index % 640;
        z = depth_image.at<float>(row, col);  // Find actual depth
        x = (col + 0.5 - cam_cal(0, 0)) * cam_cal(0, 2) * z; // Perform frustum calculations
        y = (row + 0.5 - cam_cal(1, 1)) * cam_cal(1, 2) * z;
    }

    /* Converts all of the depth values in depth_image and puts the x, y, and z into raw_3D_coords. */
    void calculate_3D_coords() {
        raw_3d_coords(4, 307200);
        for (int index = 0; index < 480 * 640; index++) {
            float x, y, z;
            depth_to_3D(cam_cal, index, x, y, z); // First, calculate the actual x, y, and z in question
            raw_3d_coords(0, index) = x;   // Store these values
            raw_3d_coords(1, index) = y;
            raw_3d_coords(2, index) = z;
            raw_3d_coords(3, index) = 1.0; // Store a 1 for w into the depth matrix (scales in the color 2D matrix)
        }
    }

    void multiply_stuff() {
        p_ideal = p_ident * rotation_trans_mat;
        p_real = p_ident * cam_cal;
        raw_2d_points = p_real * raw_3d_coords;
    }

    /* Create the grid that holds the mapping.*/
    void initialize_point_grid() {
        int **map2to3 = (int **) malloc(sizeof(int *) * 480);
        for (int r = 0; r < 480; r++) {
            map2to3[r] = (int *) malloc(sizeof(int) * 640);
            for (int c = 0; c < 640; c++) {
                map2to3[r][c] = -1; // -1 initialized
            }
        }
    }

    /* Given a pixel in the form x, y, return the 3D value corresponding to that location. */
    geometry_msgs::Point get_3d_point(int x, int y) {
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
    void build2Dto3DMap() {
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

    /* Constructor for Subscribe to Kinect.  Initializes opWrapper and sets value for p_ident. */
    SubscribeToKinect(op::Wrapper& wrapper) : opWrapper(wrapper) { 
        set_p_ident(); 
        set_cam_call(205.46963709898583, 205.46963709898583, 320.5, 240.5);
        set_rotation_trans_mat();
    }

    void master_callback(const sensor_msgs::Image::ConstPtr &color, const sensor_msgs::Image::ConstPtr &depth) {
        ROS_INFO("OMG LOL");
        /* Save the image and depth map to class variables. */
        depth_image = cv_bridge::toCvCopy(depth, depth->encoding)->image;
        color_image = cv_bridge::toCvCopy(color, color->encoding)->image;

        cv::imshow("1", depth_image);
        cv::imshow("2", color_image);
        cv::waitKey(1);
        /* Now we run openpose on it. */
        get_key_points();
    }

    /* Get camera calibration values.  Adjust them and put into camera calibration matrix. */
    Eigen::Matrix3f camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
        float fx = 1/msg->K[0];
        float fy = 1/msg->K[4];
        float cx = msg->K[2];
        float cy = msg->K[5];
        cam_cal << fx,  0, cx,
                   0, fy,  0,
                   0,  0,  1;
        return cam_cal;
    }    

    // Adapted from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/01_body_from_image_default.cpp
    /* Get the Datum with the body key points. */
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> get_key_points() {
        const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(color_image);
        auto datum = opWrapper.emplaceAndPop(imageToProcess); //TODO:TDatumsSP
        /* Error checking datum processed. */
        if (datum != nullptr) {
            return datum;
        } else {
            op::opLog("Image could not be processed.", op::Priority::High);
            return NULL;
        }
    }

    /* Adjust a x, y, depth point to reflect the camera calibration. */
    geometry_msgs::Point transform_point(float x, float y, float depth_val) {
        geometry_msgs::Point point;
        point.x = (x + 0.5 - cam_cal(0, 2)) * cam_cal(0, 0) * depth_val;
        point.y = (y + 0.5 - cam_cal(1, 2)) * cam_cal(1, 1) * depth_val;
        point.z = depth_val;
        return point;
    }

    /* Print all of the body keypoints for the first person in the list. */
    void print_keypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumPtr) {
        try {
            if (datumPtr != nullptr && !datumPtr->empty()) {
                op::opLog("Body keypoints: " + datumPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
            }
        } catch (const std::exception& e) {
            op::error(e.what());
        }
    }

    // Publish the relevant keypoints for wrists and elbows.
    void show_body_keypoints(const op::Array<float>& keyPoints) {
        /* Set up parameters for points. */
        visualization_msgs::Marker points;
        points.header.frame_id = ROOT_TRANSFORM;
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;     
        points.color.a = 1.0;
        points.color.g = 1.0;
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        /* Push points back */
        for (const geometry_msgs::Point& p : find_end_points(keyPoints))
            points.points.push_back(p);
        visualization_msgs::Marker line_list(points);
        /* Set up parameters for line_list. */
        line_list.color.a = 1.0; //set alpha to make it visible, is this a thing for a line_list?
        line_list.color.r = 1.0;
        line_list.scale.x = 0.1;
        line_list.id = 1;
        line_list.type = visualization_msgs::Marker::LINE_LIST;        
        marker_pub.publish(points);
        marker_pub.publish(line_list);
    }

    // Locate limb positions 
    std::vector<geometry_msgs::Point> find_end_points(const op::Array<float>& keyPoints) {
        visualization_msgs::Marker points;
        std::vector<geometry_msgs::Point> ret;
        if (keyPoints.getSize(0)){
            for (int i = 3; i < 5; i++) {
                std::vector<int> xIndex{0, i, 0};
                std::vector<int> yIndex{0, i, 1};
                std::cout << "Set up indicies" << std::endl;
                float rawX = keyPoints.at({0, 3, 0});
                std::cout << "Read in rawX." << std::endl;
                float rawY = keyPoints.at(yIndex);
                std::cout << "Read in rawY." << std::endl;
                if (std::isnan(rawX) || std::isnan(rawY)) {
                    return ret;
                } 
                float rawZ = depth_image.at<float>(rawY, rawX);
                if (!(std::isnan(rawZ) || rawZ <= 0.001)) {
                    geometry_msgs::Point p = transform_point(rawX, rawY, rawZ);
                    std::cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << std::endl;
                    ret.push_back(p);
                    
                }
            }
        }
        if (ret.size() >= 2) {
            // TODO Heuristic to determine how much to scale by?
            extend_point(ret, 5);
        }
        return ret;
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
        print_points(ret);
        return ret;
    }
    
    /* Show points on vector in rviz. Creates a new geometry_msg::Point each time it is called. */
    void print_points (std::vector<geometry_msgs::Point> pts) {
        /* Create visualization msg and set parameters. */
        visualization_msgs::Marker points;
        points.header.frame_id = ROOT_TRANSFORM;
        points.header.stamp = ros::Time::now();
        points.ns = "points_and_lines";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;     
        points.color.a = 1.0;
        points.color.g = 1.0;
        points.scale.x = 0.2;
        points.scale.y = 0.2;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        /* Put points onto msg. */
        for (geometry_msgs::Point pt : pts) {
            points.points.push_back(pt);
        }
        /* Publish points to show on screen. */
        marker_pub.publish(points);
    }

    void get_rotation_trans_mat (geometry_msgs::TransformStamped transform) {
        //Eigen::MatrixXf pointMatrix(10,3);
        Eigen::MatrixXd pointMatrix;
        int rowPointer = 0;
        // Find the transform between the depth frame and the rgb frame
        // Grab rotation and translation and print
        tf2::Quaternion rotation; //=transform.transform.rotation;
        tf2::convert(transform.transform.rotation , rotation);
        geometry_msgs::Vector3 translation;
        translation = transform.transform.translation;
        tf2::Matrix3x3 m(rotation);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        std::cout << "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw<< "\n";
        double x, y, z, w;
        x = translation.x;
        y = translation.y;
        z = translation.z;
        std::cout << "x: " << x << " y: " << y << " z: " << z << "\n";
    }

};

#endif
