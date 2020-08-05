#include "SubscribeToKinect.h"
#include "ProjectiveUtil.h"

#define ROOT_TRANSFORM "camera_rgb_optical_frame" //TODO: change for hsr
#define SCREEN_WIDTH    640
#define SCREEN_HEIGHT   480



    /* Constructor for Subscribe to Kinect.  Initializes opWrapper and sets value for p_ident. */
    SubscribeToKinect::SubscribeToKinect() { 
        //set_cam_call(205.46963709898583, 205.46963709898583, 320.5, 240.5); //TODO
        //set_rotation_trans_mat(); //TODO
    }

    /* recieve color and depth images from the camera and store*/
    void SubscribeToKinect::master_callback(const sensor_msgs::Image::ConstPtr &color, const sensor_msgs::Image::ConstPtr &depth) {
        ROS_INFO("OMG LOL");
        /* Save the image and depth map to class variables. */
        depth_image = cv_bridge::toCvCopy(depth, depth->encoding)->image;
        color_image = cv_bridge::toCvCopy(color, color->encoding)->image;

        cv::imshow("1", depth_image);
        cv::imshow("2", color_image);
        cv::waitKey(1);
        /* Now we run openpose on it. */
        // Comment in when using openpose again get_key_points();
    }

    /* Get camera calibration values.  Adjust them and put into camera calibration matrix. */
    Eigen::Matrix3f SubscribeToKinect::camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
        cam_cal = getCameraIntrinsicMatrix(msg->K[0], msg->K[4], msg->K[2], msg->K[5]);
        return cam_cal;
    }
    
    /* Show points on vector in rviz. Creates a new geometry_msg::Point each time it is called. */
    void SubscribeToKinect::print_points (std::vector<geometry_msgs::Point> pts) {
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

    /* for debugging purposes: prints the given transform*/
    void SubscribeToKinect::print_rotation_trans_mat (geometry_msgs::TransformStamped transform) {
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
