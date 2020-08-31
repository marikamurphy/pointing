/* This is where all things ros should go.  I would be ideal if we
 * didn't have to use ros anywhere but this file.
 */

#include <SubscribeToKinect.h>

class SubscribeToKinect {
    private:


    public:

    /* Constructor*/
    SubscribeToKinect::SubscribeToKinect() {}

    /* Return the color and depth image (in that order) in a vector. */
    vector<cv::Mat> SubscribeToKinect::get_color_and_depth() {
        vector<cv::Mat> ret;
        ret.push_back(color_image);
        ret.push_back(depth_image);
        return ret;
    }

    void SubscribeToKinect::save_cv_mats() {

    }

    /* Get camera calibration values.  The values are in the order of ???? */
    vector<double> SubscribeToKinect::camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr &msg) {
        vector<double> ret;
        ret.push_back(msg->K[0]);
        ret.push_back(msg->K[4]);
        ret.push_back(msg->K[2]);
        ret.push_back(msg->K[5]);
        return ret;
    }

    // Just pass in argc and argv from main program
    void SubscribeToKinect::logic(int argc, char **argv, bool FLAGS_disable_multi_thread) {
        // point_node is the node name
        ros::init(argc, argv, "point_node");
        // Start a roscpp node
        ros::NodeHandle node;
        
        // Use in open pose code
        // op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
        // try {
        //     // Use single thread for debugging
        //     if (FLAGS_disable_multi_thread) {
        //        opWrapper.disableMultiThreading();
        //     } //only use if openpose build with flags enabled
            
        //     op::WrapperStructPose wrapperStructPose{};
        //     wrapperStructPose.modelFolder = op::String("openpose/models");
            
        //     opWrapper.configure(wrapperStructPose);
        //     opWrapper.start();
        //     op::PoseModel& poseModel = wrapperStructPose.poseModel;
        //     const auto& pbpmBody25 = getPoseBodyPartMapping(poseModel);
        //     // std::cout << "get 3 (relbow?) " << pbpmBody25.at(3) << std::endl; 
        //     // std::cout << "get 4 (rwrist?) " << pbpmBody25.at(4) << std::endl;
        //     // std::cout << "pose model: " << static_cast<int>(poseModel) << std::endl;
        // } catch (const std::exception& e) {
        //     return -1;
        // }
        
        message_filters::Subscriber<sensor_msgs::Image> depth_image_test(node, "/camera/depth/image", 10);
        message_filters::Subscriber<sensor_msgs::Image> image_test(node, "/camera/rgb/image_color", 10);
        //message_filters::Subscriber ptCloud(node, "camera/depth/points", 10);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_test, depth_image_test);
        sync.registerCallback(boost::bind(&SubscribeToKinect::save_cv_mats, &this, _1, _2));
        ros::Subscriber cam_intrinsic_params = node.subscribe("/camera/depth_registered/sw_registered/camera_info",
            10, &SubscribeToKinect::cameraInfoCallback, &this);
        this.marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        
        std::cout<<"running"<<std::endl;
        ros::spin();
        std::cout<<"running"<<std::endl;

        // Run openposes on the cv mats: TODO much later

    }

}