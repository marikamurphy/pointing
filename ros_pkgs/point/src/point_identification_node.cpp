

int main(int argc, char **argv) {
 ros::init(argc, argv, "point_node");
    ros::NodeHandle node;
    /* Set up opWrapper and point to the right model folder. */
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    try {
        op::WrapperStructPose wrapperStructPose{};
        wrapperStructPose.modelFolder = op::String("openpose/models");
        opWrapper.configure(wrapperStructPose);
        opWrapper.start();
    } catch (const std::exception& e) {
        return -1;
    }

    /* Create the subscribe to kinect object. */
    SubscribeToKinect kSub(opWrapper);
    /* Grab the color and depth image. */
    message_filters::Subscriber<sensor_msgs::Image> depth_image_test(node, "/camera/depth/image", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_test(node, "/camera/rgb/image_color", 10); //TODO: fix for hsr

    //message_filters::Subscriber ptCloud(node, "camera/depth/points", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_test, depth_image_test);
    sync.registerCallback(boost::bind(&SubscribeToKinect::master_callback, &kSub, _1, _2));

    kSub.marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    geometry_msgs::TransformStamped transform;
    

        while (node.ok()) {
            try{                                    
            transform = buffer.lookupTransform("/head_rgbd_sensor_rgb_fra/me", "/head_rgbd_sensor_depth_registered_frame",  
                                    ros::Time(0));
            }
            catch (tf2::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            }
            kSub.get_rotation_trans_mat(transform);
        }
    
    ros::spin();

    return 0;

}