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

    // Adapted from https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/examples/tutorial_api_cpp/01_body_from_image_default.cpp
    /* Get the Datum with the body key points. */
    std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> SubscribeToKinect::get_key_points() {
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

    /* Print all of the body keypoints for the first person in the list. */
    void SubscribeToKinect::print_keypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumPtr) {
        try {
            if (datumPtr != nullptr && !datumPtr->empty()) {
                op::opLog("Body keypoints: " + datumPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
            }
        } catch (const std::exception& e) {
            op::error(e.what());
        }
    }

    // Publish the relevant keypoints for wrists and elbows.
    void SubscribeToKinect::show_body_keypoints(const op::Array<float>& keyPoints) {
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
    std::vector<geometry_msgs::Point> SubscribeToKinect::find_end_points(const op::Array<float>& keyPoints) {
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