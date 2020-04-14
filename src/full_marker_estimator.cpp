#include <marker_localization/full_marker_estimator.hpp>

std::vector<double> computeCovarianceMatrix(const std::deque<tf::Transform>& transform_raw);

bir::FullMarkerEstimator::FullMarkerEstimator(ros::NodeHandle& node): 
    node_(node)
{
    std::string posePublishTopic;
    tf::StampedTransform markerTransform;

    node_.param<bool>("debug", enableDebug_, false);
    ROS_WARN_COND(!node_.param<bool>("enable_tf", enablePublishTF_, false) && enableDebug_, 
                                                                "enable_tf did not found. Using default value: false");
    ROS_WARN_COND(!node_.param<bool>("robot_as_parent", robotAsParent_, false) && enableDebug_, 
                                                          "robot_as_parent did not found. Using default value: false");
    ROS_WARN_COND(!node_.param<bool>("static_variance", staticVariance_, false) && enableDebug_, 
                                                          "static_variance did not found. Using default value: false");
    ROS_WARN_COND(!node_.param<bool>("static_camera_transform", staticBaseCameraTransform_, false) && enableDebug_, 
                                                  "static_camera_transform did not found. Using default value: false");
    ROS_WARN_COND(!node_.param<std::string>("map_tf_name", mapTFName_, "map") && enableDebug_,
                                                                  "mapTFName did not found. Using default value: map");
    ROS_WARN_COND(!node_.param<std::string>("base_tf_name", baseTFName_, "base_link") && enableDebug_,
                                                           "baseTFName did not found. Using default value: base_link");
    ROS_WARN_COND(!node_.param<std::string>("marker_tf_name", markerTFName_, "id_") && enableDebug_,
                                                               "markerTFName did not found. Using default value: id_");
    ROS_WARN_COND(!node_.param<std::string>("camera_tf_name", cameraTFName_, "camera_optical_frame") && enableDebug_,
                                             "camera_tf_name did not found. Using default value: camera_optical_frame");
    ROS_WARN_COND(!node_.param<std::string>("pose_topic", posePublishTopic, "pose") && enableDebug_,
                                                            "pose_topic did not found. Using default value: pose");   
    ROS_WARN_COND(!node_.param<bool>("static_marker", staticMarker_, false) && enableDebug_, 
                                                            "static_marker did not found. Using default value: false");   
                                                                   
           
    posePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(posePublishTopic, 5);
    if(staticVariance_) {
        ROS_ASSERT_MSG(node_.getParam("variance_matrix", varianceValues_), 
                       "variance_matrix did not found. \nPlease fill it up the variance_matrix into rosparam server.");
    }
    if(staticMarker_) {
        ROS_ASSERT_MSG(node_.getParam("static_markers_id", staticMarkersId_), 
                   "static_markers_id did not found. \nPlease fill it up the static_markers_id into rosparam server.");
        for(int index = 0; index < staticMarkersId_.size(); index++) {
            try {
                ROS_ASSERT_MSG(
                    transformListener_.waitForTransform(mapTFName_,  markerTFName_ + std::to_string(index), 
                    ros::Time(0), ros::Duration(1)), 
                              "Some static aruco is not connected. Please check your launch file.\n Try: rqt_tf_tree.");
                transformListener_.lookupTransform(mapTFName_, (markerTFName_ + std::to_string(index)), 
                                                                                        ros::Time(0), markerTransform);
                staticMarkersTransforms_.push_back(markerTransform);
            } catch (tf::ExtrapolationException &e) {
                ROS_ERROR_STREAM_COND(enableDebug_, e.what());
                break; // Break the loop - If any tf failed, the processing will be canceled.
            }
        }
    }
    if(staticBaseCameraTransform_) {
        try {
            ROS_ASSERT_MSG(
                transformListener_.waitForTransform(cameraTFName_, baseTFName_, ros::Time(0), ros::Duration(2)),
                        "Camera and Base are not connected. Please check them transforms names. \n Try: rqt_tf_tree.");
                transformListener_.lookupTransform(cameraTFName_, baseTFName_, ros::Time(0), baseCameraTransform_);
        } catch (tf::ExtrapolationException &e) {
            ROS_ERROR_STREAM_COND(enableDebug_, e.what());
            ROS_ASSERT_MSG(
                transformListener_.waitForTransform(cameraTFName_, baseTFName_, ros::Time(0), ros::Duration(2)),
                        "Camera and Base are not connected. Please check them transforms names. \n Try: rqt_tf_tree.");
        }
    }
}

bir::FullMarkerEstimator::~FullMarkerEstimator()
{
}

void bir::FullMarkerEstimator::estimatePose(bir::MarkerPose& marker_poses, tf::Transform& base_transform) {
    std::pair<std::vector<double>, std::vector<tf::Transform>> raw_transforms;
    tf::Transform cameraTransform;

    estimateCameraPositions(marker_poses, raw_transforms);
    if(!raw_transforms.first.size()) return; // No Transforms. 
    getCameraPoseAverage(raw_transforms, cameraTransform);
    if(staticBaseCameraTransform_) base_transform = (cameraTransform * baseCameraTransform_); 
    else {
        tf::StampedTransform baseCameraTransform;
        try {
            ROS_ASSERT_MSG(
                transformListener_.waitForTransform(cameraTFName_, baseTFName_, ros::Time(0), ros::Duration(0.5)),
                        "Camera and Base are not connected. Please check them transforms names. \n Try: rqt_tf_tree.");
                transformListener_.lookupTransform(cameraTFName_, baseTFName_, ros::Time(0), baseCameraTransform);
        } catch (tf::ExtrapolationException &e) {
            ROS_ERROR_STREAM_COND(enableDebug_, e.what());
            ROS_ASSERT_MSG(
                transformListener_.waitForTransform(cameraTFName_, baseTFName_, ros::Time(0), ros::Duration(0.5)),
                        "Camera and Base are not connected. Please check them transforms names. \n Try: rqt_tf_tree.");
        }
        
        base_transform = (cameraTransform * baseCameraTransform);
    }
}

void bir::FullMarkerEstimator::estimateCameraPositions(bir::MarkerPose& marker_poses, 
                                          std::pair<std::vector<double>, std::vector<tf::Transform>>& raw_camera_poses) 
{
    std::vector<double>& markers_area = raw_camera_poses.first;
    std::vector<tf::Transform>& raw_camera_transforms = raw_camera_poses.second;

    for(int index = 0; index < marker_poses.size(); index++) {
        tf::StampedTransform markerTransform; // Get the the Marker transform relative to the World
        // std::vector<int>::iterator it;
        int transformVectorPose = 9999;
        //Search for ID inside expectedMarkersIds.
        for (int i = 0; i<staticMarkersId_.size();i++){
            if(staticMarkersId_.at(i) == marker_poses.ids.at(index)){
                transformVectorPose = i;
            };
        }
        // it = std::find_if(staticMarkersId_.begin(), staticMarkersId_.end(), 
        //                          [index, marker_poses](const int& element) { return element == marker_poses.ids.at(index);});
        if(transformVectorPose!=9999){
            markerTransform = staticMarkersTransforms_[transformVectorPose];
        } else{
            try {
                bool transformAvailable = transformListener_.waitForTransform(mapTFName_,  markerTFName_ + 
                                            std::to_string(marker_poses.ids.at(index)), ros::Time(0), ros::Duration(0.1));
                if(!transformAvailable) continue;
                transformListener_.lookupTransform(mapTFName_, markerTFName_ + std::to_string(marker_poses.ids.at(index)), 
                                                                                            ros::Time(0), markerTransform);
            } catch (tf::ExtrapolationException &e) {
                ROS_ERROR_STREAM_COND(enableDebug_, e.what());
                raw_camera_transforms.clear(); // Clean rawCameraPoses
                break; // Break the loop - If any tf failed, the processing will be canceled.
            }
        }
        // Get the Camera transform relative to the World (wTc = wTm * mTc)
        tf::Transform cameraWorldTransform = markerTransform * marker_poses.poses.at(index).inverse();
    
        markers_area.push_back(marker_poses.areas.at(index));
        raw_camera_transforms.push_back(cameraWorldTransform);
    }
}

void bir::FullMarkerEstimator::getCameraPoseAverage(
                  std::pair<std::vector<double>, std::vector<tf::Transform>>& raw_camera_poses, tf::Transform& average)
{
    std::vector<double>& markers_area = raw_camera_poses.first;
    std::vector<tf::Transform>& raw_camera_transforms = raw_camera_poses.second;
    tf::Vector3 positionAverage(0, 0, 0);
    cv::Mat quaternionAverageMatrix = cv::Mat::zeros(4, 4, CV_64F);
    
    for(int index = 0; index < markers_area.size(); index++) {
        /* Insert element translation into final average. */
        positionAverage += raw_camera_transforms.at(index).getOrigin() * markers_area.at(index) * 1e-2;
        /* Insert element rotation into final average. */
        cv::Mat quaternion(4, 1, CV_64F);
        quaternion.at<double>(0, 0) = raw_camera_transforms.at(index).getRotation().getW();
        quaternion.at<double>(1, 0) = raw_camera_transforms.at(index).getRotation().getX();
        quaternion.at<double>(2, 0) = raw_camera_transforms.at(index).getRotation().getY();
        quaternion.at<double>(3, 0) = raw_camera_transforms.at(index).getRotation().getZ();
        quaternionAverageMatrix += (markers_area.at(index) * 1e-2 * (quaternion * quaternion.t()));
    }
    
    const double TOTALWEIGHT = std::accumulate(markers_area.begin(), markers_area.end(), 0.00) * 1e-2;
    positionAverage /= TOTALWEIGHT;
    quaternionAverageMatrix /= TOTALWEIGHT;

    /* Compute Eigen Values and Vectors from Q */
    cv::Mat E, V;
    cv::eigen(quaternionAverageMatrix, E, V);
    std::vector<double> eigenValues = vectorFromMat<double>(E);
    int index = std::max_element(eigenValues.begin(), eigenValues.end()) - eigenValues.begin();
    /* Find Quaternion Average */
    tf::Quaternion quaternionAverage;
    quaternionAverage.setW(V.at<double>(index, 0));
    quaternionAverage.setX(V.at<double>(index, 1));
    quaternionAverage.setY(V.at<double>(index, 2));
    quaternionAverage.setZ(V.at<double>(index, 3));
    quaternionAverage = quaternionAverage.normalized();

    average.setOrigin(positionAverage);
    average.setRotation(quaternionAverage);
}

void bir::FullMarkerEstimator::publish(const tf::Transform& base_transform) { //TODO: Add TF Broadcaster
    if(oldCameraTransforms_.size() > 10) oldCameraTransforms_.pop_front();
        oldCameraTransforms_.push_back(base_transform);
    
    if(!staticVariance_) { // Compute the new variance
        varianceValues_ = computeCovarianceMatrix(oldCameraTransforms_);
    }
    
    geometry_msgs::PoseWithCovarianceStamped basePoseMsg;
    basePoseMsg.header.stamp = ros::Time::now();
    basePoseMsg.header.frame_id = mapTFName_;
                
    basePoseMsg.pose.pose.position.x = base_transform.getOrigin()[0];
    basePoseMsg.pose.pose.position.y = base_transform.getOrigin()[1];
    basePoseMsg.pose.pose.position.z = base_transform.getOrigin()[2];

    basePoseMsg.pose.pose.orientation.w = base_transform.getRotation().w();
    basePoseMsg.pose.pose.orientation.x = base_transform.getRotation().x();
    basePoseMsg.pose.pose.orientation.y = base_transform.getRotation().y();
    basePoseMsg.pose.pose.orientation.z = base_transform.getRotation().z();
    
    boost::array<double, 36> varianceValuesArray;
    for(int index = 0; index < 36; index++) varianceValuesArray.at(index) = varianceValues_.at(index);

    basePoseMsg.pose.covariance = varianceValuesArray;

    posePublisher_.publish(basePoseMsg);
}

std::vector<double> computeCovarianceMatrix(const std::deque<tf::Transform>& transform_raw) {
    std::vector<double> covariance;
    covariance.resize(36);
    
    if(transform_raw.size() <= 1) return covariance; // If data is empty or there is just one element there is no 
                                                                                                         // covariance.

    cv::Mat dataMatrix((int)transform_raw.size(), 6, CV_64F);
    int row = 0;

    for(tf::Transform data : transform_raw) {
        /* Translation */
        for(int index = 0; index < 3; index++) dataMatrix.at<double>(row, index) = data.getOrigin()[index];
        /* Rotation */
        double roll, pitch, yaw;
        tf::Matrix3x3(data.getRotation()).getRPY(roll, pitch, yaw);
        dataMatrix.at<double>(row, 3) = roll;
        dataMatrix.at<double>(row, 4) = pitch;
        dataMatrix.at<double>(row, 5) = yaw;

        row++; // Increase row number to change the matrix row.
    }

    cv::Mat deviationMatrix((int)transform_raw.size(), 6, CV_64F);
    cv::Mat one = cv::Mat::ones((int)transform_raw.size(), (int)transform_raw.size(), CV_64F);
    // Deviation[nx6] = Data[nx6] - MatrixOfOnes[nxn] * Data[nx6]
    deviationMatrix = dataMatrix - (one * dataMatrix) / transform_raw.size();
    cv::Mat covarianceMatrix = deviationMatrix.t() * deviationMatrix; // Covariance  = Deviation' * Deviation
    
    /* Converting CV::MAT to std::vector<double> */
    if (covarianceMatrix.isContinuous()) {
        covariance.assign((double*)covarianceMatrix.data, (double*)covarianceMatrix.data + covarianceMatrix.total());
    } else {
        for (int i = 0; i < covarianceMatrix.rows; ++i) {
            covariance.insert(covariance.end(), covarianceMatrix.ptr<double>(i), 
                                                              covarianceMatrix.ptr<double>(i) + covarianceMatrix.cols);
        }
    }

    return covariance;
}