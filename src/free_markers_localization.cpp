#include <marker_localization/free_markers_localization.hpp>

bir::FreeMarkersLocalization::FreeMarkersLocalization():
    node_(), privateNode_("~"), imgTransport_(node_), poseEstimator_(nullptr)
{
    initializeMarkersLists();
    initializeCameraParameters();
    
    enablePublishImage_ = privateNode_.param<bool>("enable_publish_image", false);
    enablePublishPose_ = privateNode_.param<bool>("enable_publish_markers", true);
    enablePublishTF_ = privateNode_.param<bool>("enable_tf", true);
    
    markerTFSufix_ = privateNode_.param<std::string>("markers_sufix", "id_");
    cameraTFName_ = privateNode_.param<std::string>("camera_tf_name", "camera");
    dictionary_ = (cv::aruco::PREDEFINED_DICTIONARY_NAME)privateNode_.param<int>("markers_dictionary", 11);  

    poseEstimator_ = std::unique_ptr<MarkerPoseEstimator>(
        new MarkerPoseEstimator(expectedMarkers_, cameraMatrix_, distCoeffs_));
    
    subImageTopic_ = imgTransport_.subscribe("/camera/image_raw", 1, 
                                            &bir::FreeMarkersLocalization::subImageTopicCallback, this);
    
    if (enablePublishImage_) {
        image_transport::SubscriberStatusCallback managerSubscriberCb = boost::bind(
            &bir::FreeMarkersLocalization::managerSubscribers, this);
        pubImageTopic_ = imgTransport_.advertise("detected_markers/image", 1, managerSubscriberCb, managerSubscriberCb);
    }

    if (enablePublishPose_) {
        pubMarkersPoseTopic_ = node_.advertise<marker_localization::MarkerPoseArray>("detected_markers/pose", 10);
    }
}

void bir::FreeMarkersLocalization::initializeMarkersLists() {
    std::vector<int> lengths = privateNode_.param<std::vector<int>>("markers_length", std::vector<int>({100}));
    
    // Check for Invalid Markers Lengths Input
    const bool length_greater_than_zero = !std::any_of( std::begin(lengths),
                                                        std::end(lengths),
                                                        [](int element){return element <= 0;}   );
    ROS_ASSERT_MSG(length_greater_than_zero, "Markers' lengths must be greater than zero. Fix it in the param file.");

    // Retrieve Markers Length with its IDs
    for(int length : lengths) {
        std::vector<int> markersIDs = privateNode_.param<std::vector<int>>(
                                        "markers_" + std::to_string(length), std::vector<int>());
        expectedMarkersIds_.insert(expectedMarkersIds_.end(), markersIDs.begin(), markersIDs.end());
        expectedMarkers_.push_back(std::make_pair(length, markersIDs));
    }
}

void bir::FreeMarkersLocalization::initializeCameraParameters() {
    std::vector<double> cameraMatrixValuesVector = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> distortionCoefVector = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    // Retrive Camera Matrix
    cameraMatrixValuesVector = privateNode_.param<std::vector<double>>("camera_matrix", cameraMatrixValuesVector);

    // Retrive Distortion Vector
    distortionCoefVector = privateNode_.param<std::vector<double>>("camera_distortion", distortionCoefVector);

    // Check for Invalid Camera Matrix Input
    ROS_ASSERT_MSG(cameraMatrixValuesVector.size() == 9, 
        "Incorrect number of elements in the camera's matrix. Expected 9 instead of %d.",
        (int)cameraMatrixValuesVector.size() );

    // Check for Invalid Distortion Vector Input
    ROS_ASSERT_MSG(distortionCoefVector.size() == 5, 
        "Incorrect number of elements in the camera_distortion vector expected 5 instead of %d.", 
        (int)distortionCoefVector.size() );

    cameraMatrix_ = (cv::Mat1d(3, 3) <<
                        cameraMatrixValuesVector[0],    0,                              cameraMatrixValuesVector[2], 
                        0,                              cameraMatrixValuesVector[4],    cameraMatrixValuesVector[5], 
                        0,                              0,                              1                           );

    distCoeffs_ = (cv::Mat1d(1, 5) << distortionCoefVector[0], 
                                        distortionCoefVector[1],
                                        distortionCoefVector[2],
                                        distortionCoefVector[3],
                                        distortionCoefVector[4] );
}

bir::MarkerVector bir::FreeMarkersLocalization::getDetectedMarkers(const cv::Mat& image) {
    // Get Every Marker in the image
    bir::MarkerVector detectMarkers = bir::MarkerDetect::GetInstance(dictionary_)->detect(image);
    
    // Get Expected Markers in the image
    bir::MarkerVector detectedAndExpectedMarkersVector;
    for(int index = 0; index < detectMarkers.size(); index++) { 
        const bir::Marker marker = detectMarkers.at(index);
        
        //Search for ID inside expectedMarkersIds.
        if (std::any_of( expectedMarkersIds_.begin(), expectedMarkersIds_.end(), 
                        [index, marker](int element) { return marker == element; })) 
        {
            detectedAndExpectedMarkersVector.pushBack(marker);
        }
    }

    return detectedAndExpectedMarkersVector;
}

void bir::FreeMarkersLocalization::publishTF(const bir::MarkersTransforms& markers_transforms) {
    if (!enablePublishTF_) return;
    
    std::vector<geometry_msgs::TransformStamped> transforms;
    const ros::Time stampTime = ros::Time::now();

    for(int index = 0; index < markers_transforms.size(); index++) {
        geometry_msgs::TransformStamped transform;
        transform.header.frame_id = cameraTFName_;
        transform.header.stamp = stampTime;
        transform.child_frame_id = markerTFSufix_ + std::to_string(markers_transforms.ids.at(index));
        transform.transform = tf2::toMsg(markers_transforms.transforms.at(index));

        transforms.push_back(std::move(transform));
    }

    tfBroadcaster_.sendTransform(transforms);
}

void bir::FreeMarkersLocalization::publishPose(const bir::MarkersTransforms& markers_transforms) {
    if (!enablePublishPose_) return;

    marker_localization::MarkerPoseArray markersPoses;
    markersPoses.header.frame_id = cameraTFName_;
    markersPoses.header.stamp = ros::Time::now();

    for(int index = 0; index < markers_transforms.size(); index++) {
        marker_localization::MarkerPose markerPose;
        markerPose.marker_id = markers_transforms.ids.at(index);
        markerPose.marker_pose = tf2::toMsg(markers_transforms.transforms.at(index));
        markerPose.error = markers_transforms.projections_erro.at(index);

        markersPoses.markers.push_back(std::move(markerPose));
    }

    pubMarkersPoseTopic_.publish(markersPoses);
}

void bir::FreeMarkersLocalization::publishImage(cv::Mat& image, bir::MarkerVector& marker_vector) {
    if (image.empty() || !enablePublishImage_) return;
    cv::aruco::drawDetectedMarkers(image, marker_vector.getCorners(), marker_vector.getIDs());
    sensor_msgs::ImagePtr ros_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pubImageTopic_.publish(ros_image);
}

void  bir::FreeMarkersLocalization::runDetectionAndEstimation(cv::Mat& image) {
    MarkerVector detectedMarkers = getDetectedMarkers(image);
    MarkersTransforms markersTransforms = poseEstimator_->estimatePose(detectedMarkers);
    
    publishTF(markersTransforms);
    publishPose(markersTransforms);
    publishImage(image, detectedMarkers);
}

void bir::FreeMarkersLocalization::subImageTopicCallback(const sensor_msgs::ImageConstPtr& ros_image) {
    cv::Mat image = cv_bridge::toCvCopy(ros_image, "bgr8")->image; // Convert sensor_msgs into cv::Mat.
    if(!image.empty()) {
        ROS_INFO("RUNNING");
        runDetectionAndEstimation(image);
    }
}

void bir::FreeMarkersLocalization::managerSubscribers() {
    enablePublishImage_ = (bool)(pubImageTopic_.getNumSubscribers() > 0);
}