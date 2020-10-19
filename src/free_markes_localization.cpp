#include <marker_localization/free_markes_localization.hpp>

bir::FreeMarkersLocalization::FreeMarkersLocalization():
    node_(), privateNode_("~"), imgTransport_(node_), update_(false)
{
    subImageTopic_ = imgTransport_.subscribe("/camera/image_raw", 1, 
                                            &bir::FreeMarkersLocalization::subImageTopicCallback, this);
    initializeMarkersLists();
    initializeCameraParameters();

    poseEstimator_ = std::make_unique<bir::MarkerPoseEstimator>(expectedMarkers_, cameraMatrix_, distCoeffs_);
}

void bir::FreeMarkersLocalization::initializeMarkersLists() {
    std::vector<int> lengths;
    ROS_ASSERT_MSG(privateNode_.getParam("markers_length", lengths), 
                         "markers_length did not found. Please fill it up the markers_length into rosparam server.");
    
    // Check for Invalid Markers Lengths Input
    std::vector<int>::iterator it = std::find_if(   std::begin(lengths),
                                                    std::end(lengths),
                                                    [](int element){return element <= 0;}
                                                );
    const bool length_greater_than_zero = (it == std::end(lengths));
    ROS_ASSERT_MSG(length_greater_than_zero, "Markers' lengths must be greater than zero. Fix it in the param file.");

    // Retrieve Markers Length with its IDs
    for(int length : lengths) {
        std::vector<int> markersIDs;
        const bool param_retrieved = privateNode_.getParam("markers_" + std::to_string(length), markersIDs);
        ROS_ASSERT_MSG(param_retrieved, "markers_%d did not found. Please fill it up into rosparam server.", length);
        expectedMarkers_.push_back(std::make_pair(length, markersIDs));
    }

    
    // expectedMarkersIds make some of the find_if operations easier 
    for (auto idVector : expectedMarkers_)
        expectedMarkersIds_.insert(expectedMarkersIds_.end(), idVector.second.begin(), idVector.second.end());
}

void bir::FreeMarkersLocalization::initializeCameraParameters() {
    std::vector<double> cameraMatrixValuesVector, distortionCoefVector;
    // Retrive Camera Matrix
    ROS_ASSERT_MSG(privateNode_.getParam("camera_matrix", cameraMatrixValuesVector), 
                           "camera_matrix did not found. \nPlease fill it up the camera_matrix into rosparam server.");
    
    // Retrive Distortion Vector
    ROS_ASSERT_MSG(privateNode_.getParam("camera_distortion", distortionCoefVector), 
                   "camera_distortion did not found. \nPlease fill it up the camera_distortion into rosparam server.");
    
    // Check for Invalid Camera Matrix Input
    ROS_ASSERT_MSG(cameraMatrixValuesVector.size() == 9, "Incorrect number of elements in the camera's matrix");

    // Check for Invalid Distortion Vector Input
    ROS_ASSERT_MSG(distortionCoefVector.size() == 5, "Incorrect number of elements in the camera_distortion vector");

    cameraMatrix_ = (cv::Mat1d(3, 3) <<
                        cameraMatrixValuesVector[0],    0,                              cameraMatrixValuesVector[2], 
                        0,                              cameraMatrixValuesVector[4],    cameraMatrixValuesVector[5], 
                        0,                              0,                              1                           );

    distCoeffs_ = (cv::Mat1d(1, distortionCoefVector.size()) << distortionCoefVector[0], 
                                                                distortionCoefVector[1],
                                                                distortionCoefVector[2],
                                                                distortionCoefVector[3],
                                                                distortionCoefVector[4] );
}

bir::MarkerVector bir::FreeMarkersLocalization::getDetectedMarkers() {
    update_ = false;
    // Get Every Marker in the image
    bir::MarkerVector detectMarkers = bir::MarkerDetect::GetInstance(dictionary_)->detect(image_);
    
    // Get Expected Markers in the image
    bir::MarkerVector detectedAndExpectedMarkersVector;
    for(int index = 0; index < detectMarkers.size(); index++) { 
        const bir::Marker marker = detectMarkers.at(index);
        
        //Search for ID inside expectedMarkersIds.
        std::vector<int>::iterator it;
        it = std::find_if(  expectedMarkersIds_.begin(), expectedMarkersIds_.end(), 
                            [index, marker](int element) { return marker == element; }  );

        if(it != expectedMarkersIds_.end()) {                      // If ID was found       
            detectedAndExpectedMarkersVector.pushBack(marker);     // Add into detectedAndExpectedMarkersVector
        }
    }
}

bir::MarkersTransforms bir::FreeMarkersLocalization::getMarkersTransforms(const bir::MarkerVector& markers) {
    return poseEstimator_->estimatePose(markers);
}

void bir::FreeMarkersLocalization::subImageTopicCallback(const sensor_msgs::ImageConstPtr& ros_image) {
    image_ = cv_bridge::toCvCopy(ros_image, "bgr8")->image; // Convert sensor_msgs into cv::Mat.
    if(!image_.empty()) 
        update_ = true;
    else {
        update_ = false;
    }
}
