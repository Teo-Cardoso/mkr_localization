#include <marker_localization/single_marker_identification.hpp>

tf::Quaternion quaternionFromVec3(const cv::Vec3d&);

bir::SingleMarkerIdentifier::SingleMarkerIdentifier(ros::NodeHandle& node): node_(node), imgTransport_(node_) {
    std::string imageTopic;
    int arucoDictionary;
    
    node_.param<bool>("debug", enableDebug_, false);
    ROS_WARN_COND(!node_.param<std::string>("image_topic", imageTopic, "/cv_camera/image_raw") && enableDebug_, 
                                               "image_topic did not found. Using default value: /cv_camera/image_raw");
    ROS_WARN_COND(!node_.param<bool>("enable_tf", enablePublishTF_, false) && enableDebug_, 
                                                                "enable_tf did not found. Using default value: false");
    ROS_WARN_COND(!node_.param<int>("marker_dictionary", arucoDictionary, 16) && enableDebug_, 
                                           "marker_dictionary did not found. Using default value: Aruco Orignal [16]");
    ROS_WARN_COND(!node_.param<std::string>("markerTF_name", markerTFName_, "id_") && enableDebug_, 
                                                               "markerTFName did not found. Using default value: id_");
    ROS_WARN_COND(!node_.param<std::string>("cameraTF_name", cameraTFName_, "/camera_link") && enableDebug_, 
                                                     "cameraTF_name did not found. Using default value: /camera_link");

    initializeMarkersLists();
    initializeCameraParameters();
    arucoDetector_ = bir::ArucoDetector(arucoDictionary);
    subImageTopic_ = imgTransport_.subscribe(imageTopic, 1, &bir::SingleMarkerIdentifier::subImageTopicCallback, this);
}

void bir::SingleMarkerIdentifier::subImageTopicCallback(const sensor_msgs::ImageConstPtr& ros_image) {
    image_ = cv_bridge::toCvCopy(ros_image, "bgr8")->image; // Convert sensor_msgs into cv::Mat.
    if(!image_.empty()) 
        update_ = true; // Allow identification update.
    else {
        update_ = false;
        ROS_WARN_COND(enableDebug_, "Receivied image is empty.");
    }
}

bir::MarkerVector bir::SingleMarkerIdentifier::detectMarkers() {
    update_ = false;

    cv::Mat markerImage = !roiRectangle_.empty() ? image_(roiRectangle_).clone() : image_.clone();  //Select between 
                                                                                                    // Full and ROI.
    ROS_WARN_COND(enableDebug_ && markerImage.empty(), "Image is empty.");
    bir::MarkerVector markerVector = arucoDetector_.detect(markerImage, cv::Point2f(roiOffset_[0], roiOffset_[1]));

    if(markerVector.empty() || roiCounter_ > 30) {
        roiOffset_[0] = 0; roiOffset_[1] = 0;
        roiRectangle_ = cv::Rect(0, 0, 0, 0);
        roiCounter_ = 0;
        return markerVector;
    } else roiCounter_++;

    bir::MarkerVector detectedMarkersVector;
    for(int index = 0; index < markerVector.size(); index++) { 
        std::vector<int>::iterator it;
        //Search for ID inside expectedMarkersIds.
        it = std::find_if(expectedMarkersIds_.begin(), expectedMarkersIds_.end(), 
                                 [index, markerVector](const int& element) { return element == markerVector[index]; });

        if(it != expectedMarkersIds_.end()) {                            // If ID was found       
            detectedMarkersVector.pushBack(markerVector.at(index));     // Add into detected markers vector.
        }
    }
    updateImageROI(detectedMarkersVector.getCorners(), image_.size());
    return detectedMarkersVector;
}

void bir::SingleMarkerIdentifier::estimatePose(bir::MarkerVector& marker_vector, bir::MarkerPose& markers_pose) {
    ROS_ASSERT_MSG(!marker_vector.empty(), "Marker Vector is empty. Function error: estimatePose.");
    std::vector<cv::Vec3d> rotationValues, translationValues;
    std::vector<double> areaValues;
    std::vector<int> markersIDsOrder;
    getRotationAndTranslationValues(marker_vector, rotationValues, translationValues, areaValues, markersIDsOrder);

    for (int index = 0; index < marker_vector.size(); index++) {
        tf::Transform pose;
        double translation[3] = {translationValues.at(index)[0], translationValues.at(index)[1], 
                                                                                       translationValues.at(index)[2]};
        pose.setOrigin(tf::Vector3(translation[0], translation[1], translation[2]));
        pose.setRotation(quaternionFromVec3(rotationValues.at(index)));

        markers_pose.ids.push_back(markersIDsOrder.at(index));
        markers_pose.areas.push_back(areaValues.at(index));
        markers_pose.poses.push_back(pose);
    }
}

void bir::SingleMarkerIdentifier::getRotationAndTranslationValues(bir::MarkerVector& marker_vector,
                                                                             std::vector<cv::Vec3d>& p_rotation_values,
                                                                          std::vector<cv::Vec3d>& p_translation_values,
                                                                                             std::vector<double>& area,
                                                                                          std::vector<int>& ids_orders)  
{
    if(marker_vector.empty()) return;
    for(auto categorie_list : expectedMarkers_) {
        std::vector<std::vector<cv::Point2f>> corner;
        fillUpCorner(marker_vector, categorie_list.second, corner, ids_orders);
        std::vector<cv::Vec3d> rotation_values, translation_values;
        cv::aruco::estimatePoseSingleMarkers(corner, 1e-3*categorie_list.first, cameraMatrix_, distCoeffs_,
                                                    /* Convert mm to cm */        rotation_values, translation_values);
        
        for(auto singleCorner : corner) area.push_back(cv::contourArea(singleCorner));
        p_rotation_values.insert(std::end(p_rotation_values), std::begin(rotation_values), std::end(rotation_values));
        p_translation_values.insert(std::end(p_translation_values), std::begin(translation_values), 
                                                                                         std::end(translation_values));
    }
}

void bir::SingleMarkerIdentifier::fillUpCorner(bir::MarkerVector& marker_vector,
                             std::vector<int>& length_id_list, std::vector<std::vector<cv::Point2f>>& corner_container,
                                                                                          std::vector<int>& ids_orders)
{
    for(int index = 0; index < marker_vector.size(); index++) { // Fill up corner vector.
        std::vector<int>::iterator it;
        it = std::find_if(std::begin(length_id_list), std::end(length_id_list), [marker_vector, index](int id){
                                                                                return (id == marker_vector[index]);});
        if(it != length_id_list.end()) { // If ID was found
            ids_orders.push_back(marker_vector[index]);
            corner_container.push_back(marker_vector.at(index).corner_);
        }
    }
}

void bir::SingleMarkerIdentifier::initializeMarkersLists() {
    std::vector<int> lengths;
    ROS_ASSERT_MSG(node_.getParam("markers_length", lengths), 
                         "markers_length did not found. \nPlease fill it up the markers_length into rosparam server.");
    std::vector<int>::iterator it = 
                           std::find_if(std::begin(lengths), std::end(lengths), [](int element){return element <= 0;});
    const bool length_greater_than_zero = (it == std::end(lengths));
    ROS_ASSERT_MSG(length_greater_than_zero, "Marker's length must be greater than zero. Fix it in the param file.");

    const char* ERROR = "Some markers_{LENGTH} did not found. Please fill it up into rosparam server.";
    for(int length : lengths) {
        std::vector<int> markersIDs;
        ROS_ASSERT_MSG(node_.getParam("markers_" + std::to_string(length), markersIDs), ERROR);
        expectedMarkers_.push_back(std::make_pair(length, markersIDs));
    }

    for (auto idVector : expectedMarkers_)
        expectedMarkersIds_.insert(expectedMarkersIds_.end(), idVector.second.begin(), idVector.second.end());
}

void bir::SingleMarkerIdentifier::initializeCameraParameters() {
    std::vector<double> cameraMatrixValuesVector, distortionCoefficientsVector;
    ROS_ASSERT_MSG(node_.getParam("camera_matrix", cameraMatrixValuesVector), 
                           "camera_matrix did not found. \nPlease fill it up the camera_matrix into rosparam server.");
    ROS_ASSERT_MSG(node_.getParam("camera_distortion", distortionCoefficientsVector), 
                   "camera_distortion did not found. \nPlease fill it up the camera_distortion into rosparam server.");
    cameraMatrix_ = (cv::Mat1d(3, 3) <<
                        cameraMatrixValuesVector[0],    0,                              cameraMatrixValuesVector[2], 
                        0,                              cameraMatrixValuesVector[4],    cameraMatrixValuesVector[5], 
                        0,                              0,                              1                           );
    distCoeffs_ = (cv::Mat1d(1, distortionCoefficientsVector.size()) << distortionCoefficientsVector[0], 
                                                                                      distortionCoefficientsVector[1],
                                                                                      distortionCoefficientsVector[2],
                                                                                      distortionCoefficientsVector[3],
                                                                                      distortionCoefficientsVector[4]);
}

void bir::SingleMarkerIdentifier::updateImageROI(const std::vector<std::vector<cv::Point2f>>& corners, 
                                                                                            const cv::Size& image_size) 
{
    std::vector<int> corner_x, corner_y;
    for (auto marker : corners) { // Move all corner x and y to one single list.
        for (auto corner : marker) {
            corner_x.push_back(corner.x);
            corner_y.push_back(corner.y);
        }
    }
    // Select Max and Min points from the corners.
    int max_x = *max_element(corner_x.begin(), corner_x.end());
    int max_y = *max_element(corner_y.begin(), corner_y.end());
    int min_x = *min_element(corner_x.begin(), corner_x.end());
    int min_y = *min_element(corner_y.begin(), corner_y.end());
    // Select Width and Height
    int width = (max_x - min_x) +  image_size.width*0.05;
    int height = (max_y - min_y) + image_size.height*0.05;
    // Adding Padding.
    min_x -= (int)(0.025*image_size.width);
    min_y -= (int)(0.025*image_size.height);
    // In case of Negative points.
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    // ROI greater than image size
    if(min_x + width >= image_size.width) {
        width = image_size.width - min_x - 1;
    }
    if(min_y + height >= image_size.height) {
        height = image_size.height - min_y - 1;
    }

    roiRectangle_ = cv::Rect(min_x, min_y, width, height);
    roiOffset_[0] = min_x;
    roiOffset_[1] = min_y;
}

tf::Quaternion quaternionFromVec3(const cv::Vec3d& rotation_value) {
    double rotationAngle = cv::norm(rotation_value);
    cv::Vec3d rotationAxis = rotation_value/ rotationAngle;
    tf::Quaternion quaternion;
    quaternion.setRotation(tf::Vector3(rotationAxis[0], rotationAxis[1], rotationAxis[2]), rotationAngle);
    return quaternion.normalized();
}