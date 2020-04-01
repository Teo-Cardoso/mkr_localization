#include <marker_localization/aruco_identification.hpp>

//! Aruco Detector

bir::ArucoDetector::ArucoDetector(int dictionary) {
    if(dictionary > cv::aruco::DICT_ARUCO_ORIGINAL) {
      dictionary = cv::aruco::DICT_ARUCO_ORIGINAL;
      throw ("Default Dictionary not Found. \n Set to Aruco Original. ");
    }

    setPredefinedDictionary(dictionary);
    
    _parameters = cv::aruco::DetectorParameters::create();
    _parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

bir::ArucoDetector::ArucoDetector(const ArucoDetector& aruco_detector) {
    this->_dictionary = aruco_detector._dictionary;
    this->_parameters = aruco_detector._parameters;
}

bir::ArucoDetector::~ArucoDetector(){

}

void bir::ArucoDetector::setPredefinedDictionary(int dictionary) {
    _dictionary = cv::aruco::getPredefinedDictionary(dictionary);
}

void bir::ArucoDetector::setCustomDictionary(int number_of_markers, int number_of_bits) {
    _dictionary = cv::aruco::generateCustomDictionary(number_of_markers, number_of_bits);
}

void bir::ArucoDetector::setParameters(cv::Ptr<cv::aruco::DetectorParameters> parameters){
    _parameters = parameters;
}

bir::MarkerVector bir::ArucoDetector::detect(const cv::Mat& img, cv::Point2f offset) {
    bir::MarkerVector markerVector;

    if (img.empty()) return markerVector;
    
    cv::aruco::detectMarkers(img, _dictionary, markerVector.corners_, markerVector.ids_, 
                                                                                _parameters, markerVector.rejected_);
    for(int index = 0; index < markerVector.size(); index++) {
        markerVector.areas_.push_back(cv::contourArea(markerVector.corners_.at(index)));
        
        if(offset == cv::Point2f(0, 0))
            continue;

        for (int index_offset = 0; index_offset < 4; index_offset++) {
            markerVector.corners_.at(index).at(index_offset) += offset; // Add the offset take off from the image to the
                                                                        // corner. Needed because the marker position
                                                                        // is computed from the corner position.
        }
    }

    return markerVector;
}

bir::Marker::Marker() {
    
}

bir::Marker::Marker(const bir::Marker& marker) {
    this->id_ = marker.id_;
    this->corner_ = marker.corner_;
    this->rejected_ = marker.rejected_;
}

bir::Marker::Marker(int id, std::vector<cv::Point2f> corner, std::vector<cv::Point2f> rejected) {
    this->id_ = id;
    this->corner_ = corner;
    this->rejected_ = rejected;
}

bir::Marker::~Marker() {

}

bool bir::Marker::operator==(const int id) {
    return (this->id_ == id);
}

bir::MarkerVector::MarkerVector() {

}

bir::MarkerVector::MarkerVector(const bir::MarkerVector& marker_vector) {
    this->ids_ = marker_vector.ids_;
    this->corners_ = marker_vector.corners_;
    this->rejected_ = marker_vector.rejected_;
}

bir::MarkerVector::MarkerVector(std::vector<int> ids, std::vector<std::vector<cv::Point2f>> corners, 
                                                                        std::vector<std::vector<cv::Point2f>> rejected) 
{
    this->ids_ = ids;
    this->corners_ = corners;
    this->rejected_ = rejected;
}

bir::MarkerVector::~MarkerVector() {

}

int bir::MarkerVector::operator[](const int index) const {
    return this->ids_.at(index);
}

bir::Marker bir::MarkerVector::at(const int index) const { 
    return bir::Marker(this->ids_.at(index), this->corners_.at(index), std::vector<cv::Point2f>{});
}

void bir::MarkerVector::pushBack(bir::Marker marker) {
    this->ids_.push_back(marker.id_);
    this->corners_.push_back(marker.corner_);
    this->rejected_.push_back(marker.rejected_);
}

std::string bir::MarkerVector::print() {
    if(this->empty()) return "[]";

    std::string print_msg = "[" + std::to_string(this->ids_.at(0));
    for (int index = 1; index < this->size(); index++) {
        print_msg += ", " + std::to_string(this->ids_.at(index));
    }
    print_msg += "]";
    return print_msg;
}

const std::vector<std::vector<cv::Point2f>> bir::MarkerVector::getCorners(){
    return this->corners_;
}

const std::vector<int> bir::MarkerVector::getIDs(){
    return this->ids_;
}

std::ostream& operator<<(std::ostream& os, bir::MarkerVector& obj) {
    return os << obj.print();
}