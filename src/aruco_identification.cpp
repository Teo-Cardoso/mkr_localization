#include <marker_localization/aruco_identification.hpp>

bir::Aruco::Aruco(int dictionary) {
    if(dictionary > cv::aruco::DICT_ARUCO_ORIGINAL) {
      dictionary = cv::aruco::DICT_ARUCO_ORIGINAL;
      throw ("Default Dictionary not Found. \n Set to Aruco Original. ");
    }

    setPredefinedDictionary(dictionary);
    
    _parameters = cv::aruco::DetectorParameters::create();
    _parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

bir::Aruco::~Aruco(){

}

void bir::Aruco::setPredefinedDictionary(int dictionary) {
    _dictionary = cv::aruco::getPredefinedDictionary(dictionary);
}

void bir::Aruco::setCustomDictionary(int number_of_markers, int number_of_bits) {
    _dictionary = cv::aruco::generateCustomDictionary(number_of_markers, number_of_bits);
}

void bir::Aruco::setParameters(cv::Ptr<cv::aruco::DetectorParameters> parameters){
    _parameters = parameters;
}

bool bir::Aruco::markers::operator==(const int& id) {
    std::vector<int>::iterator index = std::find(this->id.begin(), this->id.end(), (id));
    return (index != this->id.end());
}

int bir::Aruco::markers::operator[](const int& id) {
    std::vector<int>::iterator iterator = std::find(this->id.begin(), this->id.end(), (id));
    if(iterator != this->id.end()) {
        return std::distance(this->id.begin(), iterator);;
    } else {
        return (-1);
    }
}

bool bir::Aruco::draw(cv::Mat& image_input, markers markers_input) {
    try {
        if(image_input.empty()) return false;
        cv::aruco::drawDetectedMarkers(image_input, markers_input.corner, markers_input.id);
    } catch (cv::Exception& e) {
        return false;
    }
    return true;
}


bool bir::Aruco::markers::drawTo(cv::Mat& image_input_output) {
    try {
        if(image_input_output.empty()) return false;
        cv::aruco::drawDetectedMarkers(image_input_output, this->corner, this->id);
    } catch (cv::Exception& e) {
        return false;
    }
    return true;
}

bir::Aruco::markers bir::Aruco::operator()(const cv::Mat& img, cv::Point2f offset) {
    markers markOutput;
    if (!img.empty()) {
        cv::aruco::detectMarkers(img, _dictionary, markOutput.corner, markOutput.id, _parameters, markOutput.rejected); 
        markOutput.size = markOutput.id.size();
        for(int index = 0; index < markOutput.size; index++) {
            double area = cv::contourArea(markOutput.corner.at(index));
            markOutput.totalArea += area;
            markOutput.area.push_back(area);
            if(offset != cv::Point2f(0, 0)) {
                for (int index_offset = 0; index_offset < 4; index_offset++) {
                    markOutput.corner.at(index).at(index_offset) = markOutput.corner.at(index).at(index_offset) + offset;
                }
            }
        }
    }

    return markOutput;
}

void bir::Aruco::markers::clean(){
    this->id.clear();
    this->corner.clear();
    this->rejected.clear();
    this->area.clear();
    this->totalArea = 0;
    this->size = 0;
}

void bir::Aruco::markers::push_back(markers& source, unsigned int index) {
    this->id.push_back(source.id.at(index));
    this->corner.push_back(source.corner.at(index));
    this->area.push_back(source.area.at(index));
    this->totalArea += source.area.at(index);
    this->size++;
}

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