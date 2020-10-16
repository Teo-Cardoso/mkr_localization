#include <marker_localization/marker_detect.hpp>

bir::MarkerDetect* bir::MarkerDetect::markerDetect_ = nullptr;

bir::MarkerDetect* bir::MarkerDetect::GetInstance(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary)
{
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */
    if(markerDetect_ == nullptr) {
        markerDetect_ = new MarkerDetect(dictionary);
    } else {
        markerDetect_->setDictionary(dictionary);
    }
    
    return markerDetect_;
}

bir::MarkerDetect* bir::MarkerDetect::GetInstance()
{
    /**
     * This is a safer way to create an instance. instance = new Singleton is
     * dangeruous in case two instance threads wants to access at the same time
     */
    if(markerDetect_ == nullptr) {
        markerDetect_ = new MarkerDetect();
    }

    return markerDetect_;
}

bir::MarkerDetect::MarkerDetect(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary) {
    setDictionary(dictionary);
    
    _parameters = cv::aruco::DetectorParameters::create();
    #if (CV_VERSION_MAJOR > 3 || CV_VERSION_MAJOR == 3 && CV_VERSION_MINOR > 2)
        _parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    #else
        _parameters->doCornerRefinement = true;
    #endif
}

void bir::MarkerDetect::setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary) {
    _dictionary = cv::aruco::getPredefinedDictionary(dictionary);
}

#include <chrono>
#include <iostream>

bir::MarkerVector bir::MarkerDetect::detect(const cv::Mat& img, cv::Point2f offset) {
    MarkerVector marker_vector;

    if (img.empty()) return marker_vector;
    
    std::vector<std::vector<cv::Point2f>> rejected;

    cv::aruco::detectMarkers(img, _dictionary, marker_vector.getCorners(), marker_vector.getIDs(), _parameters, rejected);
    marker_vector.getAreas().reserve(marker_vector.getIDs().size());

    for(int index = 0; index < marker_vector.getIDs().size(); index++) {
        marker_vector.getAreas().push_back(cv::contourArea(marker_vector.getCorners().at(index)));
        
        if(offset == cv::Point2f(0, 0))
            continue;

        for (int index_offset = 0; index_offset < 4; index_offset++) {
            marker_vector.getCorners().at(index).at(index_offset) += offset;   // Add the offset take off from the image to the
                                                            // corner. Needed because the marker position
                                                            // is computed from the corner position.
        }
    }
    
    return std::move(marker_vector);
}

bir::Marker::Marker() {
    
}

bir::Marker::Marker(const bir::Marker& marker) {
    this->id_ = marker.id_;
    this->corner_ = marker.corner_;
    this->rejected_ = marker.rejected_;
}

bir::Marker::Marker(int id, const std::vector<cv::Point2f>& corner, const std::vector<cv::Point2f>& rejected) {
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

bir::MarkerVector::MarkerVector(const bir::MarkerVector& marker_vector):
    ids_(marker_vector.ids_),
    corners_(marker_vector.corners_),
    rejected_(marker_vector.rejected_),
    areas_(marker_vector.areas_)
{
}

bir::MarkerVector::MarkerVector(bir::MarkerVector&& marker_vector) noexcept:
    ids_(std::move(marker_vector.ids_)),
    corners_(std::move(marker_vector.corners_)),
    rejected_(std::move(marker_vector.rejected_)),
    areas_(std::move(marker_vector.areas_))
{
    marker_vector.ids_ = std::vector<int>();
    marker_vector.corners_ = std::vector<std::vector<cv::Point2f>>();
    marker_vector.rejected_ = std::vector<std::vector<cv::Point2f>>();
    marker_vector.areas_ = std::vector<double>();
}

bir::MarkerVector& bir::MarkerVector::operator=(MarkerVector&& marker_vector)
{
        ids_ = std::move(marker_vector.ids_),
        corners_ = std::move(marker_vector.corners_),
        rejected_ = std::move(marker_vector.rejected_),
        areas_ = std::move(marker_vector.areas_);
        return *this;
}

bir::MarkerVector::MarkerVector(    std::vector<int>& ids,
                                    std::vector<std::vector<cv::Point2f>>& corners, 
                                    std::vector<std::vector<cv::Point2f>>& rejected,
                                    std::vector<double>& areas  )
{
    this->ids_ = std::move(ids);
    this->corners_ = std::move(corners);
    this->rejected_ = std::move(rejected);
    this->areas_ = std::move(areas);
}

bir::MarkerVector::~MarkerVector() {
}

bir::Marker bir::MarkerVector::at(const int index) { 
    return bir::Marker(getIDs().at(index), getCorners().at(index), std::vector<cv::Point2f>{});
}

void bir::MarkerVector::pushBack(const bir::Marker& marker) {
    this->ids_.push_back(marker.id_);
    this->corners_.push_back(marker.corner_);
    this->rejected_.push_back(marker.rejected_);
}

void bir::MarkerVector::pushBack(bir::Marker&& marker) {
    this->ids_.push_back(marker.id_);
    this->corners_.push_back(marker.corner_);
    this->rejected_.push_back(marker.rejected_);
}

std::vector<std::vector<cv::Point2f>>& bir::MarkerVector::getCorners() {
    return this->corners_;
}

std::vector<int>& bir::MarkerVector::getIDs() {
    return this->ids_;
}