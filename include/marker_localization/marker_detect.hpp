#ifndef BIR_MARKER_DETECT_HPP
#define BIR_MARKER_DETECT_HPP

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <numeric>

namespace bir {
    class MarkerVector; // Forward declaration of MarkerVector
    
    class MarkerDetect {
    public:
        cv::Ptr<cv::aruco::DetectorParameters> _parameters;

        MarkerDetect(const MarkerDetect& other) = delete;
        void operator=(const MarkerDetect& other) = delete;

        static MarkerDetect* markerDetect_;
        static MarkerDetect* GetInstance();
        static MarkerDetect* GetInstance(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary);
        void setDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary);
        MarkerVector detect(const cv::Mat&, cv::Point2f offset = cv::Point2f(0.0, 0.0));

    protected:
        MarkerDetect(cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary = cv::aruco::DICT_ARUCO_ORIGINAL);
        
        cv::Ptr<cv::aruco::Dictionary> _dictionary;
    };

    class Marker {
    public:
        Marker();
        Marker(const bir::Marker&);
        Marker(int, const std::vector<cv::Point2f>&, const std::vector<cv::Point2f>&);
        virtual ~Marker();

        unsigned int id_;
        std::vector<cv::Point2f> corner_;
        std::vector<cv::Point2f> rejected_;
        double area();

        bool operator ==(const int id);
    };

    class MarkerVector {        
    public:
        MarkerVector();
        MarkerVector(const bir::MarkerVector&);
        MarkerVector(bir::MarkerVector&&) noexcept;
        MarkerVector(   std::vector<int>& ids, 
                        std::vector<std::vector<cv::Point2f>>& corners, 
                        std::vector<std::vector<cv::Point2f>>& rejectes,
                        std::vector<double>& areas );
        virtual ~MarkerVector();
        
        size_t size() const {return ids_.size();}
        bool isEmpty() const {return !this->size();}
        void clear() {this->ids_.clear(); this->corners_.clear(); this->rejected_.clear();}
        bir::Marker at(const int index); // Return a Marker Object.
        void pushBack(const bir::Marker&);
        void pushBack(bir::Marker&&);
        std::vector<std::vector<cv::Point2f>>& getCorners();
        std::vector<int>& getIDs();
        std::vector<double>& getAreas() {return areas_;};
        
        MarkerVector& operator=(MarkerVector&&);

    private:
        std::vector<int> ids_;
        std::vector<std::vector<cv::Point2f>> corners_;
        std::vector<std::vector<cv::Point2f>> rejected_;
        std::vector<double> areas_;

    };

}
#endif