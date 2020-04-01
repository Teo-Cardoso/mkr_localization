#ifndef BIR_ARUCO_IDENTIFICATION
#define BIR_ARUCO_IDENTIFICATION
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <numeric>
namespace bir {
    class MarkerVector; // Forward declaration of MarkerVector
    class ArucoDetector {

        public:
            explicit ArucoDetector(int dictionary = cv::aruco::DICT_ARUCO_ORIGINAL);
            ArucoDetector(const ArucoDetector&);
            virtual ~ArucoDetector();
            

            void setParameters(cv::Ptr<cv::aruco::DetectorParameters>);
            void setPredefinedDictionary(int);
            void setCustomDictionary(int, int);
            bir::MarkerVector detect(const cv::Mat&, cv::Point2f offset = cv::Point2f(0.0, 0.0));

        private:
            cv::Ptr<cv::aruco::DetectorParameters> _parameters;
            cv::Ptr<cv::aruco::Dictionary> _dictionary;
    };

    class Marker {
        public:
            Marker();
            Marker(const bir::Marker&);
            Marker(int, std::vector<cv::Point2f>, std::vector<cv::Point2f>);
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
        MarkerVector(std::vector<int>, std::vector<std::vector<cv::Point2f>>, std::vector<std::vector<cv::Point2f>>);
        virtual ~MarkerVector();
        
        unsigned int size() const {return ids_.size();}
        bool empty() const {return !this->size();}
        void clear() {this->ids_.clear(); this->corners_.clear(); this->rejected_.clear();}
        int operator[](const int index) const; // Return the ID.
        bir::Marker at(const int index) const; // Return a Marker Object.
        void pushBack(bir::Marker);
        std::string print();
        const std::vector<std::vector<cv::Point2f>> getCorners();
        const std::vector<int> getIDs();
        private:
        friend class ArucoDetector;
        friend std::ostream& operator<<(std::ostream&, bir::MarkerVector& obj);

        std::vector<int> ids_;
        std::vector<std::vector<cv::Point2f>> corners_;
        std::vector<std::vector<cv::Point2f>> rejected_;
        std::vector<double> areas_;

    };

}
#endif