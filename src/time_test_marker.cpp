#include <marker_localization/marker_detect.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <chrono>

int main(int argc, char* argv[]) {
    const char* file_path = "/home/teo/Documents/mkr_localization_ws/src/bir_marker_localization/test/resource/tag_245_246_d7_14cm.png";
    cv::Mat image = cv::imread(file_path, cv::IMREAD_GRAYSCALE);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    bir::MarkerVector marker_vector;
    for (int index = 0; index < 1000; index++)
        marker_vector = bir::MarkerDetect::GetInstance(cv::aruco::DICT_5X5_1000)->detect(image);
    
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;


}