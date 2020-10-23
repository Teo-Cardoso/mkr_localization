#include <marker_localization/marker_detect.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <chrono>

int main(int argc, char* argv[]) {
   
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<int> a = {1, 2, 3, 4, 5};
    std::vector<int> b = std::move(a);

    std::cout << a.size() << " " << b.size() << std::endl;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "[WITHOUT STD::MOVE] Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[us]" << std::endl;
}