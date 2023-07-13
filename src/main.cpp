#include <iostream>
#include <opencv2/opencv.hpp>
#include "Detector.hpp"

using namespace std;
using namespace cv;

int main() {
    Detector detector;
    detector.run();
    return 0;
}
