#include <string>
#include <opencv2/opencv.hpp>

#include "qrdetector.h"

int main(int argc, char** argv) {

    // Create a device for capturing images.
    cv::VideoCapture capture(0); // 0 taking the device being found first

    // This will hold the captured images.
    cv::Mat image;

    // Initialize an instance of our QR Detector.
    QRDetector detector;

    // Main Loop, quit on keypress.
    while (cv::waitKey(1) != 'q') {

        // Capture a new image.
        capture >> image;

        // Extract the qr code and show the result.
        std::vector<cv::Mat> result = detector.findQRCodes(image);
        for(int i = 0; i < result.size(); i++) {
            cv::imshow("Output_" + std::to_string(i), result[i]);
        }

    }

    return 0;
}