#include <string>
#include <algorithm>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "qrdetector.h"

//////////////////////////////////////////////////////
/// Preprocessor definition section
///
//#define MODE_WEBCAM // uncomment this line to use webcam
#define MODE_RELEASE // uncomment this line for evaluation build
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
/// Constants definition section
///
static const int DEFAULT_MAX_IMAGE_SIZE = 500;
//////////////////////////////////////////////////////

#if defined(MODE_WEBCAM) && defined(MODE_RELEASE)
#error "Both MODE_WEBCAM and MODE_RELEASE are defined"
#endif

// http://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
cv::Mat resizeImage(const cv::Mat& img, int newSize) {
    int maxDim = std::max(img.rows, img.cols);
    double scale = newSize / static_cast<double>(maxDim);

    cv::Mat scaledImg;
    
    int interpolater = (scale > 1) ? CV_INTER_LINEAR : CV_INTER_AREA;
    cv::resize(img, scaledImg, cv::Size(), scale, scale, interpolater);

    return scaledImg;
}

int main(int argc, char** argv) {

    int scaleSize = DEFAULT_MAX_IMAGE_SIZE;

    int opt = 0;
    while((opt = getopt(argc, argv, "s:")) != -1) {
        switch (opt) {
        case 's':
            scaleSize = atoi(optarg);
            break;
        default: /* '?' */
            std::cout << "Usage: " << argv[0] << "[-s size]\n" << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Parse command line arguments
    const char* input = "qr.jpg";
    const char* output = "out.jpg";
    if(argc > 2) {
        input = argv[1];
        output = argv[2];
    }

    // This will hold the captured images.
    cv::Mat image;

#ifdef MODE_WEBCAM
    // Create a device for capturing images.
    cv::VideoCapture capture(0); // 0 taking the device being found first
#else
    // Load the image
    image = cv::imread(input);
    if(image.rows == 0 || image.cols == 0) {
        std::cout << "Failed loading image!" << std::endl;
        return EXIT_FAILURE;
    }

    image = resizeImage(image, scaleSize);
#endif

    // Initialize an instance of our QR Detector.
    QRDetector detector;

#ifndef MODE_RELEASE
    // Main Loop, quit on keypress.
    while (cv::waitKey(1) != 'q') {

#ifdef MODE_WEBCAM
        // Capture a new image.
        capture >> image;
#endif

        // Extract the qr code and show the result.
        cv::Mat result = detector.findQRCode(image);
        if(result.cols > 0 && result.rows > 0)
            cv::imshow("QRCode", result);
        //else
        //    cv::destroyWindow("QRCode");
    }

#else

    cv::Mat qr = detector.findQRCode(image);
    if(cv::imwrite(output, qr))
        std::cout << "QR Code successfully written!" << std::endl;
    else
        std::cout << "Writing QR Code failed!" << std::endl;

#endif

    return EXIT_SUCCESS;
}
