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
static const int DEFAULT_IMAGE_UPSCALE = 1000;
static const char* DEFAULT_INPUT = "qr.jpg";
static const char* DEFAULT_OUTPUT = "out.png";
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

    int size = DEFAULT_IMAGE_UPSCALE;

    int opt = 0;
    while((opt = getopt(argc, argv, "s:")) != -1) {
        switch (opt) {
        case 's':
            size = atoi(optarg);
            break;
        default: /* '?' */
            std::cout << "Usage: " << argv[0] << "[-s size]\n" << std::endl;
            return EXIT_FAILURE;
        }
    }

    // Parse command line arguments
    const char* input = DEFAULT_INPUT;
    const char* output = DEFAULT_OUTPUT;
    if(argc > 2) {
        input = argv[1];
        output = argv[2];
    }

    // This will hold the captured images.
    cv::Mat image;

    // Initialize an instance of our QR Detector.
    QRDetector detector;

#ifdef MODE_WEBCAM
    // Create a device for capturing images.
    cv::VideoCapture capture(0); // 0 - taking the first device being found
#else
    // Load the image.
    image = cv::imread(input);
    if(image.empty()) {
        std::cout << "Failed loading image!" << std::endl;
        return EXIT_FAILURE;
    }

    // Resize the image. FIXME: not necessary any longer?
    //image = resizeImage(image, size);
#endif

#ifndef MODE_RELEASE
    // Main Loop, quit on keypress.
    while (cv::waitKey(1) != 'q') {

#ifdef MODE_WEBCAM
        // Capture a new image.
        capture >> image;
#endif
        // Extract the qr code and show the result.
        cv::Mat result = detector.detectQRCode(image);
        if(!result.empty())
            cv::imshow("result", result);
        else
            cv::imshow("input", image);
    }

#else

    cv::Mat qr = detector.detectQRCode(image);
    if(qr.empty()) {
        std::cout << "QR Code not found!" << std::endl;
        qr = cv::Mat::zeros(1, 1, CV_8UC1);
    } else
        std::cout << "QR Code found!" << std::endl;

    // Deactivate compression.
    std::vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(0);

    if(cv::imwrite(output, qr, params))
        std::cout << "QR Code successfully written!" << std::endl;
    else
        std::cout << "Writing QR Code failed!" << std::endl;

#endif

    return EXIT_SUCCESS;
}
