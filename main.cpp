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
static const int MIN_IMAGE_DIMENSION = 1000; // Default upscale for input images.
static const char* DEFAULT_INPUT = "qr.jpg";
static const char* DEFAULT_OUTPUT = "out.png";
static const char* DEFAULT_REFERENCE = nullptr;
static const char* DEFAULT_DIFF = "diff.png";
//////////////////////////////////////////////////////

#if defined(MODE_WEBCAM) && defined(MODE_RELEASE)
#error "Both MODE_WEBCAM and MODE_RELEASE are defined"
#endif

/**
 * Resizes the input image, that the smallest dimension matches
 * the <p>minDimension</p> and the aspect ratio is preserved.
 * @param image the image being about to be resized
 * @return the resized image
 */
cv::Mat resizeImage(const cv::Mat& img, int minDimension) {
    int maxDim = std::max(img.rows, img.cols);
    double scale = minDimension / static_cast<double>(maxDim);

    cv::Mat scaledImg;

    int interpolater = (scale > 1) ? CV_INTER_LINEAR : CV_INTER_AREA;
    cv::resize(img, scaledImg, cv::Size(), scale, scale, interpolater);

    return scaledImg;
}

int main(int argc, char** argv) {

    // Parse command line arguments
    const char* input = DEFAULT_INPUT;
    const char* output = DEFAULT_OUTPUT;
    const char* reference = DEFAULT_REFERENCE;
    if(argc > 2) {
        input = argv[1];
        output = argv[2];
        if(argc > 3)
            reference = argv[3];
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

    image = resizeImage(image, MIN_IMAGE_DIMENSION);
#endif

#ifndef MODE_RELEASE

    // Configure debug output.
    int debug = QRDetector::DEBUG_ALIGNED | QRDetector::DEBUG_AUGMENTED | QRDetector::DEBUG_BINARY | QRDetector::DEBUG_EDGE;

    // Main Loop, quit on keypress.
    while (cv::waitKey(1) != 'q') {

#ifdef MODE_WEBCAM
        // Capture a new image.
        capture >> image;
#endif
        // Extract the qr code and show the result.
        cv::Mat result = detector.detectQRCode(image, debug);
        if(!result.empty())
            cv::imshow("result", result);
        else
            cv::imshow("input", image);
    }

#else

    bool codeFound = false;

    cv::Mat qr = detector.detectQRCode(image);
    if(qr.empty()) {
        std::cout << "QR Code not found!" << std::endl;
        qr = cv::Mat::zeros(1, 1, CV_8UC1);
    } else {
        std::cout << "QR Code found!" << std::endl;
        codeFound = true;
    }

    // Deactivate compression.
    std::vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(0);

    if(cv::imwrite(output, qr, params))
        std::cout << "Result successfully written!" << std::endl;
    else
        std::cout << "Writing Result failed!" << std::endl;

    // Compare with reference image.
    if(reference && codeFound) {
        bool equals = true;
        std::cout << "Comparison yields: ";
        cv::Mat ref = cv::imread(reference, cv::IMREAD_GRAYSCALE);
        if(ref.cols == qr.cols && ref.rows == qr.rows && ref.type() == qr.type()) {
            cv::Mat diff = cv::Mat::zeros(ref.rows, ref.cols, CV_8UC1);
            for(int r = 0; r < ref.rows; r++) {
                for(int c = 0; c < ref.cols; c++) {
                    if(ref.at<uchar>(r, c) != qr.at<uchar>(r, c)) {
                        diff.at<uchar>(r, c) = 255;
                        equals = false;
                    }
                }
            }
            if(!equals) {
                if(cv::imwrite(DEFAULT_DIFF, diff, params))
                    std::cout << "diff file successfully written." << std::endl;
                else
                    std::cout << "writing diff file failed." << std::endl;
            } else
                std::cout << "perfect match!" << std::endl;

            return EXIT_SUCCESS;
        } else {
            std::cout << "size or type not equals." << std::endl;
            return EXIT_FAILURE;
        }
    } else if(!codeFound)
        return EXIT_FAILURE;

#endif

    return EXIT_SUCCESS;
}
