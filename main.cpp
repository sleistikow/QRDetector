#include <string>
#include <algorithm>
#include <unistd.h>
#include <opencv2/opencv.hpp>

#include "qrdetector.h"

// uncomment the following line to use the webcam
//#define USE_WEBCAM

// http://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
cv::Mat resizeImage(const cv::Mat& img, int newSize=500) {
    int maxDim = std::max(img.rows, img.cols);
    double scale = newSize / (double) maxDim;

    cv::Mat scaledImg;
    
    int interpolater = (scale > 1) ? CV_INTER_LINEAR : CV_INTER_AREA;
    cv::resize(img, scaledImg, cv::Size(), scale, scale, interpolater);

    return scaledImg;
}

int main(int argc, char** argv) {

    int scaleSize = 500;
    int opt = 0;
    
    while((opt = getopt(argc, argv, "s:")) != -1) {
        switch (opt) {
        case 's':
            scaleSize = atoi(optarg);;
            break;
        default: /* '?' */
            std::cout << "Usage: " << argv[0] << "[-s size]\n" << std::endl;
            return EXIT_FAILURE;
        }
    }


    // This will hold the captured images.
    cv::Mat image;

#ifdef USE_WEBCAM
    // Create a device for capturing images.
    cv::VideoCapture capture(0); // 0 taking the device being found first
#else
    // Load the image
    image = cv::imread("qr.jpg");
    if(image.rows == 0 || image.cols == 0) {
        std::cout << "Failed loading image" << std::endl;
        return EXIT_FAILURE;
    }

    image = resizeImage(image, scaleSize);
    
    /*
     * TODO: The program does not work when the input image is large.
     * In this case scale the image down to a resolution around 800x600,
     * preserving the aspect ration (!).
     */
    //float ratio = static_cast<float>(image.rows) / image.cols;
    //CvSize size = cvSize(600, static_cast<int>(600 / ratio));
    //cv::Mat image = cv::Mat::zeros(edgeMap.size(), CV_MAKETYPE(edgeMap.depth(), 3));
    //cvResize(, image, CV_INTER_CUBIC);
#endif


    // Initialize an instance of our QR Detector.
    QRDetector detector;

    // Main Loop, quit on keypress.
    while (cv::waitKey(1) != 'q') {

#ifdef USE_WEBCAM
        // Capture a new image.
        capture >> image;
#endif

        // Extract the qr code and show the result.
        std::vector<cv::Mat> result = detector.findQRCodes(image);
        for(int i = 0; i < result.size(); i++) {
            cv::imshow("Output_" + std::to_string(i), result[i]);
        }

    }

    return EXIT_SUCCESS;
}
