#ifndef QR_CODE_QRDETECTOR_H
#define QR_CODE_QRDETECTOR_H

#include <opencv2/opencv.hpp>

class QRDetector {

    // Canny Operator Thresholds.
    static constexpr double CANNY_LOWER_THRESHOLD = 50.0;
    static constexpr double CANNY_UPPER_THRESHOLD = 150.0;

    // Median filter neighbourhood.
    static constexpr int MEDIAN_BLUR_NEIGHBOURHOOD = 5;

    // Binary imaging threshold.
    static constexpr int BINARY_THRESHOLD = 127;

    // Epsilon used for contour simplification.
    static constexpr double SIMPLFIICATION_START_EPSILON = 5.0;

    // Min. ratio between bounding rect of a marker and the actual area.
    static constexpr double MAX_AREA_THRESHOLD = 0.1;

    // Size of the buffer the QR code will be stored in for further calculations.
    static constexpr int QR_BUFFER_SIZE = 800;

    /**
     * This struct encodes the positions of the three markers of a QR code,
     * in relation to the image they are contained in.
     */
    struct QRCode {
        cv::Point a, b, c, d;
        std::vector<std::vector<cv::Point>> patterns;
        float cellsize;
    };

public:

    // Debug Details
    enum {
        DEBUG_NONE      = 0,
        DEBUG_BINARY    = 1 << 1,
        DEBUG_EDGE      = 1 << 2,
        DEBUG_AUGMENTED = 1 << 3,
        DEBUG_ALIGNED   = 1 << 4,
    };

    /**
     * Finds QR code and normalize it.
     * @param input Image containing the one or more QR codes.
     * @return a list holding the normalized QR Codes.
     */
    cv::Mat detectQRCode(const cv::Mat& image, int debug = DEBUG_NONE) const;

private:

    QRCode extractQRCode(const std::vector<std::vector<cv::Point>>& patterns, cv::Mat& augmented) const;
    cv::Mat alignQRCode(const cv::Mat& image, QRCode& code) const;
    cv::Mat normalizeQRCode(const cv::Mat& image, const QRCode& code) const;
    std::vector<cv::Point> simplifyContour(const std::vector<cv::Point>& contour, int numPoints) const;
    cv::Point intersect(const cv::Point& a0, const cv::Point& a1, const cv::Point& c0, const cv::Point& c1) const;
};


#endif //QR_CODE_QRDETECTOR_H
