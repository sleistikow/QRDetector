#ifndef QR_CODE_QRDETECTOR_H
#define QR_CODE_QRDETECTOR_H

#include <opencv2/opencv.hpp>

class QRDetector {

    // Canny Operator Thresholds.
    static constexpr double CANNY_LOWER_THRESHOLD = 50.0;
    static constexpr double CANNY_UPPER_THRESHOLD = 150.0;

    // Binary imaging threshold.
    static constexpr int BINARY_THRESHOLD = 127;

    // Epsilon used for contour simplification.
    static constexpr double SIMPLFIICATION_START_EPSILON = 5.0;

    // Size of the buffer the QR code will be stored in for further calculations.
    static constexpr int QR_BUFFER_SIZE = 800;

    /**
     * This struct encodes the positions of the three markers of a QR code,
     * in relation to the image they are contained in.
     */
    struct QRCode {
        cv::Point a, b, c, d;
        float cellsize;
    };

public:

    /**
     * Finds QR code and normalize it.
     * @param image Image containing the one or more QR codes.
     * @return a list holding the normalized QR Codes.
     */
    cv::Mat findQRCode(const cv::Mat& image);

private:

    QRCode extractQRCode(const std::vector<cv::Point>& corners);
    cv::Mat alignQRCode(const cv::Mat& image, const QRCode& code) const;
    cv::Mat normalizeQRCode(const cv::Mat& image, const QRCode& code) const;
    std::vector<cv::Point> simplyfyContour(const std::vector<cv::Point>& contour) const;
    cv::Point intersect(const cv::Point& a0, const cv::Point& a1, const cv::Point& c0, const cv::Point& c1) const;

    // Members
    cv::Mat inpainting; ///< Used for debug visuals

};


#endif //QR_CODE_QRDETECTOR_H
