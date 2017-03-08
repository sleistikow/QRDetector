#ifndef QR_CODE_QRDETECTOR_H
#define QR_CODE_QRDETECTOR_H

#include <opencv2/opencv.hpp>

class QRDetector {

    // Canny Operator Thresholds.
    static constexpr double CANNY_LOWER_THRESHOLD = 50.0;
    static constexpr double CANNY_UPPER_THRESHOLD = 180.0;

    // Median filter neighbourhood.
    static constexpr int MEDIAN_BLUR_NEIGHBOURHOOD = 3;

    // Binary imaging threshold.
    static constexpr int BINARY_THRESHOLD = 127;

    // Epsilon used for contour simplification.
    static constexpr double SIMPLFIICATION_START_EPSILON = 1.0;

    // Min. ratio between bounding rect of a marker and the actual area.
    static constexpr double MAX_AREA_THRESHOLD = 10.0;

    /**
     * This struct encodes the positions of the three markers of a QR code,
     * in relation to the image they are contained in.
     */
    struct QRCode {
        cv::Point a, b, c, d;
        std::vector<std::vector<cv::Point>> patterns;
        float moduleSize;
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
     * @param debug Optional debug flags
     * @return a list holding the normalized QR Codes.
     */
    cv::Mat detectQRCode(const cv::Mat& image, int debug = DEBUG_NONE) const;

private:

    //! @brief Given the marker contours, this function will locate the QRCode.
    QRCode extractQRCode(const std::vector<std::vector<cv::Point>>& patterns, cv::Mat& augmented) const;

    //! @brief Transforms the QR-Code onto the xy-plane.
    cv::Mat alignQRCode(const cv::Mat& image, QRCode& code) const;

    //! @brief Normalizes the given QR-Code, including binarization and shrinking to min. size.
    cv::Mat normalizeQRCode(const cv::Mat& image, const QRCode& code) const;

    //! @brief Simplifies a given contour to ultimately consisting out of max. the specified amount of points.
    std::vector<cv::Point> simplifyContour(const std::vector<cv::Point>& contour, int numPoints, float* const eps = nullptr) const;

    //! @brief Counts the number of switches in color, when moving across the image along a line given by start and end.
    int countColorSwitch(const cv::Mat& image, const cv::Point& start, const cv::Point& end) const;

    //! @brief Picks a pixel out of image taking a neighbourhood of k into account.
    uchar pickMeanPixel(const cv::Mat& image, const cv::Point& position, int k) const;

    //! @brief Calculates the intersection point betwee, two lines defined by a0a1 and c0c1, respectively.
    cv::Point intersect(const cv::Point& a0, const cv::Point& a1, const cv::Point& c0, const cv::Point& c1) const;
};


#endif //QR_CODE_QRDETECTOR_H
