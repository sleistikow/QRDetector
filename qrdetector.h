#ifndef QR_CODE_QRDETECTOR_H
#define QR_CODE_QRDETECTOR_H

#include <opencv2/opencv.hpp>

class QRDetector {

    static constexpr double CANNY_LOWER_THRESHOLD = 50.0;
    static constexpr double CANNY_UPPER_THRESHOLD = 150.0;

    /**
     * This struct encodes the positions of the three markers of a QR code,
     * in relation to the image they are contained in.
     */
    struct QRCode {
        cv::Point a, b, c;
    };

public:

    /**
     * Finds QR codes and normalizes them.
     * @param image Image containing the one or more QR codes.
     * @return a list holding the normalized QR Codes.
     */
    std::vector<cv::Mat> findQRCodes(const cv::Mat& image) const;

private:

    cv::Mat normalizeQRCode(const cv::Mat& image, const QRCode& code) const;

};


#endif //QR_CODE_QRDETECTOR_H
