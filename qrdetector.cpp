#include "qrdetector.h"

std::vector<cv::Mat> QRDetector::findQRCodes(const cv::Mat &image) const {

    // We convert the input image into greyscale, making it easier to handle.
    cv::Mat gray(image.size(), CV_MAKETYPE(image.depth(), 1));
    cvtColor(image,gray,CV_RGB2GRAY);

    // Apply the Canny operator and store the result in a new image.
    cv::Mat edgeMap(image.size(), CV_MAKETYPE(image.depth(), 1));
    Canny(gray, edgeMap, CANNY_LOWER_THRESHOLD , CANNY_UPPER_THRESHOLD);

    // Detect contours inside the edge map and store their hierarchy.
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(edgeMap, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // For each contour, retrieve the moment and calculate the center of mass.
    std::vector<cv::Point2d> centerOfMass(contours.size());
    for( int i = 0; i < contours.size(); i++ ) {
        // retrieve the moments
        cv::Moments m = moments(contours[i], false);
        // calculate the mass center of the moments
        centerOfMass[i] = cv::Point2d(m.m10/m.m00 , m.m01/m.m00);
    }

    // Loop through hierarchy and find each contour containing two other nested contours.
    std::vector<int> positionCandidates;
    for(int i=0; i<contours.size(); i++) {
        int children = 0;
        int level = hierarchy[i][2];
        while(level >= 0) {
            level = hierarchy[level][2];
            children++;
        }

        // We accept each marker with more than 2 'onion'-layers around it.
        if(children > 3) {
            positionCandidates.push_back(i);
        }
    }

    // This vector will hold the resulting qr codes.
    std::vector<cv::Mat> result;

    //---- Debug output begin
    result.push_back(gray);
    result.push_back(edgeMap);
    //---- Debug output end

    // Do not continue, if less than three markers have been found.
    if(positionCandidates.size() < 3) {
        return result;
    }

    // Currently, only 1 qr code is supported at a time.
    // This approach simply takes the first three markers, it can find.
    std::vector<QRCode> codes;

    QRCode tmp;
    tmp.a = centerOfMass[positionCandidates[0]];
    tmp.b = centerOfMass[positionCandidates[1]];
    tmp.c = centerOfMass[positionCandidates[2]];

    codes.push_back(tmp);

    /*
    for(const QRCode& code : codes) {
        result.push_back(normalizeQRCode(image, code));
    }
    */
    // Just for now, draw teh contours being found and filtered before.
    cv::Mat drawing = cv::Mat::zeros(edgeMap.size(), CV_MAKETYPE(edgeMap.depth(), 3));
    for(int i = 0; i< positionCandidates.size(); i++) {
        int index = positionCandidates[i];
        cv::Scalar color = cv::Scalar(255, 255, 255);
        drawContours( drawing, contours, index, color, 2, 8, hierarchy, 0, cv::Point() );
        circle( drawing, centerOfMass[index], 4, color, -1, 8, 0 );
    }


    result.push_back(drawing);

    return result;
}

cv::Mat QRDetector::normalizeQRCode(const cv::Mat& image, const QRCode& code) const {
    return image;
}